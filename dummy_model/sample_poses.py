#!/usr/bin/env python3
"""Sample physics-settled dummy poses on a goal tunnel.

The rulebook constrains the pose space far more than "any pose anywhere":

    "The figure's pelvis must be in contact with the top surface of the goal"
    "Poses that obstruct ball passage are not allowed"

So a valid pose is: pelvis resting on a 200 x 195 mm tunnel top, torso and
limbs free, nothing dangling in front of the bore.  That is a small, well
defined manifold, which is why 2000 samples covers it densely.

Poses are *adversarial*, not random -- the opposing team chooses the pose of
the robot figure on your field, and will pick whatever defeats a classifier.
Their only weapon is self-occlusion (appearance is fixed), so a fraction of
the samples is deliberately biased toward compact, curled, heavily
self-occluding configurations rather than sampling uniformly.

Output is JSONL, one settled pose per line:
    {"id", "hard", "qpos", "root", "links": {name: [x,y,z, qw,qx,qy,qz]},
     "bbox_local", "settled_after"}
Link transforms are expressed relative to the centre of the goal's top face,
so Blender can drop them onto any goal without rework.  Joint angles are kept
alongside so the same pose can drive an armature (needed for the garments).

Usage:
    python3 sample_poses.py --n 2000 --workers 16 --out poses.jsonl
"""

import argparse
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from multiprocessing import Pool

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# Goal tunnel, from the user's tape measurements.
TUNNEL_W, TUNNEL_D = 0.200, 0.195
TUNNEL_H = 0.201                      # mean of the 191 / 212 mm edges
WALL = 0.012
FIELD_SLOPE = math.radians(5.0)

HARD_FRACTION = 0.30
# Settling criterion.  Velocity-based, not energy-based: MuJoCo leaves a
# residual jitter floor around |qvel| ~ 1e-3 that never decays, so an energy
# threshold below that floor is unreachable and every sample "fails to settle"
# (measured: floor of 1.2e-6 against a 2e-7 threshold -> 0 % acceptance).
SETTLE_MAX_S = 5.0
SETTLE_VEL = 0.010                    # rad/s or m/s, sustained
SETTLE_VEL_TIMEOUT = 0.050            # accept-on-timeout if slower than this
DROP_H = 0.010                        # start just above the surface

# Links whose transforms we export (the real ones; intermediates are massless
# frames with no geometry and are of no use to the renderer).
REAL_LINKS = [
    "pelvis", "chest", "head",
    "upper_arm_l", "forearm_l", "hand_l",
    "upper_arm_r", "forearm_r", "hand_r",
    "thigh_l", "shin_l", "foot_l",
    "thigh_r", "shin_r", "foot_r",
]


# --------------------------------------------------------------------------
# scene
# --------------------------------------------------------------------------

def build_scene(dummy_scene=os.path.join(HERE, "dummy_scene.xml"),
                friction_scale=1.0):
    """dummy_scene.xml plus a goal tunnel sitting on the sloped field.

    The tunnel top is level (the boxes are built square, the field is what
    slopes), which is why the user measured 191 mm on the low edge and 212 mm
    on the high edge.  We model the top face as horizontal at z = TUNNEL_H and
    let the field slope under it.
    """
    tree = ET.parse(dummy_scene)
    root = tree.getroot()
    wb = root.find("worldbody")

    # friction_scale > 1 strengthens joint stiction for the POSED sampler.
    # The solver enforces frictionloss imperfectly on held-up limbs -- they
    # creep at 0.01-0.15 rad/s where the real toy holds indefinitely (photo
    # _12) -- so for deliberately-set poses the scaled model is closer to the
    # truth.  Drop-and-settle keeps 1.0: its sprawls were validated at that
    # calibration, and its limbs end up supported rather than held.
    if friction_scale != 1.0:
        for j in root.iter("joint"):
            fl = j.get("frictionloss")
            if fl is not None:
                j.set("frictionloss", f"{float(fl) * friction_scale:.6f}")
            j.set("damping", "0.002")

    # Remove the preview floor; the field replaces it.
    for g in list(wb.findall("geom")):
        if g.get("name") == "floor":
            wb.remove(g)

    # Torsional / rolling friction raised well above the MuJoCo defaults:
    # with the stock 0.001 rolling coefficient the figure ROCKS on its rounded
    # pelvis for the whole 5 s settle window (residual qvel 0.07-0.17 rad/s)
    # and every seated pose times out.  Real wood on painted ply damps such
    # rocking in a couple of oscillations.
    ET.SubElement(wb, "geom", {
        "name": "field", "type": "plane", "size": "1.5 2 0.05",
        "pos": f"0 0 {-TUNNEL_H:.4f}", "euler": f"{math.degrees(FIELD_SLOPE):.3f} 0 0",
        "rgba": "0.62 0.78 0.13 1", "condim": "6", "friction": "1.0 0.15 0.005",
    })

    # Tunnel: two side walls and a roof.  The bore stays open so a limb that
    # blocks ball passage is geometrically detectable rather than assumed away.
    hw, hd = TUNNEL_W / 2, TUNNEL_D / 2
    tun = ET.SubElement(wb, "body", {"name": "goal", "pos": "0 0 0"})
    for sx in (-1, 1):
        ET.SubElement(tun, "geom", {
            "type": "box",
            "size": f"{WALL/2:.4f} {hd:.4f} {TUNNEL_H/2:.4f}",
            "pos": f"{sx*(hw-WALL/2):.4f} 0 {-TUNNEL_H/2:.4f}",
            "rgba": "0.93 0.93 0.91 1", "condim": "3",
            "friction": "0.7 0.10 0.004",
        })
    ET.SubElement(tun, "geom", {
        "name": "goal_top", "type": "box",
        "size": f"{hw:.4f} {hd:.4f} {WALL/2:.4f}",
        "pos": f"0 0 {-WALL/2:.4f}",
        "rgba": "0.95 0.95 0.93 1", "condim": "6",
        "friction": "0.8 0.15 0.005",
    })
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


# --------------------------------------------------------------------------
# pose sampling
# --------------------------------------------------------------------------

def sample_swing(rng, lo_x, hi_x, lo_y, hi_y):
    """A swing direction distributed uniformly over the reachable cap.

    Sampling the two swing angles independently and uniformly over-represents
    directions near the poles: the Jacobian of the Euler parameterisation is
    not constant.  Drawing the polar angle with density proportional to
    sin(theta) and the azimuth uniformly fixes that, so the limb points in a
    genuinely uniform spread of directions.
    """
    max_x = min(abs(lo_x), abs(hi_x))
    max_y = min(abs(lo_y), abs(hi_y))
    theta_max = max(max_x, max_y)
    # Inverse-CDF for density ~ sin(theta) on [0, theta_max].
    u = rng.random()
    theta = math.acos(1.0 - u * (1.0 - math.cos(theta_max)))
    phi = rng.uniform(0.0, 2.0 * math.pi)
    ax = theta * math.cos(phi)
    ay = theta * math.sin(phi)
    return (min(max(ax, lo_x), hi_x), min(max(ay, lo_y), hi_y))


def between(rng, a, b):
    """rng.uniform with the bounds sorted.

    Sampling "from 65 % of a limit to the limit" reverses the bounds whenever
    that limit is negative, which numpy rejects outright.  Ordering here means
    callers can write the intent directly.
    """
    lo, hi = (a, b) if a <= b else (b, a)
    return rng.uniform(lo, hi)


def sample_qpos(m, rng, hard):
    """One joint configuration.

    hard=True biases toward compact, self-occluding poses: elbows and knees
    deeply flexed, limbs drawn in toward the torso, waist curled.  These are
    what an opponent picks to defeat a classifier, and uniform sampling
    produces almost none of them.
    """
    q = np.zeros(m.nq)
    q[3] = 1.0  # root quaternion w

    def jrange(name):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        return jid, m.jnt_qposadr[jid], m.jnt_range[jid]

    def setj(name, val):
        _, adr, (lo, hi) = jrange(name)
        q[adr] = min(max(val, lo), hi)

    for side in ("l", "r"):
        # --- shoulder / hip: twist + swing ---------------------------------
        for key, elbow_knee in ((f"shoulder_{side}", f"elbow_{side}"),
                                (f"hip_{side}", f"knee_{side}")):
            _, _, (tlo, thi) = jrange(f"{key}_z")
            _, _, (xlo, xhi) = jrange(f"{key}_x")
            _, _, (ylo, yhi) = jrange(f"{key}_y")
            setj(f"{key}_z", rng.uniform(tlo * 0.5, thi * 0.5) if hard
                 else rng.uniform(tlo, thi))
            if hard:
                # Curl the limb against the torso.  Note this needs a LARGE
                # swing, not a small one: the zero pose has the limbs hanging
                # in line with the body, so a small swing leaves them extended.
                # Folding a knee to the chest or an arm across the body is
                # near-maximal flexion about the primary swing axis.
                edge = yhi if rng.random() < 0.5 else ylo
                sy = between(rng, 0.65 * edge, edge)
                sx = between(rng, xlo * 0.25, xhi * 0.25)
            else:
                sx, sy = sample_swing(rng, xlo, xhi, ylo, yhi)
            setj(f"{key}_x", sx)
            setj(f"{key}_y", sy)

            # --- elbow / knee ----------------------------------------------
            _, _, (klo, khi) = jrange(elbow_knee)
            if hard:
                # Deep flexion, in whichever direction this joint bends.
                setj(elbow_knee, between(rng, *((0.55 * khi, khi) if khi > 0
                                                else (klo, 0.55 * klo))))
            else:
                setj(elbow_knee, rng.uniform(klo, khi))

        for j in (f"wrist_{side}", f"ankle_{side}"):
            _, _, (lo, hi) = jrange(j)
            setj(j, rng.uniform(lo, hi))

    # --- waist / neck ------------------------------------------------------
    for key, axes in (("waist", "zxy"), ("neck", "zxy")):
        for a in axes:
            _, _, (lo, hi) = jrange(f"{key}_{a}")
            if hard and a in "xy":
                # Curl the torso / tuck the head, hiding the bare-wood head
                # which is the strongest human cue.
                edge = hi if rng.random() < 0.5 else lo
                setj(f"{key}_{a}", between(rng, 0.55 * edge, edge))
            else:
                setj(f"{key}_{a}", rng.uniform(lo, hi))
    return q


def _fk_sign(m, d, joint, body, axis=0, orient=False):
    """+1 if a positive joint angle moves `body` toward +axis, else -1.

    The ball chains carry an rpy=pi flip whose effect on axis signs is easy to
    get wrong by inspection (and was, twice, in the hard-case sampler).  A
    half-radian probe through forward kinematics settles it empirically.

    orient=True reads the tilt of the body's +Z axis instead of its position.
    Needed for the head: its frame origin sits exactly on the neck joint, so
    no neck rotation ever translates it.
    """
    d.qpos[:] = 0.0
    d.qpos[3] = 1.0
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)

    def read():
        mujoco.mj_forward(m, d)
        if orient:
            return d.xmat[bid].reshape(3, 3)[axis, 2]   # +Z axis, world component
        return d.xpos[bid][axis]

    d.qpos[m.jnt_qposadr[jid]] = 0.5
    x1 = read()
    d.qpos[m.jnt_qposadr[jid]] = 0.0
    x0 = read()
    return 1.0 if x1 > x0 else -1.0


def probe_signs(m, d):
    """Empirical joint-sign table for the posed sampler, one FK probe each."""
    return {
        "hip_fwd": _fk_sign(m, d, "hip_l_y", "shin_l"),
        "sh_fwd": _fk_sign(m, d, "shoulder_l_y", "hand_l"),
        "waist_fwd": _fk_sign(m, d, "waist_y", "head"),
        "neck_fwd": _fk_sign(m, d, "neck_y", "head", orient=True),
    }


def sample_posed_qpos(m, d, rng, hard, signs):
    """A deliberately-set pose: what a person makes the figure do.

    The venue photos (172514/172618 bursts) show what teams actually place:
    the figure SITTING UPRIGHT on the goal, arms raised or spread, legs
    dangling over a face.  Drop-and-settle almost never produces raised arms
    -- stiction can hold an arm up, but only a hand puts it there -- so this
    samples expressive joint configurations directly and lets a short settle
    make them physical.

    hard=True is the adversarial variant of the same act: still seated
    (pelvis contact is a rule), but curled -- head tucked, arms crossed over
    the face, knees up -- hiding the wood/silver cues.
    """
    q = np.zeros(m.nq)
    q[3] = 1.0

    def setj(name, val):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        lo, hi = m.jnt_range[jid]
        q[m.jnt_qposadr[jid]] = min(max(val, lo), hi)

    def U(a, b):
        return rng.uniform(a, b)

    deg = math.radians
    fwd = signs["hip_fwd"]          # hip_y sign that swings the leg forward

    # --- torso: upright with a lean; hard curls it over ---------------------
    if hard:
        setj("waist_x", U(deg(15), deg(35)) * (1 if rng.random() < 0.5 else -1))
        setj("waist_y", signs["waist_fwd"] * U(deg(15), deg(35)))   # hunch forward
        setj("neck_y", signs["neck_fwd"] * U(deg(20), deg(40)))     # tuck the head
    else:
        setj("waist_z", U(-deg(30), deg(30)))
        setj("waist_x", U(-deg(12), deg(12)))
        # Slight backward bias: forward hunch plus raised arms tips the
        # figure onto its face.
        setj("waist_y", signs["waist_fwd"] * U(-deg(20), deg(10)))
        setj("neck_z", U(-deg(40), deg(40)))
        setj("neck_x", U(-deg(15), deg(15)))
        setj("neck_y", U(-deg(20), deg(25)))

    # --- arms ---------------------------------------------------------------
    for side in ("l", "r"):
        s = 1 if side == "l" else -1
        if hard:
            # Crossed over the chest / face.
            setj(f"shoulder_{side}_z", U(-deg(40), deg(40)))
            setj(f"shoulder_{side}_x", -s * U(deg(30), deg(70)))   # across body
            setj(f"shoulder_{side}_y", U(deg(30), deg(80)) * signs["sh_fwd"])
            setj(f"elbow_{side}", U(deg(80), deg(150)))
        else:
            style = rng.choice(["up", "out", "fwd", "down", "akimbo"])
            if style == "up":
                # Both swing axes near their limits compose to ~arm overhead.
                setj(f"shoulder_{side}_x", s * U(deg(55), deg(85)))
                setj(f"shoulder_{side}_y", U(deg(65), deg(95)) *
                     (1 if rng.random() < 0.7 else -1))
                setj(f"elbow_{side}", U(0, deg(30)))
            elif style == "out":
                setj(f"shoulder_{side}_x", s * U(deg(55), deg(85)))
                setj(f"shoulder_{side}_y", U(-deg(20), deg(20)))
                setj(f"elbow_{side}", U(0, deg(45)))
            elif style == "fwd":
                setj(f"shoulder_{side}_x", s * U(-deg(15), deg(15)))
                setj(f"shoulder_{side}_y", signs["sh_fwd"] * U(deg(50), deg(95)))
                setj(f"elbow_{side}", U(0, deg(50)))
            elif style == "akimbo":
                setj(f"shoulder_{side}_x", s * U(deg(20), deg(50)))
                setj(f"shoulder_{side}_y", U(-deg(20), deg(20)))
                setj(f"elbow_{side}", U(deg(80), deg(150)))
            else:
                setj(f"shoulder_{side}_x", s * U(-deg(10), deg(25)))
                setj(f"shoulder_{side}_y", U(-deg(25), deg(25)))
                setj(f"elbow_{side}", U(0, deg(60)))
            setj(f"shoulder_{side}_z", U(-deg(45), deg(45)))
        setj(f"wrist_{side}", U(-deg(45), deg(45)))

    # --- legs: seated -------------------------------------------------------
    # Legs mostly act as a pair -- the venue sit has BOTH shins hanging over
    # the rim.  Independent per-leg styles made that pairing rare.
    dangles = False
    both_dangle = rng.random() < 0.45
    for side in ("l", "r"):
        s = 1 if side == "l" else -1
        if hard:
            # Knees drawn up in front of the torso.
            setj(f"hip_{side}_y", fwd * U(deg(70), deg(90)))
            setj(f"hip_{side}_x", s * U(0, deg(25)))
            setj(f"knee_{side}", -U(deg(90), deg(150)))
        else:
            style = "dangle" if both_dangle else \
                rng.choice(["dangle", "splay", "tuck", "straight"])
            if style == "dangle":
                # Thigh forward over the edge, shin hanging.
                dangles = True
                setj(f"hip_{side}_y", fwd * U(deg(60), deg(90)))
                setj(f"hip_{side}_x", s * U(0, deg(20)))
                setj(f"knee_{side}", -U(deg(50), deg(110)))
            elif style == "splay":
                setj(f"hip_{side}_y", fwd * U(deg(40), deg(80)))
                setj(f"hip_{side}_x", s * U(deg(20), deg(60)))
                setj(f"knee_{side}", -U(0, deg(60)))
            elif style == "tuck":
                setj(f"hip_{side}_y", fwd * U(deg(75), deg(90)))
                setj(f"hip_{side}_x", s * U(0, deg(15)))
                setj(f"knee_{side}", -U(deg(110), deg(150)))
            else:
                setj(f"hip_{side}_y", fwd * U(deg(70), deg(90)))
                setj(f"knee_{side}", -U(0, deg(30)))
        setj(f"ankle_{side}", U(-deg(30), deg(30)))
    return q, dangles


def _link_positions(m, d):
    out = np.empty((len(REAL_LINKS), 3))
    for i, name in enumerate(REAL_LINKS):
        out[i] = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)]
    return out


def posed_root(rng, q, dangles=False):
    """Seat the pelvis upright on the top face, as a person places it.

    The lean direction is biased BACKWARD (tilt axis near -Y in the pelvis
    frame): seated with legs forward, a slight backward lean is the stable
    configuration, and it is what a person leaves the figure in.  An unbiased
    direction made half the samples pitch forward onto their face -- top-heavy
    with raised arms -- which lifts the pelvis and forfeits the sample.
    """
    # Every seated style extends the legs forward, so facing a mouth face
    # (+-Y) usually ends with a shin drooping into the ball passage and the
    # sample rejected -- a waste, not a wrong answer.  Bias toward facing the
    # side WALLS; the minority that still faces a mouth keeps legal
    # over-the-mouth variety (blocks_bore arbitrates).
    if rng.random() < 0.75:
        yaw = rng.choice([0.0, math.pi]) + rng.uniform(-0.7, 0.7)
    else:
        yaw = rng.uniform(0, 2 * math.pi)
    # Mostly near-vertical; occasionally a recline as if propped on its hands.
    tilt = rng.uniform(0, math.radians(12)) if rng.random() < 0.8 \
        else rng.uniform(math.radians(12), math.radians(24))
    tdir = -math.pi / 2 + rng.normal(0.0, math.radians(60))
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    ct, st = math.cos(tilt / 2), math.sin(tilt / 2)
    ax, ay = math.cos(tdir) * st, math.sin(tdir) * st
    q[3:7] = [cz * ct, cz * ax - sz * ay, cz * ay + sz * ax, sz * ct]

    if dangles:
        # Seat the pelvis AT the rim it faces, thighs past the edge, shins
        # hanging -- the venue-photo sit.  Centred placement instead presses
        # the bent shins into the top, which levers the torso onto its back:
        # the first pass produced almost only reclining figures because of it.
        fx, fy = math.cos(yaw), math.sin(yaw)
        half = TUNNEL_W / 2 if abs(fx) > abs(fy) else TUNNEL_D / 2
        reach = half - rng.uniform(0.020, 0.055)
        lat = rng.uniform(-0.25, 0.25)
        q[0] = fx * reach - fy * lat * TUNNEL_W / 2
        q[1] = fy * reach + fx * lat * TUNNEL_D / 2
    else:
        q[0] = rng.uniform(-0.35, 0.35) * TUNNEL_W / 2
        q[1] = rng.uniform(-0.40, 0.40) * TUNNEL_D / 2
    q[2] = DROP_H
    return q


def random_root(rng, q, upright_prob=0.5):
    """Place the pelvis just above the goal top.

    Two modes, because teams *place* these figures deliberately rather than
    dropping them:

      upright  the pelvis starts roughly z-up with a modest tilt, as when
               someone sits or kneels the figure on the goal.  Produces seated
               and kneeling arrangements.
      tumbled  a uniformly random orientation, which mostly topples into the
               prone sprawls seen in _04 and _08.

    Sampling only the tumbled mode (the obvious choice) starves the
    distribution of upright poses, because a random orientation almost never
    balances.  Both appear in the reference photographs, so both are sampled.
    """
    upright = rng.random() < upright_prob
    if upright:
        yaw = rng.uniform(0, 2 * math.pi)
        tilt = rng.uniform(0, math.radians(35))
        tdir = rng.uniform(0, 2 * math.pi)
        # yaw about z, then tilt about a random horizontal axis
        cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
        ct, st = math.cos(tilt / 2), math.sin(tilt / 2)
        ax, ay = math.cos(tdir) * st, math.sin(tdir) * st
        # quaternion product q_yaw * q_tilt, both (w,x,y,z)
        w = cz * ct
        x = cz * ax - sz * ay
        y = cz * ay + sz * ax
        z = sz * ct
        q[3:7] = [w, x, y, z]
    else:
        # Uniform random quaternion (Shoemake).
        u1, u2, u3 = rng.random(), rng.random(), rng.random()
        quat = np.array([
            math.sqrt(1 - u1) * math.sin(2 * math.pi * u2),
            math.sqrt(1 - u1) * math.cos(2 * math.pi * u2),
            math.sqrt(u1) * math.sin(2 * math.pi * u3),
            math.sqrt(u1) * math.cos(2 * math.pi * u3),
        ])
        q[3:7] = [quat[3], quat[0], quat[1], quat[2]]  # MuJoCo wants w first
    # Keep the pelvis over the tunnel top so it can land in contact with it.
    q[0] = rng.uniform(-0.30, 0.30) * TUNNEL_W / 2
    q[1] = rng.uniform(-0.30, 0.30) * TUNNEL_D / 2
    q[2] = DROP_H
    return q, upright


# --------------------------------------------------------------------------
# settling
# --------------------------------------------------------------------------

def pelvis_on_goal(m, d):
    """True if the pelvis rests on the goal's top face.

    Checks actual contact first, then proximity: a seated figure's weight can
    ride on the thigh roots with the pelvis skin hovering a hair above the
    surface.  A referee placing the real figure would call 3 mm "in contact";
    demanding a literal contact point rejected a third of good seated poses.
    """
    pel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    top = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "goal_top")
    for i in range(d.ncon):
        c = d.contact[i]
        b1 = m.geom_bodyid[c.geom1]
        b2 = m.geom_bodyid[c.geom2]
        if (c.geom1 == top and b2 == pel) or (c.geom2 == top and b1 == pel):
            return True
    # Proximity fallback over the pelvis collision primitives.
    hw, hd = TUNNEL_W / 2, TUNNEL_D / 2
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != pel or m.geom_group[g] != 3:
            continue
        x, y, z = d.geom_xpos[g]
        if abs(x) > hw or abs(y) > hd:
            continue
        r = m.geom_size[g][0]
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
            half = m.geom_size[g][1] * abs(d.geom_xmat[g].reshape(3, 3)[2, 2])
            z -= half
        if -0.002 < z - r < 0.003:
            return True
    return False


def settle(m, d, rng, hard, upright_prob=0.5, posed=False, signs=None):
    """Drop one sampled configuration and integrate until it stops moving.

    posed=True starts from a deliberately-set seated pose instead of a random
    drop; the settle then only has to make it physical (droop the cantilevers
    slightly, seat the pelvis into contact) rather than find it.
    """
    if posed:
        q, dangles = sample_posed_qpos(m, d, rng, hard, signs)
        d.qpos[:] = posed_root(rng, q, dangles)
        upright = True
        # Lift the spawn so nothing starts inside the goal.  A seated figure's
        # bent shins reach ~100 mm below the pelvis; spawned at pelvis height
        # they begin EMBEDDED in the top slab, and the solver ejects them out
        # of the nearest face -- downward, into the bore, leaving legs hanging
        # through solid wood (autopsied: feet at z=-60..-90 mm with the pelvis
        # centred).  Raising by the deepest penetration turns the spawn into
        # what a person does: lower the whole figure until it touches.
        mujoco.mj_forward(m, d)
        low = 0.0
        for g in range(m.ngeom):
            if m.geom_group[g] != 3 or m.body_weldid[m.geom_bodyid[g]] == 0:
                continue
            x, y, z = d.geom_xpos[g]
            r = m.geom_size[g][0]
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
                z -= m.geom_size[g][1] * abs(d.geom_xmat[g].reshape(3, 3)[2, 2])
            low = min(low, z - r)
        d.qpos[2] += DROP_H - low
    else:
        d.qpos[:], upright = random_root(rng, sample_qpos(m, rng, hard),
                                         upright_prob)
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)

    # The steadying hand.  A person setting a figure holds it while it takes
    # up contact; they do not drop it and let it ring.  Without this, the
    # spawn impact injects 1-3 rad/s into the light limbs and the pose spends
    # the whole window in a contact-excited wobble.  Zeroing the velocity a
    # few times early is exactly what the hand does.
    # Every 0.1 s through the first second: the placing hand never lets the
    # figure build up speed.  (The spawn may start up to ~100 mm high once
    # lifted clear of the goal, so the descent needs to be damped, not free.)
    steady = {int(t / m.opt.timestep) for t in np.arange(0.1, 1.01, 0.1)} \
        if posed else set()

    if posed:
        # Fixed-horizon criterion, inverted burden of proof.  Stillness tests
        # (qvel or mm-scale drift) can NEVER pass here: the solver's residual
        # enforcement error keeps held-up limbs creeping at 1-3 mm per half
        # second forever, motion no render would show.  The real toy is
        # statically placed, so integrate 2 s -- long enough to seat contacts
        # and droop the cantilevers to holdable angles -- and reject only
        # proof of REAL motion: a toppling root or centimetre-scale drift.
        # pelvis_on_goal / blocks_bore then rule on the snapshot.
        steps = int(2.0 / m.opt.timestep)
        span = int(0.5 / m.opt.timestep)
        prev = None
        for i in range(steps):
            if i in steady:
                d.qvel[:] = 0.0
            mujoco.mj_step(m, d)
            if not np.isfinite(d.qvel).all():
                return None, upright
            if i == steps - span:
                prev = _link_positions(m, d)
        if np.abs(d.qvel[:6]).max() > 0.5:                     # root toppling
            return None, upright
        drift = np.linalg.norm(_link_positions(m, d) - prev, axis=1).max()
        if drift > 0.015:                                      # real motion
            return None, upright
        return 2.0, upright

    steps = int(SETTLE_MAX_S / m.opt.timestep)
    quiet = 0
    for i in range(steps):
        mujoco.mj_step(m, d)
        vmax = float(np.abs(d.qvel).max())
        if not np.isfinite(vmax):
            # Rare solver blowup (~0.1 % of samples).  Both comparisons below
            # are False for NaN so it would be rejected anyway, but bail out
            # now instead of integrating a dead state for the full window.
            return None, upright
        if vmax < SETTLE_VEL:
            quiet += 1
            if quiet > 50:                      # 0.1 s of stillness
                return i * m.opt.timestep, upright
        else:
            quiet = 0
    # Slow creep rather than motion: the pose is visually static even though
    # it never fully stopped, so keep it rather than discarding the sample.
    if float(np.abs(d.qvel).max()) < SETTLE_VEL_TIMEOUT:
        return SETTLE_MAX_S, upright
    return None, upright


def extract(m, d):
    """Per-link transforms relative to the centre of the goal's top face."""
    out = {}
    for name in REAL_LINKS:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        pos = d.xpos[bid].copy()
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, d.xmat[bid])
        out[name] = [round(float(v), 6) for v in (*pos, *quat)]
    return out


def bbox(m, d):
    """World-axis bounding box of the visual meshes, metres."""
    lo = np.full(3, 1e9)
    hi = np.full(3, -1e9)
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        v0, nv = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        V = m.mesh_vert[v0:v0 + nv].reshape(-1, 3)
        W = V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        lo = np.minimum(lo, W.min(0))
        hi = np.maximum(hi, W.max(0))
    return lo, hi


# The ball passage: the open volume between the tunnel walls, under the roof.
BORE_X = TUNNEL_W / 2 - WALL
BORE_Z = (-TUNNEL_H + WALL, -WALL)


def blocks_bore(m, d):
    """True if any link intrudes into the tunnel's ball passage.

    The rulebook forbids poses that obstruct ball passage, so those samples
    are invalid training data -- a referee would have made the team re-pose.

    Note this is specifically the bore, not "anything below the top face": the
    figures in the reference photos routinely drape limbs over the tunnel's
    sides and front edge, which is legal and must stay in the distribution.
    """
    lo, hi = bbox(m, d)
    if lo[2] > BORE_Z[1] or abs(lo[0]) > BORE_X and abs(hi[0]) > BORE_X:
        return False                    # cheap reject: nothing reaches down
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        v0, nv = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        V = m.mesh_vert[v0:v0 + nv].reshape(-1, 3)
        W = V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        inside = ((np.abs(W[:, 0]) < BORE_X)
                  & (np.abs(W[:, 1]) < TUNNEL_D / 2)
                  & (W[:, 2] > BORE_Z[0]) & (W[:, 2] < BORE_Z[1]))
        if inside.any():
            return True
    return False


def worker(arg):
    n, seed, hard_frac, upright_prob, posed_frac = arg
    # Two scenes: drop-and-settle at calibrated friction, posed at 3x (see
    # build_scene).  Same geometry, so poses transfer between them freely.
    m_drop = build_scene()
    m_pose = build_scene(friction_scale=3.0)
    d_drop, d_pose = mujoco.MjData(m_drop), mujoco.MjData(m_pose)
    signs = probe_signs(m_drop, d_drop)
    rng = np.random.default_rng(seed)
    rows, tried = [], 0
    kept_posed = 0
    quota_posed = round(n * posed_frac)
    while len(rows) < n and tried < n * 12:
        tried += 1
        hard = rng.random() < hard_frac
        # Fill KEPT quotas, not attempt quotas.  Posed accepts ~4x more often
        # than drop, so drawing the mode per attempt at posed_frac skewed the
        # kept set to 88 % posed.  Pick whichever mode is further below quota.
        kept_drop = len(rows) - kept_posed
        posed = (kept_posed * (n - quota_posed)
                 <= kept_drop * quota_posed) if quota_posed < n else True
        m, d = (m_pose, d_pose) if posed else (m_drop, d_drop)
        t, upright = settle(m, d, rng, hard, upright_prob,
                            posed=posed, signs=signs)
        if t is None:
            continue
        if not pelvis_on_goal(m, d):
            continue
        if blocks_bore(m, d):
            continue
        lo, hi = bbox(m, d)
        kept_posed += bool(posed)
        rows.append({
            "hard": bool(hard),
            "posed": bool(posed),
            "upright": bool(upright),
            "qpos": [round(float(v), 6) for v in d.qpos],
            "links": extract(m, d),
            "bbox_local": [round(float(v), 5) for v in (*lo, *hi)],
            "settled_after": round(t, 3),
        })
    return rows, tried


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--hard-fraction", type=float, default=HARD_FRACTION)
    ap.add_argument("--upright-prob", type=float, default=0.5,
                    help="fraction started in a deliberate upright placement")
    ap.add_argument("--posed-frac", type=float, default=0.65,
                    help="fraction sampled as deliberately-set seated poses "
                         "(what the venue photos show teams actually do); the "
                         "rest are drop-and-settle sprawls")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=os.path.join(HERE, "poses.jsonl"))
    args = ap.parse_args()

    per = math.ceil(args.n / args.workers)
    jobs = [(per, args.seed * 10000 + i, args.hard_fraction, args.upright_prob,
             args.posed_frac)
            for i in range(args.workers)]

    t0 = time.time()
    with Pool(args.workers) as pool:
        results = pool.map(worker, jobs)

    rows = [r for chunk, _ in results for r in chunk][:args.n]
    tried = sum(t for _, t in results)

    with open(args.out, "w") as f:
        for i, r in enumerate(rows):
            r["id"] = i
            f.write(json.dumps(r) + "\n")

    dt = time.time() - t0
    hard = sum(1 for r in rows if r["hard"])
    upr = sum(1 for r in rows if r["upright"])
    posed = sum(1 for r in rows if r.get("posed"))
    print(f"wrote {args.out}")
    print(f"  poses kept    : {len(rows)} / {tried} attempts "
          f"({100*len(rows)/max(tried,1):.0f}% accepted)")
    print(f"  hard-case     : {hard} ({100*hard/max(len(rows),1):.0f}%)")
    print(f"  posed (seated): {posed} ({100*posed/max(len(rows),1):.0f}%)")
    print(f"  upright start : {upr} ({100*upr/max(len(rows),1):.0f}%)")
    print(f"  wall clock    : {dt:.1f} s on {args.workers} workers")
    if rows:
        ex = np.array([r["bbox_local"] for r in rows])
        d = ex[:, 3:] - ex[:, :3]
        print(f"  extent median : {np.median(d[:,0])*1000:.0f} x "
              f"{np.median(d[:,1])*1000:.0f} x {np.median(d[:,2])*1000:.0f} mm")


if __name__ == "__main__":
    main()
