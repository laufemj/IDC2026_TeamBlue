#!/usr/bin/env python3
"""Solve for the joint stiction that makes the figure hold a pose.

The wooden mannequin is not a ragdoll.  Photo _12 shows an arm cantilevered
horizontally with nothing supporting it; photo _04 shows a figure standing
unassisted.  That is stiction in the wooden ball joints, and it is the single
most important physical property to get right -- a frictionless model settles
into heaps of limbs that look nothing like the reference photographs, and the
synthetic pose distribution would then be systematically wrong.

Two independent estimates, which should agree:

  1. Analytic.  A limb held horizontal needs joint torque
         tau = m_limb * g * r_com
     where r_com is the horizontal distance from the joint to the limb
     subtree's centre of mass.  This is the minimum frictionloss.

  2. Simulated.  Weld the pelvis, pose each limb horizontally, run for 2 s,
     and measure how far the joint drifts.  Bisect the frictionloss scale
     until droop is under the tolerance.

The analytic value is the floor; the reported value adds a safety factor
because the real toy visibly holds these poses rather than barely holding
them.  Feed the result back into make_mjcf.DEFAULT_FRICTIONLOSS.

Usage:  python3 calibrate_friction.py [--droop-tol-deg 2.0]
"""

import argparse
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81

# Which joint to test, and the subtree it has to hold up when horizontal.
# Each entry: (joint to load, body at the root of the supported subtree).
TESTS = [
    ("shoulder_l_y", "upper_arm_l"),
    ("elbow_l", "forearm_l"),
    ("wrist_l", "hand_l"),
    ("hip_l_y", "thigh_l"),
    ("knee_l", "shin_l"),
    ("ankle_l", "foot_l"),
    ("waist_y", "chest"),
    ("neck_x", "head"),
]


def subtree_stats(m, d, body_name):
    """Mass and centre of mass of a body and everything below it."""
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    mass = m.body_subtreemass[bid]
    com = d.subtree_com[bid].copy()
    return bid, mass, com


def worst_case_angle(m, joint_name):
    """The in-range angle that loads the joint hardest, radians.

    Gravity torque peaks with the limb horizontal, i.e. 90 deg from the
    hanging zero pose.  Joints that cannot reach 90 deg (waist +-35, neck
    +-40, ankle +-45, knee -150..0) are tested at their own limit instead.
    A 5 % margin keeps the test strictly inside the range: sitting exactly on
    a limit hands the load to the constraint solver, which makes the measured
    torque meaningless (and numerically explosive).
    """
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    lo, hi = m.jnt_range[jid]
    margin = 0.05 * (hi - lo)
    lo, hi = lo + margin, hi - margin
    target = math.pi / 2
    return max(lo, min(hi, target)) if hi >= 0 else max(lo, min(hi, -target))


def descendants(m, root_bid):
    """Body ids of root_bid and everything below it."""
    out = [root_bid]
    changed = True
    while changed:
        changed = False
        for b in range(m.nbody):
            if b not in out and m.body_parentid[b] in out:
                out.append(b)
                changed = True
    return out


def holding_torque(m, joint_name, body_name):
    """Static joint torque needed to hold the limb at its worst-case angle.

    Computed as a direct sum of gravity moments over the supported subtree:

        tau = | axis . sum_i (com_i - anchor) x (m_i * g) |

    Inverse dynamics would be the obvious tool, but mj_inverse returns garbage
    (torques of 1e14) whenever a limit constraint is active, and several of
    these joints are tested near their limits by construction.  The direct sum
    is exact for a static gravity load and cannot blow up.
    """
    d = mujoco.MjData(m)
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qadr = m.jnt_qposadr[jid]
    ang = worst_case_angle(m, joint_name)
    d.qpos[qadr] = ang
    mujoco.mj_forward(m, d)

    anchor, axis = d.xanchor[jid].copy(), d.xaxis[jid].copy()
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)

    tau_vec = np.zeros(3)
    mass = 0.0
    weighted = np.zeros(3)
    for b in descendants(m, bid):
        mb = m.body_mass[b]
        if mb <= 0:
            continue
        com = d.xipos[b]                      # world COM of this body alone
        tau_vec += np.cross(com - anchor, np.array([0.0, 0.0, -mb * G]))
        mass += mb
        weighted += mb * com
    tau = abs(float(np.dot(tau_vec, axis)))
    lever = float(np.linalg.norm(weighted / mass - anchor)) if mass else 0.0
    return tau, mass, lever, math.degrees(ang)


def build(scale, damping=None, absolute=None):
    """MJCF with the pelvis welded, for measuring joint droop.

    scale     multiplies every DEFAULT_FRICTIONLOSS value
    absolute  {family: value} overriding scale entirely
    damping   if given, forces this damping on every joint
    """
    import make_mjcf
    import xml.etree.ElementTree as ET
    urdf = os.path.join(HERE, "dummy.urdf")
    tree = make_mjcf.urdf_to_mjcf(urdf)
    friction = absolute or {k: v * scale
                            for k, v in make_mjcf.DEFAULT_FRICTIONLOSS.items()}
    make_mjcf.patch(tree, friction, 640, 480,
                    make_mjcf.read_urdf_inertial(urdf, "pelvis"), floor=False)
    if damping is not None:
        for j in tree.getroot().iter("joint"):
            if j.get("name") and j.get("name") != "root":
                j.set("damping", f"{damping:g}")
    # Weld the pelvis so we measure joint droop, not the figure falling over.
    root = tree.getroot()
    pel = root.find("worldbody/body[@name='pelvis']")
    fj = pel.find("freejoint")
    if fj is not None:
        pel.remove(fj)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def droop(m, joint_name, seconds=2.0):
    """Degrees a worst-case-posed joint sags over `seconds` of simulation."""
    d = mujoco.MjData(m)
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    adr = m.jnt_qposadr[jid]
    start = worst_case_angle(m, joint_name)
    d.qpos[adr] = start
    mujoco.mj_forward(m, d)
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
    return abs(math.degrees(d.qpos[adr] - start))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--droop-tol-deg", type=float, default=2.0)
    ap.add_argument("--safety", type=float, default=1.6,
                    help="multiplier on the analytic minimum")
    args = ap.parse_args()

    import make_mjcf
    # Measure holding torque on a frictionless, undamped copy so the number is
    # pure gravity, not gravity plus whatever stiction is currently set.
    m0 = build(0.0, damping=0.0)

    print(f"{'joint':<14} {'subtree':<12} {'mass g':>7} {'test deg':>9} "
          f"{'lever mm':>9} {'tau_min mNm':>12} {'current mNm':>12} {'verdict':>8}")
    print("-" * 88)

    recommended = {}
    for joint, body in TESTS:
        tau, mass, lever, ang = holding_torque(m0, joint, body)
        fam = make_mjcf.joint_family(joint)
        cur = make_mjcf.DEFAULT_FRICTIONLOSS[fam]
        ok = "HOLDS" if cur >= tau else "SAGS"
        print(f"{joint:<14} {body:<12} {mass*1000:7.2f} {ang:9.1f} "
              f"{lever*1000:9.1f} {tau*1000:12.3f} {cur*1000:12.2f} {ok:>8}")
        want = tau * args.safety
        recommended[fam] = max(recommended.get(fam, 0.0), want)

    print("\nRecommended DEFAULT_FRICTIONLOSS "
          f"(analytic minimum x {args.safety} safety factor):")
    for k in sorted(recommended):
        print(f'    "{k}": {recommended[k]:.4f},')

    # Simulated confirmation using the recommended per-family values.
    print("\nSimulated droop over 2 s at the recommended values:")
    full = dict(make_mjcf.DEFAULT_FRICTIONLOSS)
    full.update(recommended)
    m = build(1.0, absolute=full)
    worst = 0.0
    for joint, _ in TESTS:
        dg = droop(m, joint)
        worst = max(worst, dg)
        flag = "ok" if dg <= args.droop_tol_deg else "SAGS"
        print(f"    {joint:<14} {dg:6.2f} deg   {flag}")
    print(f"\n  worst droop {worst:.2f} deg (tolerance {args.droop_tol_deg} deg)")
    print("\n  Validate against photo _12 (arm cantilevered horizontally) and\n"
          "  _04 (figure standing unassisted) before trusting these numbers.")


if __name__ == "__main__":
    main()
