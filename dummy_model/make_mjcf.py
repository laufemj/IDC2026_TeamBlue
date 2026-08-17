#!/usr/bin/env python3
"""Convert dummy.urdf to a physics-ready MJCF for pose settling.

The URDF is the canonical, portable artifact.  MuJoCo can load it directly,
but a raw URDF load is not simulation-ready: the root is welded to the world,
limb collisions are cylinders with sharp rims, and the joints are frictionless
so the figure collapses into a heap.

This step patches those, producing dummy_scene.xml:

  * free joint on the pelvis, so the figure can be dropped and settled
  * cylinder collision geoms -> capsules.  Rounded ends give far more stable
    contact for a tumbling figure than a cylinder's sharp rim.
  * frictionloss + armature on every joint.  THIS IS THE IMPORTANT ONE.  The
    real toy is not a ragdoll -- it holds a pose (see photo _12, where an arm
    is cantilevered horizontally with nothing supporting it).  Without joint
    stiction the settled poses look nothing like the reference photographs.
  * offscreen framebuffer large enough to render preview images
  * a ground plane and lighting for visualisation

Usage:  python3 make_mjcf.py [--frictionloss X] [--out dummy_scene.xml]
"""

import argparse
import os
import xml.etree.ElementTree as ET

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))

# Per-joint stiction, N*m.  Calibrated so a horizontally cantilevered arm
# holds its own weight (photo _12) without the figure becoming rigid.
# calibrate_friction.py solves for this; the default is its result.
# Solved by calibrate_friction.py: the gravity torque each joint must hold at
# its worst-case in-range angle, times a 1.6 safety factor.  The initial
# hand-guessed values were 2-5x too low and the figure collapsed.
DEFAULT_FRICTIONLOSS = {
    "shoulder": 0.0056,
    "elbow": 0.0016,
    "wrist": 0.0003,
    "hip": 0.0134,
    "knee": 0.0041,
    "ankle": 0.0003,
    "waist": 0.0235,
    "neck": 0.0068,
}
# Reflected rotor inertia.  Small but non-zero: damps the numerical spikes a
# 25-DOF chain of very light links produces on impact.
ARMATURE = 2e-7


def joint_family(name):
    for key in DEFAULT_FRICTIONLOSS:
        if name.startswith(key):
            return key
    raise KeyError(name)


def urdf_to_mjcf(urdf_path):
    """Load the URDF and dump MuJoCo's MJCF translation of it."""
    model = mujoco.MjModel.from_xml_path(urdf_path)
    tmp = os.path.join(HERE, ".dummy_raw.xml")
    mujoco.mj_saveLastXML(tmp, model)
    tree = ET.parse(tmp)
    os.remove(tmp)
    return tree


def read_urdf_inertial(urdf_path, link_name):
    """Pull one link's <inertial> out of the URDF as MJCF attributes.

    Needed because MuJoCo drops the root link's mass when it merges that link
    into the worldbody, and we want the pelvis to weigh what gen_dummy.py said
    it weighs rather than a number retyped here.
    """
    link = ET.parse(urdf_path).getroot().find(f"link[@name='{link_name}']")
    if link is None:
        raise RuntimeError(f"link '{link_name}' not in {urdf_path}")
    inert = link.find("inertial")
    mass = inert.find("mass").get("value")
    I = inert.find("inertia")
    origin = inert.find("origin")
    pos = origin.get("xyz") if origin is not None else "0 0 0"
    return {
        "pos": pos,
        "mass": mass,
        "diaginertia": f"{I.get('ixx')} {I.get('iyy')} {I.get('izz')}",
    }


def patch(tree, friction, offw, offh, pelvis_inertial, floor=True):
    root = tree.getroot()

    def find_or_make(tag):
        el = root.find(tag)
        if el is None:
            el = ET.SubElement(root, tag)
        return el

    # --- offscreen framebuffer + nicer default lighting -------------------
    vis = find_or_make("visual")
    g = vis.find("global")
    if g is None:
        g = ET.SubElement(vis, "global")
    g.set("offwidth", str(offw))
    g.set("offheight", str(offh))

    # --- integrator / solver ---------------------------------------------
    opt = find_or_make("option")
    opt.set("timestep", "0.002")
    opt.set("integrator", "implicitfast")
    # Elliptic cone + high impratio: the figure is light (~117 g) with small
    # contact patches, and the default pyramidal cone lets it skate.
    opt.set("cone", "elliptic")
    opt.set("impratio", "10")
    # The default solver budget cannot satisfy joint-friction constraints on a
    # 25-DOF chain of ~1 g bodies: droop was measured identical at
    # frictionloss 0.006, 0.05 and 0.5, i.e. completely insensitive to the
    # friction value.  More Newton iterations plus a noslip pass fixes it, and
    # the same limbs then hold to <0.1 deg over 2 s.
    opt.set("solver", "Newton")
    opt.set("iterations", "200")
    opt.set("tolerance", "1e-12")
    opt.set("noslip_iterations", "20")

    worldbody = root.find("worldbody")

    # --- free joint on the root body --------------------------------------
    # A URDF root link carries no joint, so MuJoCo merges it into the
    # worldbody: the pelvis geoms and its three kinematic chains end up loose
    # at top level, and the pelvis mass is silently discarded (86 g instead of
    # 117 g).  Re-wrap them all in a floating body named "pelvis".
    if worldbody.find("body[@name='pelvis']") is None:
        pelvis = ET.Element("body", {"name": "pelvis", "pos": "0 0 0"})
        pelvis.append(ET.Element("freejoint", {"name": "root"}))
        for child in list(worldbody):
            if child.tag in ("geom", "body", "site"):
                worldbody.remove(child)
                pelvis.append(child)
        # Restore the mass the merge threw away, reading it back out of the
        # URDF's pelvis <inertial> rather than hardcoding it here.
        ET.SubElement(pelvis, "inertial", pelvis_inertial)
        worldbody.append(pelvis)

    # --- separate visual from collision, and round off the collision --------
    # Group 2 = visual meshes (rendered, no contact).  Group 3 = collision
    # primitives (contact, not rendered).  MuJoCo's URDF importer does not
    # make this split itself, so preview renders otherwise show the collision
    # boxes sitting on top of the lathe meshes.
    #
    # Split on what the URDF importer already recorded, NOT on geom type.  It
    # used to test `type == "mesh"`, which was only ever a proxy for "came
    # from a <visual>" -- and it stopped being true the moment the joint balls
    # were added as sphere visuals.  Those spheres fell through to the
    # collision branch, so the balls got filed as group 3 and vanished from
    # every render: the exact symptom they were added to fix.
    #
    # MuJoCo's URDF importer stamps contype=0 conaffinity=0 density=0 on
    # <visual> geoms and leaves <collision> geoms at the defaults, so that
    # flag is the real signal.
    n_caps = n_vis = n_col = 0
    for geom in root.iter("geom"):
        if geom.get("contype") == "0" and geom.get("conaffinity") == "0":
            geom.set("group", "2")
            n_vis += 1
        else:
            if geom.get("type") == "cylinder":
                # Rounded ends give far more stable contact for a tumbling
                # figure than a cylinder's sharp rim.
                geom.set("type", "capsule")
                n_caps += 1
            geom.set("group", "3")
            geom.set("rgba", "0.9 0.4 0.2 0.35")
            n_col += 1

    # --- joint stiction ----------------------------------------------------
    n_j = 0
    for j in root.iter("joint"):
        name = j.get("name")
        if not name or name == "root":
            continue
        j.set("frictionloss", f"{friction[joint_family(name)]:.6f}")
        j.set("armature", f"{ARMATURE:.3e}")
        j.set("damping", "0.0005")
        n_j += 1

    # --- ground + light for preview renders --------------------------------
    if floor and worldbody.find("geom[@name='floor']") is None:
        ET.SubElement(worldbody, "geom", {
            "name": "floor", "type": "plane", "size": "2 2 0.05",
            "pos": "0 0 0", "rgba": "0.85 0.85 0.87 1",
            "condim": "6", "friction": "1.0 0.02 0.001",
        })
        ET.SubElement(worldbody, "light", {
            "name": "key", "pos": "0.4 -0.4 1.0", "dir": "-0.4 0.4 -1.0",
            "directional": "true", "diffuse": "0.7 0.7 0.7",
        })
        ET.SubElement(worldbody, "light", {
            "name": "fill", "pos": "-0.5 0.3 0.8", "dir": "0.5 -0.3 -0.8",
            "directional": "true", "diffuse": "0.3 0.3 0.32",
        })

    # --- contact exclusions across the ball chains -------------------------
    # MuJoCo automatically skips contacts between a parent body and its child.
    # Our ball joints are three stacked revolutes with two zero-length links
    # in between, so chest and upper_arm are great-grandparent/child, not
    # parent/child, and MuJoCo happily collides them -- against the ball they
    # are pivoting on.  These pairs are physically welded through a joint and
    # must be excluded by hand.
    adjacent = [("pelvis", "chest"), ("chest", "head"),
                ("chest", "upper_arm_l"), ("chest", "upper_arm_r"),
                ("pelvis", "thigh_l"), ("pelvis", "thigh_r")]
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    have = {(e.get("body1"), e.get("body2")) for e in contact.iter("exclude")}
    n_ex = 0
    for a, b in adjacent:
        if (a, b) in have or (b, a) in have:
            continue
        ET.SubElement(contact, "exclude", {"body1": a, "body2": b})
        n_ex += 1

    return n_caps, n_j, n_ex


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=os.path.join(HERE, "dummy.urdf"))
    ap.add_argument("--out", default=os.path.join(HERE, "dummy_scene.xml"))
    ap.add_argument("--offw", type=int, default=1280)
    ap.add_argument("--offh", type=int, default=960)
    ap.add_argument("--scale-friction", type=float, default=1.0,
                    help="multiply every frictionloss value (for calibration)")
    ap.add_argument("--no-floor", action="store_true")
    args = ap.parse_args()

    friction = {k: v * args.scale_friction
                for k, v in DEFAULT_FRICTIONLOSS.items()}

    tree = urdf_to_mjcf(args.urdf)
    n_caps, n_j, n_ex = patch(tree, friction, args.offw, args.offh,
                              read_urdf_inertial(args.urdf, "pelvis"),
                              floor=not args.no_floor)
    tree.write(args.out, encoding="unicode")

    m = mujoco.MjModel.from_xml_path(args.out)
    print(f"wrote {args.out}")
    print(f"  cylinders -> capsules : {n_caps}")
    print(f"  joints given stiction : {n_j}")
    print(f"  adjacent-pair excludes: {n_ex}")
    print(f"  nq={m.nq} nv={m.nv} nbody={m.nbody} ngeom={m.ngeom}")
    print(f"  total mass = {m.body_subtreemass[1]*1000:.1f} g")


if __name__ == "__main__":
    main()
