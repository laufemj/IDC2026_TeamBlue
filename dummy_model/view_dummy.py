#!/usr/bin/env python3
"""Open the mannequin in MuJoCo's interactive viewer.

    python3 view_dummy.py              # standing on the floor
    python3 view_dummy.py --collision  # show collision primitives too
    python3 view_dummy.py --drop       # drop it and let it settle

Controls once the window is open:
    Ctrl + drag on a body   pull it around (this is how you check the joints)
    double-click            select a body
    right-drag              pan,  scroll = zoom,  left-drag = orbit
    Tab                     side panel: joint sliders, physics options
    space                   pause / resume the simulation
    backspace               reset

The side panel's Joint tab gives a slider per DOF, which is the quickest way
to confirm the limits and the ball-joint decomposition behave sensibly.
"""

import argparse
import os

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))


def sole_offset(m, d):
    """How far the lowest visual vertex sits below the pelvis frame."""
    mujoco.mj_forward(m, d)
    lo = 1e9
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        v0, nv = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        V = m.mesh_vert[v0:v0 + nv].reshape(-1, 3)
        W = V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        lo = min(lo, W[:, 2].min())
    return -lo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=os.path.join(HERE, "dummy_scene.xml"))
    ap.add_argument("--collision", action="store_true",
                    help="also draw the collision primitives (group 3)")
    ap.add_argument("--drop", type=float, default=None, metavar="H",
                    help="start H metres above the floor and let it fall")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.scene)
    d = mujoco.MjData(m)

    d.qpos[2] = sole_offset(m, d) + (args.drop or 0.0)
    mujoco.mj_forward(m, d)

    with mujoco.viewer.launch_passive(m, d) as v:
        v.opt.geomgroup[3] = 1 if args.collision else 0
        v.cam.lookat[:] = [0, 0, 0.15]
        v.cam.distance = 0.7
        v.cam.elevation = -10
        v.cam.azimuth = 135
        print("viewer open -- Tab for joint sliders, Ctrl+drag to pull limbs")
        while v.is_running():
            if args.drop is not None:
                mujoco.mj_step(m, d)
            v.sync()


if __name__ == "__main__":
    main()
