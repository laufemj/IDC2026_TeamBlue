"""Render the material-calibration wall and report where each patch landed.

Not run by hand -- calibrate.py drives it:

    blender -b -P calib_scene.py -- --out /tmp/calib.png --patches /tmp/patches.json

A wall of material samples faces the camera under the SAME lighting rig as
the training scene.  Flat cards stand in for the diffuse materials; the
actual dummy meshes stand in for silver / cloth / wood, because a metal's
patch median depends on curvature (highlight + body averaged) and a flat
card would calibrate the wrong thing.

The view transform is Standard, not Filmic/AgX: calibrate.py's update rule
is multiplicative in linear light, and a shoulder curve would break the
convergence arithmetic.  The training renders keep the same transform so the
calibrated albedos stay valid.
"""

import argparse
import json
import math
import os
import sys

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene_params as P
import build_scene as B


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--patches", required=True)
    args = ap.parse_args(argv)

    sc = B.reset_scene()
    sc.view_settings.view_transform = "Standard"
    sc.render.resolution_x = 1280
    sc.render.resolution_y = 960
    B.build_lights()

    # A lime floor for bounce: in the venue, vertical surfaces (goal fronts,
    # dummy torsos) are lit substantially by light reflected off the bright
    # mats, and the first version of this scene had no bounce source at all
    # -- its camera-facing cards saw only the dim world background, so the
    # anchor stayed dark no matter how hard the tubes were driven.
    floor_mat = B.new_mat("bounce_floor", P.ALBEDO["field_bright"], rough=0.6)
    B.box("bounce_floor", (6.0, 6.0, 0.02), (0.0, 0.0, -0.01), floor_mat)
    # White walls/ceiling: the silver samples mirror their surroundings, so
    # the calibration environment has to look like the venue room.
    B.build_room(floor_z=-0.02)

    # --- sample cards: FLAT and low-specular ------------------------------
    # Tipping them toward the camera aimed the ceiling tubes' specular lobe
    # straight into the lens: every card, black tape included, rendered as
    # the same white sheen.  The venue photo patches are matte surfaces seen
    # off the mirror direction, so flat + spec 0.15 is the honest analogue.
    cards = ["field_bright", "field_dark", "tape", "goal_paint", "goal_ply"]
    samples = {}
    for i, name in enumerate(cards):
        mat = B.new_mat(f"card_{name}", P.ALBEDO[name], rough=0.6, spec=0.15)
        o = B.box(f"card_{name}", (0.16, 0.16, 0.006),
                  (-0.44 + 0.22 * i, 0.10, 0.30), mat)
        samples[name] = o

    # real meshes for the view-dependent materials, standing on the floor
    robot = B.dummy_materials("robot", 90)
    human = B.dummy_materials("human", 91)
    for name, part, mats, sx in (
            ("silver", "chest", robot["default"], -0.33),
            ("cloth_grey", "chest", human["default"], 0.0),
            ("wood_bare", "head", human["bare"], 0.33)):
        mesh = B.part_mesh(part).copy()
        o = bpy.data.objects.new(f"m_{name}", mesh)
        o.data.materials.clear()
        o.data.materials.append(mats)
        o.location = (sx, -0.25, 0.02)
        o.scale = (2.2, 2.2, 2.2)     # bigger patch = stabler median
        B.link_obj(o)
        samples[name] = o

    cam_data = bpy.data.cameras.new("calib_cam")
    cam_data.sensor_fit = "HORIZONTAL"
    cam_data.sensor_width = P.SENSOR_W_MM
    cam_data.lens = (P.SENSOR_W_MM / 2) / math.tan(
        math.radians(P.HFOV_DEG) / 2)
    cam = bpy.data.objects.new("calib_cam", cam_data)
    cam.location = (0.0, -0.85, 1.15)          # steep view onto flat cards
    from mathutils import Vector as _V
    d = _V((0.0, 0.02, 0.15)) - _V(cam.location)
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    B.link_obj(cam)
    sc.camera = cam

    bpy.context.view_layer.update()
    patches = {}
    for name, o in samples.items():
        # project the object's median vertex neighbourhood centre
        centre = o.matrix_world @ Vector((sum(v.co[i] for v in
                                          o.data.vertices) /
                                          len(o.data.vertices)
                                          for i in range(3)))
        ndc = world_to_camera_view(sc, cam, centre)
        patches[name] = [int(ndc.x * sc.render.resolution_x),
                         int((1.0 - ndc.y) * sc.render.resolution_y)]
    json.dump(patches, open(args.patches, "w"))

    sc.render.filepath = args.out
    bpy.ops.render.render(write_still=True)
    print("calib render done")


main()
