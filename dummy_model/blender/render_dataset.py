"""Render the YOLO training dataset: images + bbox labels, randomized.

    ~/apps/blender/blender -b -P blender/render_dataset.py -- \
        --n 4000 --out dataset --shard 0 --seed 1

Per frame:
  * four goals, each pair holding one human + one robot figure, assignment
    shuffled per frame (the rulebook randomizes it per match),
  * POSE PARITY: every frame is rendered twice with the class assignment
    swapped and everything else identical, so pose carries zero class
    information across the dataset,
  * camera sampled over the robot's plausible approach region (start corner
    down to mid-field), aimed at the goal row with jitter + roll,
  * illumination jitter (strength / colour) and exposure jitter around the
    photo-calibrated baseline,
  * background clutter: random boxes/cylinders in muted colours behind the
    goal row (tables, cartons) and small litter on the carpet -- the venue
    is a working lab, not a studio, and empty-background FP suppression
    needs the model to see junk that is not a dummy,
  * labels: class 0 = human_dummy, 1 = robot_dummy, YOLO xywhn format from
    the projected 2D bounds of every visual in the figure.

Output:  <out>/images/<split>/shardS_NNNNN.jpg
         <out>/labels/<split>/shardS_NNNNN.txt
"""

import argparse
import json
import math
import os
import random
import sys

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Euler, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene_params as P
import build_scene as B

MODEL = P.MODEL_DIR
CLS = {"human": 0, "robot": 1}


def sample_camera(rng, sc, cam, tops):
    """Robot's-eye view: somewhere on the approach, looking at the goals."""
    y = rng.uniform(1.0, 3.45)
    half = P.FIELD_W / 2
    x = rng.uniform(-0.85, 0.85) * half
    z = B.surface_z(y) + P.CAM_HEIGHT + rng.uniform(-0.05, 0.10)
    cam.location = (x, y, z)
    # aim at a point on the goal row with generous jitter
    gx = rng.uniform(P.GOAL_X[0] - 0.15, P.GOAL_X[-1] + 0.15)
    gz = tops[0] + rng.uniform(-0.05, 0.15)
    direction = Vector((gx, P.GOAL_EDGE_Y, gz)) - cam.location
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    cam.rotation_euler.rotate_axis("Z", rng.uniform(-0.12, 0.12))  # roll


def jitter_lights(rng, sc):
    em = bpy.data.materials["lum_ceiling"].node_tree.nodes["Emission"]
    em.inputs["Strength"].default_value = (
        P.LIGHT["energy_scale"] * rng.uniform(0.7, 1.4))
    c = [min(1.0, v * rng.uniform(0.92, 1.08)) for v in P.LIGHT["color"]]
    em.inputs["Color"].default_value = (*c, 1.0)
    sc.view_settings.exposure = rng.uniform(-0.35, 0.35)


_CLUTTER = []


def build_clutter(rng):
    """Random junk behind the goal row and on the carpet."""
    global _CLUTTER
    for o in _CLUTTER:
        bpy.data.objects.remove(o, do_unlink=True)
    _CLUTTER = []
    palettes = [(0.35, 0.25, 0.15), (0.55, 0.45, 0.32),   # cardboard
                (0.25, 0.25, 0.28), (0.55, 0.55, 0.58),   # steel/plastic
                (0.30, 0.05, 0.05), (0.05, 0.15, 0.35),   # chairs
                (0.65, 0.6, 0.2), (0.2, 0.4, 0.25)]       # bags
    n = rng.randint(6, 14)
    for i in range(n):
        col = palettes[rng.randrange(len(palettes))]
        col = tuple(min(1.0, c * rng.uniform(0.6, 1.5)) for c in col)
        mat = B.new_mat(f"cl_{i}", (*col, 1.0), rough=rng.uniform(0.5, 0.95))
        big = rng.random() < 0.4
        size = ((rng.uniform(0.5, 1.6), rng.uniform(0.3, 0.8),
                 rng.uniform(0.4, 0.9)) if big else
                (rng.uniform(0.1, 0.45),) * 3)
        x = rng.uniform(-2.2, 2.2)
        y = rng.uniform(-1.7, -0.35)
        o = B.box(f"cl_{i}", size, (x, y, -0.212 + size[2] / 2), mat)
        o.rotation_euler = (0, 0, rng.uniform(0, math.pi))
        _CLUTTER.append(o)


_DUMMIES = []


def clear_dummies():
    global _DUMMIES
    for o in _DUMMIES:
        bpy.data.objects.remove(o, do_unlink=True)
    _DUMMIES = []


def visible(sc, cam, objs, min_hits=2):
    """True if the figure is actually visible from the camera.

    Rays from the lens to each part's origin; count how many strike one of
    the figure's own objects first.  Without this, a dummy draped down the
    goal's far side -- hidden behind the field mass from a low camera --
    still got a full label, and ~5 % of ground truth was empty boxes over
    walls and bare goals.  Those ghosts teach exactly the failure the real
    photos showed: hallucinating figures onto clutter.
    """
    deps = bpy.context.evaluated_depsgraph_get()
    origin = Vector(cam.location)
    own = {o.name_full for o in objs}
    hits = 0
    for o in objs:
        target = o.matrix_world.translation
        d = target - origin
        if d.length < 1e-6:
            continue
        ok, _, _, _, obj, _ = sc.ray_cast(deps, origin, d.normalized())
        if ok and obj is not None and obj.name_full in own:
            hits += 1
            if hits >= min_hits:
                return True
    return False


def bbox_2d(sc, cam, objs):
    """Projected pixel bbox over the objects' bound boxes.  None if behind."""
    xs, ys = [], []
    for o in objs:
        for corner in o.bound_box:
            w = o.matrix_world @ Vector(corner)
            ndc = world_to_camera_view(sc, cam, w)
            if ndc.z <= 0:
                continue
            xs.append(ndc.x)
            ys.append(1.0 - ndc.y)
    if not xs:
        return None
    x0 = max(0.0, min(xs)); x1 = min(1.0, max(xs))
    y0 = max(0.0, min(ys)); y1 = min(1.0, max(ys))
    if x1 - x0 < 1e-4 or y1 - y0 < 1e-4:
        return None
    return x0, y0, x1, y1


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=200,
                    help="frame PAIRS per shard (output = 2n images)")
    ap.add_argument("--out", default=os.path.join(MODEL, "dataset"))
    ap.add_argument("--shard", type=int, default=0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--val-every", type=int, default=10,
                    help="every k-th pair goes to the val split")
    ap.add_argument("--cams", type=int, default=3,
                    help="camera viewpoints rendered per arrangement")
    ap.add_argument("--poses", default=os.path.join(MODEL, "poses.jsonl"))
    args = ap.parse_args(argv)

    rng = random.Random(args.seed * 7919 + args.shard)
    poses = [json.loads(l) for l in open(args.poses)]

    sc = B.reset_scene()
    sc.render.resolution_x, sc.render.resolution_y = 640, 480
    sc.cycles.samples = 48
    sc.render.image_settings.file_format = "JPEG"
    sc.render.image_settings.quality = 90
    B.build_field()
    B.build_room()
    tops = [B.build_goal(i, x) for i, x in enumerate(P.GOAL_X)]
    B.build_lights()
    cam = B.build_camera(look_at=(0.0, P.GOAL_EDGE_Y, tops[0]))

    for split in ("train", "val"):
        os.makedirs(os.path.join(args.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(args.out, "labels", split), exist_ok=True)

    for i in range(args.n):
        frame_poses = [poses[rng.randrange(len(poses))] for _ in range(4)]
        # one human + one robot per pair, shuffled independently
        pair1 = ["human", "robot"]; rng.shuffle(pair1)
        pair2 = ["human", "robot"]; rng.shuffle(pair2)
        kinds = pair1 + pair2
        # Empty-goal negatives: goals stand bare between matches, and a
        # detector that has never seen a bare white tunnel will hallucinate
        # figures onto it.  ~12 % of goals empty, occasionally a whole frame.
        occupied = [rng.random() > 0.12 for _ in range(4)]
        if rng.random() < 0.04:
            occupied = [False] * 4

        build_clutter(rng)
        split = "val" if i % args.val_every == 0 else "train"

        # The scene mutation (respawn dummies, rebuild clutter) forces a full
        # Cycles re-sync that costs several times the 640x480 render itself
        # -- measured 22 % GPU utilisation with five parallel shards.  So
        # each arrangement is rendered from --cams viewpoints: sync once,
        # render K times.
        for twin in (0, 1):
            clear_dummies()
            twin_kinds = kinds if twin == 0 else \
                [{"human": "robot", "robot": "human"}[k] for k in kinds]
            per_dummy = []
            for g, (x, kind, pose) in enumerate(
                    zip(P.GOAL_X, twin_kinds, frame_poses)):
                if not occupied[g]:
                    continue
                objs = B.spawn_dummy(pose, kind, g, x, tops[g])
                _DUMMIES.extend(objs)
                per_dummy.append((kind, objs))

            for view in range(args.cams):
                sample_camera(rng, sc, cam, tops)
                jitter_lights(rng, sc)
                bpy.context.view_layer.update()
                name = f"s{args.shard}_{i:05d}_{twin}_{view}"
                lines = []
                for kind, objs in per_dummy:
                    if not visible(sc, cam, objs):
                        continue
                    bb = bbox_2d(sc, cam, objs)
                    if bb is None:
                        continue
                    x0, y0, x1, y1 = bb
                    w, h = x1 - x0, y1 - y0
                    if w * 640 < 5 or h * 480 < 5:
                        continue                 # too small to learn from
                    lines.append(
                        f"{CLS[kind]} {(x0+x1)/2:.6f} {(y0+y1)/2:.6f} "
                        f"{w:.6f} {h:.6f}")
                sc.render.filepath = os.path.join(
                    args.out, "images", split, name + ".jpg")
                bpy.ops.render.render(write_still=True)
                with open(os.path.join(args.out, "labels", split,
                                       name + ".txt"), "w") as f:
                    f.write("\n".join(lines) + ("\n" if lines else ""))
        if i % 25 == 0:
            print(f"PROGRESS shard {args.shard}: {i+1}/{args.n} pairs")
    print(f"DONE shard {args.shard}: {2*args.n} images")


main()
