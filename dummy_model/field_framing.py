#!/usr/bin/env python3
"""What the IMX500 actually sees from Start A -- framing + pixel budget.

Answers the open question from the design review: at the deployed camera pose,
how many goals fall in frame, how many pixels tall is a dummy, and what camera
pitch is needed.  Everything here is geometry, so it settles the question
before any training image is generated.

Camera model (Raspberry Pi AI Camera, IMX500):
    4056 x 3040 sensor, HFOV 66.3 deg, VFOV 52.3 deg, square pixels.
    The network input is 640x640 letterboxed, so the 4:3 frame occupies
    640 x 480 and fx = fy = 320 / tan(HFOV/2) = 490.3 px.

Scene geometry is the user's tape measurements, not the STL:
    goal tunnel   200 (W) x 195 (D) x 191..212 (H) mm, 12 mm walls
    field slope   5 deg
    dummy         300 mm standing
Goal spacing is taken from scene_IDC2026.xml (0.30 m) but that layout is
flagged as unverified in the repo README -- pass --goal-spacing to override.

Usage:
    python3 field_framing.py --distance 2.0 --cam-height 0.15
"""

import argparse
import math
import os

import mujoco
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))

HFOV_DEG = 66.3
NET = 640                     # network input, square
CONTENT_H = 480               # 4:3 content inside the letterbox
FX = (NET / 2) / math.tan(math.radians(HFOV_DEG) / 2)

TUNNEL_W, TUNNEL_D = 0.200, 0.195
TUNNEL_H_LOW, TUNNEL_H_HIGH = 0.191, 0.212
WALL = 0.012
FIELD_SLOPE_DEG = 5.0
DUMMY_H = 0.300


def px_height(metres, distance):
    """Apparent height in network pixels of an object at a given range."""
    return FX * metres / distance


def scene_xml(goal_spacing, distance, cam_height, cam_pitch_deg, n_goals):
    """Field + tunnels + placeholder dummy boxes, seen through the IMX500."""
    tunnel_h = 0.5 * (TUNNEL_H_LOW + TUNNEL_H_HIGH)
    goals = [(i - (n_goals - 1) / 2) * goal_spacing for i in range(n_goals)]

    bodies = []
    for i, gx in enumerate(goals):
        # Tunnel walls: two sides plus a roof, leaving the bore clear so the
        # "must not obstruct ball passage" rule stays visible in the render.
        half_w, half_d = TUNNEL_W / 2, TUNNEL_D / 2
        bodies.append(f"""
    <body name="goal{i}" pos="{gx:.4f} 0 0">
      <geom type="box" size="{WALL/2:.4f} {half_d:.4f} {tunnel_h/2:.4f}"
            pos="{-half_w+WALL/2:.4f} 0 {tunnel_h/2:.4f}" rgba="0.93 0.93 0.91 1"/>
      <geom type="box" size="{WALL/2:.4f} {half_d:.4f} {tunnel_h/2:.4f}"
            pos="{ half_w-WALL/2:.4f} 0 {tunnel_h/2:.4f}" rgba="0.93 0.93 0.91 1"/>
      <geom type="box" size="{half_w:.4f} {half_d:.4f} {WALL/2:.4f}"
            pos="0 0 {tunnel_h-WALL/2:.4f}" rgba="0.93 0.93 0.91 1"/>
    </body>""")
        # Placeholder for the figure: a 300 mm bar standing on the goal top,
        # pelvis in contact, which is the rulebook constraint.
        bodies.append(f"""
    <body name="fig{i}" pos="{gx:.4f} 0 {tunnel_h:.4f}">
      <geom type="capsule" size="0.030 {DUMMY_H/2-0.03:.4f}"
            pos="0 0 {DUMMY_H/2:.4f}"
            rgba="{'0.80 0.80 0.83 1' if i%2==0 else '0.42 0.42 0.45 1'}"/>
    </body>""")

    # Camera sits `distance` back along -Y, looking at the goal line.
    cx = sum(goals) / len(goals)
    return f"""
<mujoco model="framing">
  <visual><global offwidth="{NET}" offheight="{CONTENT_H}"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.62 0.78 0.13" rgb2="0.58 0.74 0.11"/>
    <material name="field" texture="grid" texrepeat="6 8" specular="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8"/>
    <light pos="1 -2 2" dir="-0.4 0.8 -0.9" directional="true" diffuse="0.3 0.3 0.3"/>
    <geom name="field" type="plane" size="2 3 0.05" material="field"
          euler="{FIELD_SLOPE_DEG:.3f} 0 0"/>
    {''.join(bodies)}
    <camera name="robot" pos="{cx:.4f} {-distance:.4f} {cam_height:.4f}"
            euler="{90-cam_pitch_deg:.3f} 0 0"/>
  </worldbody>
</mujoco>"""


def letterbox(img):
    """Pad a 640x480 render to the 640x640 the network actually receives."""
    out = Image.new("RGB", (NET, NET), (114, 114, 114))
    out.paste(img, (0, (NET - CONTENT_H) // 2))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--distance", type=float, default=2.0)
    ap.add_argument("--cam-height", type=float, default=0.15)
    ap.add_argument("--cam-pitch", type=float, default=4.0,
                    help="downward tilt, degrees")
    ap.add_argument("--goal-spacing", type=float, default=0.30)
    ap.add_argument("--n-goals", type=int, default=4)
    ap.add_argument("--out", default=os.path.join(HERE, "framing_check.png"))
    args = ap.parse_args()

    print(f"IMX500 @ 640x640 letterbox:  fx = fy = {FX:.1f} px")
    print(f"  horizontal FOV {HFOV_DEG} deg -> "
          f"{2*args.distance*math.tan(math.radians(HFOV_DEG)/2):.2f} m wide "
          f"at {args.distance:.1f} m")
    print()
    print("  dummy apparent height (network px):")
    for d in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
        std = px_height(DUMMY_H, d)
        # A pelvis-pinned figure curled up projects far smaller than standing.
        curled = px_height(0.18, d)
        print(f"    {d:4.1f} m   standing {std:5.1f} px   curled {curled:5.1f} px")
    print()

    span = args.goal_spacing * (args.n_goals - 1) + TUNNEL_W
    fov_w = 2 * args.distance * math.tan(math.radians(HFOV_DEG) / 2)
    print(f"  {args.n_goals} goals span {span:.2f} m; frame is {fov_w:.2f} m wide "
          f"-> {'ALL IN FRAME' if span < fov_w else 'DOES NOT FIT'}")

    xml = scene_xml(args.goal_spacing, args.distance, args.cam_height,
                    args.cam_pitch, args.n_goals)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, CONTENT_H, NET)
    r.update_scene(d, camera="robot")
    img = letterbox(Image.fromarray(r.render()))
    img.save(args.out)
    print(f"\nwrote {args.out}  ({NET}x{NET}, exactly what the network sees)")


if __name__ == "__main__":
    main()
