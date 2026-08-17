#!/usr/bin/env python3
"""Iterate Blender material albedos until renders match the venue photos.

    python3 blender/calibrate.py [--iters 4]

Targets are the patch medians sampled from the real IMX500 venue frame
(venue_swatches.json).  Those carry the venue's illumination and the
camera's white balance baked in, which is exactly the point: the training
renders should look like THAT camera in THAT room, not like a neutral
studio.

Loop:
  0. render the calibration wall; scale the lighting's energy and colour so
     the white goal paint patch matches its target (the illuminant anchor),
  1+. for every other material, update its albedo multiplicatively in
     linear light: albedo *= target/render per channel.  Diffuse response
     is linear in albedo, so this converges in 2-3 rounds.

State lives in calibrated_albedo.json + calibrated_light.json, which
scene_params.py overlays on its defaults, so the training scene picks the
result up with no further wiring.
"""

import argparse
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.dirname(HERE)
BLENDER = os.path.expanduser("~/apps/blender/blender")

ALBEDO_JSON = os.path.join(HERE, "calibrated_albedo.json")
LIGHT_JSON = os.path.join(HERE, "calibrated_light.json")
TARGETS = json.load(open(os.path.join(MODEL, "venue_swatches.json")))

# calibration patch -> venue swatch name
PAIRS = {
    "field_bright": "field_bright",
    "field_dark": "field_dark",
    "tape": "tape_black",
    "goal_paint": "goal_top_white",
    "goal_ply": "goal_rawply_out",
    "silver": "robot_silver",
    "cloth_grey": "human_cloth",
    "wood_bare": "human_head_wood",
}
ANCHOR = "goal_paint"
HALF = 16                       # patch half-size, px


def srgb_to_linear(c):
    c = np.asarray(c, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def render():
    out, pat = "/tmp/calib.png", "/tmp/calib_patches.json"
    r = subprocess.run(
        [BLENDER, "-b", "-P", os.path.join(HERE, "calib_scene.py"),
         "--", "--out", out, "--patches", pat],
        capture_output=True, text=True, cwd=MODEL)
    if "calib render done" not in r.stdout:
        sys.exit(f"blender failed:\n{r.stdout[-3000:]}\n{r.stderr[-2000:]}")
    img = np.asarray(Image.open(out).convert("RGB")).astype(float)
    locs = json.load(open(pat))
    med = {}
    for name, (cx, cy) in locs.items():
        p = img[max(cy - HALF, 0):cy + HALF, max(cx - HALF, 0):cx + HALF]
        med[name] = np.median(p.reshape(-1, 3), axis=0)
    return med


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=4)
    args = ap.parse_args()

    sys.path.insert(0, HERE)

    for it in range(args.iters):
        med = render()
        tgt = {k: np.array(TARGETS[v], float) for k, v in PAIRS.items()}

        # --- illuminant anchor, EVERY iteration (cumulative) ---------------
        # One shot is not enough: the first render can be an order of
        # magnitude too dark, and a single multiplicative step computed from
        # near-noise pixels under-corrects (measured: x8 applied where x44
        # was needed, after which every albedo slammed into its cap chasing
        # light that was not there).
        light = json.load(open(LIGHT_JSON)) if os.path.exists(LIGHT_JSON) \
            else {"energy_scale": 1.0, "color": [1.0, 0.98, 0.92]}
        anchor_err = float(np.abs(med[ANCHOR] - tgt[ANCHOR]).mean())
        if anchor_err > 8.0:
            # Damped updates (gain**0.7), and the colour is kept near-white:
            # the undamped version oscillated through a nonphysical
            # blue-dominant illuminant and never settled.
            r_lin = srgb_to_linear(med[ANCHOR])
            t_lin = srgb_to_linear(tgt[ANCHOR])
            gain_e = float(np.clip(t_lin.mean() / max(r_lin.mean(), 1e-6),
                                   0.2, 20.0)) ** 0.7
            light["energy_scale"] = float(np.clip(
                light["energy_scale"] * gain_e, 0.05, 100.0))
            gain_c = np.clip(t_lin / np.maximum(r_lin, 1e-6)
                             / max(t_lin.mean() / max(r_lin.mean(), 1e-6),
                                   1e-6), 0.8, 1.25) ** 0.7
            colour = np.array(light["color"]) * gain_c
            colour = np.clip(colour / colour.max(), 0.6, 1.0)
            light["color"] = colour.tolist()
            json.dump(light, open(LIGHT_JSON, "w"), indent=1)
        else:
            colour = np.array(light["color"])
        print(f"[{it}] anchor err {anchor_err:5.1f}  light x"
              f"{light['energy_scale']:.2f}  colour "
              f"({colour[0]:.2f},{colour[1]:.2f},{colour[2]:.2f})")
        if anchor_err > 12.0:
            continue        # albedos wait until the light is right

        # --- ambient fill (second illumination DOF) ------------------------
        # Only once a curved sample's albedo is pinned at the cap does its
        # remaining luminance deficit measure missing shadow fill; before
        # that it just measures albedo that has not been raised yet.  (The
        # first version gated albedo updates behind this step and cranked
        # ambient x12 chasing a 0.30 default cloth albedo.)
        mesh_names = ("silver", "cloth_grey", "wood_bare")
        albedo_state = json.load(open(ALBEDO_JSON)) \
            if os.path.exists(ALBEDO_JSON) else {}
        capped = [n for n in mesh_names
                  if max(albedo_state.get(n, (0, 0, 0))) >= 0.94]
        if capped:
            deficit = float(np.mean([
                srgb_to_linear(tgt[n]).mean()
                / max(srgb_to_linear(med[n]).mean(), 1e-6)
                for n in capped]))
            if deficit > 1.15:
                light["ambient"] = float(np.clip(
                    light.get("ambient", 0.25) * deficit ** 0.5, 0.02, 1.5))
                json.dump(light, open(LIGHT_JSON, "w"), indent=1)
                print(f"    ambient -> {light['ambient']:.3f} "
                      f"(capped {capped}, deficit x{deficit:.2f})")

        # --- albedo updates ------------------------------------------------
        import scene_params as P
        albedo = json.load(open(ALBEDO_JSON)) if os.path.exists(ALBEDO_JSON) \
            else {}
        worst = 0.0
        print(f"    patch          render          target      dE")
        for name in PAIRS:
            r8, t8 = med[name], tgt[name]
            de = float(np.abs(r8 - t8).mean())
            print(f"    {name:13s} ({r8[0]:3.0f},{r8[1]:3.0f},{r8[2]:3.0f})"
                  f" -> ({t8[0]:3.0f},{t8[1]:3.0f},{t8[2]:3.0f})  {de:5.1f}")
            if name == ANCHOR:
                continue
            worst = max(worst, de)
            cur = np.array(albedo.get(name, P.ALBEDO[name][:3]), float)
            gain = np.clip(srgb_to_linear(t8) /
                           np.maximum(srgb_to_linear(r8), 1e-5), 0.2, 5.0)
            need = cur * gain
            if need.max() > 0.95:
                # Curved samples shade darker than a flat card, so a too-dim
                # patch can demand super-white albedo in every channel; naive
                # per-channel clipping then pins the material at NEUTRAL
                # 0.95^3 and wipes out its chroma -- for the bare wood, the
                # one colour cue separating human from robot.  Rescale the
                # DEMAND vector instead: its ratios already account for the
                # (greenish) illuminant, so this keeps hue and gives up
                # luminance uniformly.  (Substituting the target's own sRGB
                # chroma here double-applies the illuminant tint -- tried,
                # drove the wood green.)
                need = need / need.max() * 0.95
            albedo[name] = np.clip(need, 0.003, 0.95).tolist()
        json.dump(albedo, open(ALBEDO_JSON, "w"), indent=1)
        if worst < 6.0:
            print(f"converged: worst mean channel error {worst:.1f}/255")
            break
    print("final albedos ->", ALBEDO_JSON)


if __name__ == "__main__":
    main()
