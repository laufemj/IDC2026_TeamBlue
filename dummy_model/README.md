# Synthetic-data tooling

[Project home](../README.md) · [Detector weights](../models/README.md)

Tools for synthetic human/robot figure data and the goal-identification detector.

```text
Mannequin geometry → MuJoCo pose settling → Blender rendering
                  → labelled dataset → YOLO training → real-photo evaluation
```

| Files | Purpose |
|---|---|
| `dummy_params.py`, `gen_dummy.py`, `dummy.urdf` | Parameterized mannequin and URDF |
| `make_mjcf.py`, `sample_poses.py`, `calibrate_friction.py` | Simulation conversion, poses and friction experiments |
| `view_dummy.py`, `preview_poses.py` | Geometry and pose inspection |
| `blender/scene_params.py`, `blender/build_scene.py` | Arena, materials and camera setup |
| `blender/calib_scene.py`, `blender/calibrate.py` | Scene calibration |
| `blender/render_dataset.py` | Randomized renders and YOLO labels |
| `train_v1.py` | Historical v1 training script |
| `infer_real.py`, `analyze_real.py`, `field_framing.py` | Real-photo evaluation and framing |

## Reproduction scope

The source is available, but the complete pipeline is not self-contained:

- Datasets, sampled poses and generated meshes are excluded by `.gitignore`.
- Blender references organizer assets under `IDC2026_mujoco/assets/parts`,
  which are not included in this checkout.
- Rendering requires Blender's Python environment (`bpy`); simulation needs
  MuJoCo. Read each script's imports and docstring for its inputs.
- `train_v1.py` retains an original workstation path and early configuration.
  It is not an exact recipe for the final `v25f2` checkpoint.
- For inference, use the [model artifacts](../models/README.md) and
  [desktop example](../docs/getting-started.md).

Real photos calibrated materials and lighting and supported evaluation. They
were not used as detector training images.
