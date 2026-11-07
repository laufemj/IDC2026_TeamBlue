# Getting started

[Project home](../README.md) · [Documentation index](README.md)

Desktop examples work without a robot. Hardware operation requires a Raspberry
Pi, ROS 2, calibration and missing integration work; this checkout is not a
ready-to-run competition image.

## 1. Check the navigation logic

From the repository root, using Python 3:

```bash
python nav/test_logic.py
```

The script contains 25 checks and needs no third-party packages. It exercises
steering, goal-side decisions and mission logic, not hardware, camera sharing
or the original competition deployment.

## 2. Run the sample colour classifier

Use a virtual environment if desired, then run:

```bash
python -m pip install numpy pillow
python ball_color/ball_color_node.py --images "ball_color/samples/*.png"
```

The folder contains two orange crops, one silver crop and two distractors. A
light neutral distractor can be labelled silver: colour alone cannot establish
ball presence. The robot needs a calibrated pocket and empty-reference presence
gating. See [the perception guide](../ball_color/README.md).

## 3. Try the figure detector

Install `ultralytics` into a suitable Python environment. Run this Python code
from the repository root, replacing `your_photo.jpg` with a real input image:

```python
from ultralytics import YOLO

model = YOLO("models/dummy_yolo11n_v25f2.pt")
results = model.predict(
    "your_photo.jpg",
    imgsz=640,
    conf=0.25,
    agnostic_nms=True,
)
```

This is raw inference. Edge filtering, close-range gating and goal-side voting
belong to the downstream policy. See [model metadata](../models/README.md).

## 4. Before running on the Raspberry Pi

| Requirement | Current status |
|---|---|
| ROS 2 | Source an installation with `rclpy`, `std_msgs`, `geometry_msgs` and `vision_msgs` |
| Cameras | Needs `picamera2`/`libcamera`, Camera Module 3 and mast camera; camera ownership across processes needs integration checks |
| GPIO | Needs a Pi-compatible `gpiozero` backend and verified wiring |
| Figure detections | A separate node must publish `/dummy_detections`; the launcher does not start one |
| Motor pins | Recovered defaults: BCM 12/17 and 13/23; verify left/right assignment and polarity |
| Servo pins | Placeholder BCM 12 and 13 overlap motor defaults; assign verified, non-conflicting pins before use |
| Calibration | Verify servo travel, motor trim, power, camera pose, colour references and `TODO-FIELD` values |
| Full mission | This archive is not the original contest implementation; validate the full sequence before a physical run |

The detector interface is `vision_msgs/Detection2DArray` on
`/dummy_detections`, with normalized coordinates and class IDs `0`/`human` or
`1`/`robot`. See [the navigation contract](../nav/README.md).

After resolving those requirements, the existing Linux/Pi launcher accepts:

```bash
bash nav/launch_all.sh <n_balls> <red|blue> <HR|RH>
```

`HR` means human left / robot right; `RH` means robot left / human right. These
are fallback goal assignments. The script exposes only these three match
arguments; resolve pin configuration separately in the nodes or launch setup.

## 5. Rendering, training and CAD

- [Synthetic-data tooling](../dummy_model/README.md): scripts and missing inputs.
  The full dataset and source arena assets are not versioned here.
- [CAD guide](../cad/README.md): generators and dependencies.
- [Recovery notes](pi_session_recovery.md): recovered values versus unavailable
  original Pi-only code.
