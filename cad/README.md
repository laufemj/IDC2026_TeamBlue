# Autonomous robot CAD

[Project home](../README.md) · [Engineering overview](../docs/project-overview.md)

| Generator | Purpose |
|---|---|
| `autonomous_robot.py` | Rev B ground-level ball-sorter assembly |
| `autonomous_robot_extended.py` | Extended robot assembly variant |

The generators use `build123d` and `cadpy.assembly.AssemblyHelper`. The latter
is an additional environment dependency, not bundled here. Review imports and
export paths before running the scripts in a CAD environment.

Units are millimetres. The assembly datum is the floor point midway between
wheel contact patches, with +X forward, +Y left and +Z upward. Geometry includes
the MDF structure, drive layout and retractable arms. Some gear features use
envelopes rather than fabrication-ready teeth.

Generated STEP, mesh and intermediate files are not comprehensively versioned.
These are design/reconstruction sources, not certified as-built manufacturing
drawings. See [recovery records](../docs/pi_session_recovery.md) for dimensions
and superseded assumptions.
