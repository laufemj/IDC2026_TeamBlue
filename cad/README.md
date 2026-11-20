# Parametric CAD representation

[Hardware archive](../hardware/README.md) · [Project report](../docs/project_report.md)

`autonomous_robot.py` describes a rev-B ground-level funnel, corridor, chassis,
camera mast and electronics layout. `autonomous_robot_extended.py` selects an
extended-arm pose. Dimensions use millimetres: +X forward, +Y left and +Z up,
with the origin on the floor midway between wheel contact patches.

This is a design representation with simplified geometry, not an exact digital
twin or a validated fabrication assembly. Its comments explicitly omit gear
teeth and rack guides, and the represented gate/actuator arrangement differs
from the final mechanism described in the team report.

## Dependencies and missing inputs

The scripts import `build123d` and `cadpy.assembly.AssemblyHelper`. The latter is
an external helper absent from this repository. They also load component STEP
files from `parts/`, including Raspberry Pi 5 and Camera Module 3 models; these
files are not versioned here. See the import paths in the generator before
attempting regeneration. Installing `build123d` alone is insufficient.

The sources define assembly-building functions; running the script does not by
itself provide a documented standalone export workflow. The local source folder
also contained a slightly different generator copy; it was not substituted for
the version already tracked in this repository.

For geometry inspection, use the supplied Fusion designs and neutral-format
files in [hardware/](../hardware/README.md). Full generated assembly STEP files
and cached GLBs were omitted from the import because of their size and duplicate
representation. The [asset manifest](../docs/asset_manifest.json) identifies
the retained originals.
