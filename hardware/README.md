# Hardware archive

[Project report](../docs/project_report.md) · [Evidence notes](../docs/evidence.md) · [File origins and checksums](../docs/asset_manifest.json) · [Parametric CAD](../cad/README.md)

This folder preserves selected files supplied with the IDC2026 project. They
are byte-identical copies, organized by format. Original revision labels remain
in filenames; “Final” and “v3” are source labels, not a new verification of which
parts were fabricated or used in the competition.

## Native designs

| File | Source context |
|---|---|
| [Autonomous Robot.f3d](fusion/Autonomous%20Robot.f3d) | Main Fusion design from the project-folder root |
| [BaseWall_Left.f3d](fusion/BaseWall_Left.f3d) | Left wall design from `Things To Print` |
| [BaseWall_Right.f3d](fusion/BaseWall_Right.f3d) | Right wall design from `Things To Print` |
| [Gear_Right_Final.f3d](fusion/Gear_Right_Final.f3d) | Gear design carrying the source's “Final” revision label |

Open `.f3d` files in Fusion. Their internal design state has not been validated
against the final physical assembly.

## Fabrication references

| Directory | Selection | Use |
|---|---|---|
| [fabrication/dxf/](fabrication/dxf/) | All supplied DXF files: boards, arm parts, connectors, dividing wall and flap; includes alternate board revisions | Inspect 2D cutting geometry and choose the intended revision |
| [fabrication/step/](fabrication/step/) | Selected gears, mounts, rack/pinion, servo tray and walls | Inspect neutral-format part/assembly geometry |
| [fabrication/stl/](fabrication/stl/) | Supplied `Autonomous Robot.stl` | View the supplied mesh representation |

File formats can be opened in a compatible CAD viewer without the Python
generator. Check units, dimensions, material thickness, fits and assembly
clearances before fabrication; the archive does not provide a validated bill
of materials or assembly procedure. No teleoperated robot CAD was identified
in the supplied selection; its mechanism is documented in the report and photos.

## Selection and provenance

The [manifest](../docs/asset_manifest.json) gives each file's original relative
path, byte size and SHA-256. The import preserves source geometry and names;
it does not reconcile all intermediate revisions into a new “final” design.

Large generated STEP/GLB assembly exports, `__pycache__`, duplicate ZIP bundles,
printer-specific `.gx`/`.print`/sliced `.3mf` jobs and the third-party MG996R model
bundle were omitted. The original selected STL is explicitly included despite
the repository's general ignore rule for generated meshes.

No new license or authorship attribution is inferred from possession of these
files. Supplied third-party component models are not redistributed here.
