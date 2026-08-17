"""Build the IDC2026 field scene in Blender and render a test frame.

Run:
    ~/apps/blender/blender -b -P blender/build_scene.py -- \
        --pose-id 0 --out /tmp/scene_test.png

Stage 1 of the render pipeline: geometry, materials, camera, lights.  The
dummy is assembled by parsing dummy.urdf directly -- every visual (lathe mesh
or joint-ball sphere) with its link-local offset lives there already, and the
pose JSONL stores world transforms per link, so no armature is needed for the
wooden figure.  (The human garment will need one; that is stage 2.)

Class appearance:
    robot  worn silver paint over wood, everywhere
    human  grey cloth material on torso/limbs, bare wood head/hands/feet
           (placeholder until the simulated garment lands -- the real figure
           wears real fabric)
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Quaternion, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene_params as P

MODEL = P.MODEL_DIR


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 96
    sc.cycles.use_denoising = True
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.get_devices()
    for dev in prefs.devices:
        dev.use = dev.type in ("OPTIX", "CUDA")
    sc.cycles.device = "GPU"
    sc.render.resolution_x = P.RENDER_W
    sc.render.resolution_y = P.RENDER_H
    sc.render.film_transparent = False
    # Standard, not Filmic/AgX: the material calibration is multiplicative in
    # linear light and its result is only valid under the same transform.
    sc.view_settings.view_transform = "Standard"
    return sc


def new_mat(name, base, rough=0.6, metallic=0.0, spec=None):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = base
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metallic
    if spec is not None:
        bsdf.inputs["Specular IOR Level"].default_value = spec
    return m


def link_obj(obj, collection=None):
    (collection or bpy.context.scene.collection).objects.link(obj)
    return obj


def box(name, size, loc, mat=None):
    mesh = bpy.data.meshes.new(name)
    bm_verts, bm_faces = _box_geom(size)
    mesh.from_pydata(bm_verts, [], bm_faces)
    mesh.update()
    o = bpy.data.objects.new(name, mesh)
    o.location = loc
    if mat:
        mesh.materials.append(mat)
    return link_obj(o)


def _box_geom(size):
    x, y, z = size[0] / 2, size[1] / 2, size[2] / 2
    v = [(-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
         (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]
    f = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1),
         (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    return v, f


# --------------------------------------------------------------------------
# field
# --------------------------------------------------------------------------

def field_material():
    """Lime mat tiles + black tape grid, procedural on world x/y."""
    m = bpy.data.materials.new("field")
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    bsdf.inputs["Roughness"].default_value = 0.55

    geo = nt.nodes.new("ShaderNodeNewGeometry")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    nt.links.new(geo.outputs["Position"], sep.inputs[0])

    def wave(src, scale):
        """0/1 checker along one axis with period 2*scale."""
        mul = nt.nodes.new("ShaderNodeMath"); mul.operation = "MULTIPLY"
        mul.inputs[1].default_value = 1.0 / scale
        nt.links.new(src, mul.inputs[0])
        flo = nt.nodes.new("ShaderNodeMath"); flo.operation = "FLOOR"
        nt.links.new(mul.outputs[0], flo.inputs[0])
        odd = nt.nodes.new("ShaderNodeMath"); odd.operation = "MODULO"
        odd.inputs[1].default_value = 2.0
        nt.links.new(flo.outputs[0], odd.inputs[0])
        return odd.outputs[0]

    # two-tone mat checker at the tape pitch
    cx = wave(sep.outputs["X"], P.TAPE_PITCH)
    cy = wave(sep.outputs["Y"], P.TAPE_PITCH)
    xor = nt.nodes.new("ShaderNodeMath"); xor.operation = "ABSOLUTE"
    sub = nt.nodes.new("ShaderNodeMath"); sub.operation = "SUBTRACT"
    nt.links.new(cx, sub.inputs[0]); nt.links.new(cy, sub.inputs[1])
    nt.links.new(sub.outputs[0], xor.inputs[0])
    tone = nt.nodes.new("ShaderNodeMixRGB")
    tone.inputs["Color1"].default_value = P.ALBEDO["field_bright"]
    tone.inputs["Color2"].default_value = P.ALBEDO["field_dark"]
    nt.links.new(xor.outputs[0], tone.inputs["Fac"])

    # Tape layout, from the venue photos: one lane runs DOWN-SLOPE through
    # the bore of every goal (it is the ball's guide line into the tunnel),
    # plus cross lines at the mat pitch.  The first version put the
    # down-slope lines on the 478 mm grid instead, which cannot align with
    # goals spaced 300/400 mm -- the user caught the mismatch on sight.
    # Anchoring the lanes to GOAL_X also means a corrected goal position
    # automatically drags its lane along.
    def lane_at(src, centre):
        d = nt.nodes.new("ShaderNodeMath"); d.operation = "SUBTRACT"
        nt.links.new(src, d.inputs[0]); d.inputs[1].default_value = centre
        a = nt.nodes.new("ShaderNodeMath"); a.operation = "ABSOLUTE"
        nt.links.new(d.outputs[0], a.inputs[0])
        lt = nt.nodes.new("ShaderNodeMath"); lt.operation = "LESS_THAN"
        lt.inputs[1].default_value = P.TAPE_W / 2
        nt.links.new(a.outputs[0], lt.inputs[0])
        return lt.outputs[0]

    def near_line(src):
        md = nt.nodes.new("ShaderNodeMath"); md.operation = "MODULO"
        md.inputs[1].default_value = P.TAPE_PITCH
        nt.links.new(src, md.inputs[0])
        lt = nt.nodes.new("ShaderNodeMath"); lt.operation = "LESS_THAN"
        lt.inputs[1].default_value = P.TAPE_W
        nt.links.new(md.outputs[0], lt.inputs[0])
        return lt.outputs[0]

    either = None
    for gx in P.GOAL_X:
        lane = lane_at(sep.outputs["X"], gx)
        if either is None:
            either = lane
        else:
            mx = nt.nodes.new("ShaderNodeMath"); mx.operation = "MAXIMUM"
            nt.links.new(either, mx.inputs[0])
            nt.links.new(lane, mx.inputs[1])
            either = mx.outputs[0]
    ty = near_line(sep.outputs["Y"])
    mx = nt.nodes.new("ShaderNodeMath"); mx.operation = "MAXIMUM"
    nt.links.new(either, mx.inputs[0]); nt.links.new(ty, mx.inputs[1])
    either = mx
    tape = nt.nodes.new("ShaderNodeMixRGB")
    tape.inputs["Color2"].default_value = P.ALBEDO["tape"]
    nt.links.new(either.outputs[0], tape.inputs["Fac"])
    nt.links.new(tone.outputs["Color"], tape.inputs["Color1"])
    nt.links.new(tape.outputs["Color"], bsdf.inputs["Base Color"])
    return m


def build_field():
    bpy.ops.wm.stl_import(filepath=P.FIELD_STL)
    field = bpy.context.selected_objects[0]
    field.name = "field"
    # Width correction: the organisers' STL is 2.70 m wide; the real field
    # measures narrower (see scene_params.FIELD_W).
    field.scale = (P.FIELD_W / P.FIELD_W_STL, 1.0, 1.0)
    field.data.materials.append(field_material())
    return field


def surface_z(y):
    """Field surface height at position y (slope rises with y)."""
    return P.FIELD_RISE * (y / P.FIELD_L)


# --------------------------------------------------------------------------
# goals
# --------------------------------------------------------------------------

def build_goal(i, x):
    """One tunnel: two side walls + level top, legs meeting the slope."""
    y = P.GOAL_EDGE_Y
    z_surf = surface_z(y)
    top_z = z_surf + P.GOAL_H_MEAN          # level top face height
    paint = new_mat(f"goal_paint_{i}", P.ALBEDO["goal_paint"], rough=0.5)
    ply = new_mat(f"goal_ply_{i}", P.ALBEDO["goal_ply"], rough=0.65)

    parts = []
    hw = P.GOAL_W / 2
    wall_h = P.GOAL_H_MEAN + 0.03           # sink 30 mm into the slope
    for sx in (-1, 1):
        parts.append(box(
            f"goal{i}_wall{sx}", (P.GOAL_WALL, P.GOAL_D, wall_h),
            (x + sx * (hw - P.GOAL_WALL / 2), y, top_z - wall_h / 2), paint))
    parts.append(box(f"goal{i}_top", (P.GOAL_W, P.GOAL_D, P.GOAL_WALL),
                     (x, y, top_z - P.GOAL_WALL / 2), paint))
    # raw ply inner faces, as thin inserts
    for sx in (-1, 1):
        parts.append(box(
            f"goal{i}_ply{sx}", (0.002, P.GOAL_D * 0.98, wall_h * 0.9),
            (x + sx * (hw - P.GOAL_WALL - 0.001), y, top_z - wall_h / 2),
            ply))
    return top_z


# --------------------------------------------------------------------------
# dummy from URDF
# --------------------------------------------------------------------------

def parse_urdf_links():
    """{link: [(kind, param, offset_xyz, rpy)]}, kind in {mesh, sphere}."""
    tree = ET.parse(os.path.join(MODEL, "dummy.urdf"))
    out = {}
    for link in tree.getroot().iter("link"):
        vis = []
        for v in link.findall("visual"):
            org = v.find("origin")
            xyz = [float(s) for s in (org.get("xyz") or "0 0 0").split()]
            rpy = [float(s) for s in (org.get("rpy") or "0 0 0").split()]
            geo = v.find("geometry")
            mesh = geo.find("mesh")
            if mesh is not None:
                name = os.path.splitext(
                    os.path.basename(mesh.get("filename")))[0]
                vis.append(("mesh", name, xyz, rpy))
            else:
                sph = geo.find("sphere")
                vis.append(("sphere", float(sph.get("radius")), xyz, rpy))
        if vis:
            out[link.get("name")] = vis
    return out


_MESH_CACHE = {}


def part_mesh(name):
    if name not in _MESH_CACHE:
        bpy.ops.wm.obj_import(filepath=os.path.join(MODEL, "meshes",
                                                    f"{name}.obj"))
        o = bpy.context.selected_objects[0]
        _MESH_CACHE[name] = o.data
        bpy.data.objects.remove(o)
    return _MESH_CACHE[name]


def dummy_materials(kind, idx):
    """kind: 'robot' or 'human'."""
    if kind == "robot":
        # Worn brushed paint over wood, not chrome: mostly a broad glossy
        # lobe showing the room, with enough dielectric body to hold colour.
        silver = new_mat(f"silver_{idx}", P.ALBEDO["silver"],
                         rough=0.45, metallic=0.6)
        return {"default": silver, "bare": silver}
    cloth = new_mat(f"cloth_{idx}", P.ALBEDO["cloth_grey"], rough=0.9)
    wood = new_mat(f"wood_{idx}", P.ALBEDO["wood_bare"], rough=0.55)
    return {"default": cloth, "bare": wood}


BARE_PARTS = {"head", "hand", "foot"}       # wood on the human figure


def spawn_dummy(pose, kind, idx, goal_x, goal_top_z):
    """Instantiate one posed figure above goal i.  Returns its objects."""
    links = parse_urdf_links()
    mats = dummy_materials(kind, idx)
    objs = []
    for lname, transform in pose["links"].items():
        px, py, pz, qw, qx, qy, qz = transform
        # pose frame: origin at goal top centre, z up
        world = (Matrix.Translation(
                    Vector((goal_x + px, P.GOAL_EDGE_Y + py,
                            goal_top_z + pz)))
                 @ Quaternion((qw, qx, qy, qz)).to_matrix().to_4x4())
        base = lname.rsplit("_", 1)[0] if lname[-2:] in ("_l", "_r") \
            else lname
        for kindv, param, xyz, rpy in links[lname]:
            off = (Matrix.Translation(Vector(xyz))
                   @ (Quaternion((1, 0, 0, 0)).to_matrix().to_4x4()
                      if rpy == [0, 0, 0] else
                      _rpy_matrix(rpy)))
            if kindv == "mesh":
                o = bpy.data.objects.new(f"d{idx}_{lname}_{param}",
                                         part_mesh(param).copy())
                mat = mats["bare" if param in BARE_PARTS else "default"]
            else:
                mesh = _ball_mesh(param)
                o = bpy.data.objects.new(f"d{idx}_{lname}_ball", mesh)
                mat = mats["default"]
            o.data.materials.clear()
            o.data.materials.append(mat)
            o.matrix_world = world @ off
            objs.append(link_obj(o))
    return objs


def _rpy_matrix(rpy):
    r, p, y = rpy
    return (Matrix.Rotation(y, 4, "Z") @ Matrix.Rotation(p, 4, "Y")
            @ Matrix.Rotation(r, 4, "X"))


_BALL_CACHE = {}


def _ball_mesh(radius):
    key = round(radius, 5)
    if key not in _BALL_CACHE:
        mesh = bpy.data.meshes.new(f"ball_{key}")
        import bmesh
        bm = bmesh.new()
        bmesh.ops.create_uvsphere(bm, u_segments=24, v_segments=16,
                                  radius=radius)
        bm.to_mesh(mesh)
        bm.free()
        _BALL_CACHE[key] = mesh
    return _BALL_CACHE[key].copy()


# --------------------------------------------------------------------------
# camera + light
# --------------------------------------------------------------------------

def build_camera(look_at):
    cam_data = bpy.data.cameras.new("imx500")
    cam_data.sensor_fit = "HORIZONTAL"
    cam_data.sensor_width = P.SENSOR_W_MM
    cam_data.lens = (P.SENSOR_W_MM / 2) / math.tan(
        math.radians(P.HFOV_DEG) / 2)
    cam = bpy.data.objects.new("cam", cam_data)
    x, y = P.START_A
    cam.location = (x, y, surface_z(y) + P.CAM_HEIGHT)
    direction = Vector(look_at) - cam.location
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    link_obj(cam)
    bpy.context.scene.camera = cam
    return cam


def build_lights():
    # One luminous ceiling as the ONLY light -- which is what the venue is: a
    # grid of fluorescent rows filling a white room with near-shadowless
    # light.  A single emitter keeps the calibration well-posed: the anchor
    # loop scales exactly the source that matters.  Earlier versions were
    # unfixable by construction -- world ambient never reaches inside the
    # closed room, and a secondary emitter fights the anchor's only knob.
    em = bpy.data.materials.new("lum_ceiling")
    em.use_nodes = True
    nt = em.node_tree
    nt.nodes.clear()
    emit = nt.nodes.new("ShaderNodeEmission")
    emit.inputs["Color"].default_value = tuple(P.LIGHT["color"]) + (1.0,)
    emit.inputs["Strength"].default_value = 1.0 * P.LIGHT["energy_scale"]
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(emit.outputs[0], out.inputs[0])
    lum = box("lum_ceiling", (11, 11, 0.02), (0, 1.0, 2.55), em)
    lum.visible_camera = False

    w = bpy.context.scene.world or bpy.data.worlds.new("w")
    bpy.context.scene.world = w
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.35, 0.36, 0.35, 1.0)
    bg.inputs[1].default_value = 0.2


def build_room(floor_z=-0.212):
    """Carpet floor plus white walls and ceiling.

    The walls matter beyond backdrop: the robot figure's silver paint is
    semi-glossy and mostly shows its SURROUNDINGS -- in the venue frames it
    reads as bright grey-green because it reflects white walls and lime
    bounce.  Without walls it reflected black world and calibrated wrong.
    """
    carpet = new_mat("carpet", P.ALBEDO["carpet"], rough=0.9)
    box("room_floor", (12, 12, 0.02), (0, 1.8, floor_z), carpet)
    wall = new_mat("wall", (0.75, 0.74, 0.70, 1.0), rough=0.8, spec=0.2)
    for name, size, loc in (
            ("wall_n", (12, 0.04, 3.2), (0, 6.5, floor_z + 1.6)),
            ("wall_s", (12, 0.04, 3.2), (0, -4.5, floor_z + 1.6)),
            ("wall_e", (0.04, 12, 3.2), (5.5, 1.0, floor_z + 1.6)),
            ("wall_w", (0.04, 12, 3.2), (-5.5, 1.0, floor_z + 1.6)),
            ("ceiling", (12, 12, 0.04), (0, 1.0, floor_z + 3.2))):
        box(name, size, loc, wall)


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose-id", type=int, default=0)
    ap.add_argument("--poses", default=os.path.join(MODEL, "poses.jsonl"))
    ap.add_argument("--out", default="/tmp/scene_test.png")
    ap.add_argument("--save-blend", default="")
    args = ap.parse_args(argv)

    poses = [json.loads(l) for l in open(args.poses)]

    reset_scene()
    build_field()
    build_room()
    tops = [build_goal(i, x) for i, x in enumerate(P.GOAL_X)]

    # one match layout: our pair human+robot, opponent pair mirrored classes
    order = ["human", "robot", "robot", "human"]
    for i, (x, kind) in enumerate(zip(P.GOAL_X, order)):
        pose = poses[(args.pose_id + i) % len(poses)]
        spawn_dummy(pose, kind, i, x, tops[i])

    build_camera(look_at=(0.0, P.GOAL_EDGE_Y, tops[0]))
    build_lights()

    if args.save_blend:
        bpy.ops.wm.save_as_mainfile(filepath=args.save_blend)
    bpy.context.scene.render.filepath = args.out
    bpy.ops.render.render(write_still=True)
    print(f"rendered {args.out}")


# Blender gives -P scripts __name__ == "__main__"; calib_scene.py imports
# this module for its helpers and must not trigger a full scene build.
if __name__ == "__main__":
    main()
