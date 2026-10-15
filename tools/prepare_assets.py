# Prepares the FPV drone plugin's runtime assets (no cooking, no pak). They are the input to tools/pack,
# which encrypts them into the DLL; nothing here ships as loose files.
#
#   Assets/goggles_mount.obj        G24 shroud plate + hinge housing (stays on the helmet)      group MI_G24
#   Assets/goggles_fold.obj         G24 rack + slider, DJI bracket, DJI Goggles 2 (flips)      groups MI_G24, MI_Goggles
#   Assets/drone_body.obj           DJI Mini 4K body                                           group MI_Drone
#   Assets/drone_camera.obj         its gimbal camera, around its own pivot                    group MI_Drone
#   Assets/drone_prop_{FL,FR,BL,BR}.obj  propellers, each around its own hub                   group MI_Drone
#   Assets/drone_release.obj        payload release under the body                             group MI_Bomb
#   Assets/drone_bomb.obj           the payload, around its own centre                         group MI_Bomb
#   Assets/{Goggles,G24,Drone,Bomb}_{Diffuse,Normal,ORM}.png   textures for M_DefaultShader
#   Assets/Icon_Goggles.png         the gear menu icon
#   Assets/fpv.txt                  pivots and sizes the plugin needs (cm, UE axes)
#
# Input: the two 3DMA downloads, extracted (not in the repo):
#   set FPV_GOGGLES_DIR=...\Goggles_DJI     (Mesh\DJI_Goggles_V2.blend, Textures\)
#   set FPV_DRONE_DIR=...\DJI_4k_v01        (Mesh\DIJI_4k.blend, Textures\)
#   blender.exe --background --python tools\prepare_assets.py
# The .blend files themselves are not changed.
#
# Frames. Goggles: the game hangs its NVG actor on the VR camera, so the goggles are exported in camera
# axes (x forward, y right, z up) around the G24's hinge (the rack's knuckle), and fpv.txt says where that
# hinge is from the eye. The eye is taken 2 cm behind the goggles' face cushion, level with its centre.
# Drone: x forward (the camera end), y right, z up, around the body's centre; parts that move are exported
# around their own pivot, which fpv.txt gives in the drone's frame.
#
# GML's BLENDER_OBJ import turns Blender (x, y, z) into UE (x, -y, z) * 100, so every part is written in
# the Blender coordinates that land on the UE ones above (a turn about z, never a mirror).
#
# Textures. Goggles, G24 and payload keep the models' own UVs; their textures are resized. The drone's
# atlas maps its two arms onto the same texels and its "MINI 4K" lettering is a floating decal with an
# opacity map, which the game's opaque weapon shader cannot show. So body, camera and propellers get one
# new non-overlapping layout, the atlas is baked onto it (Cycles: colour, AO, roughness, metallic as
# emission, the normal in tangent space) and the decals are baked onto it from just above the surface and
# blended in. The game imports loose PNGs as sRGB, so Normal and ORM (linear data) are pre-encoded with
# the sRGB curve; ORM is R = AO, G = roughness, B = metallic. Blender-style (OpenGL) normals get green
# flipped for UE; the goggles ship DirectX normals, used as they are.
import bpy, bmesh, math, os
import numpy as np
from mathutils import Vector, Matrix

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOG = os.environ.get("FPV_GOGGLES_DIR", "")
DRN = os.environ.get("FPV_DRONE_DIR", "")
OUT = os.path.join(REPO, "Assets")
for d, what in ((GOG, "FPV_GOGGLES_DIR (the extracted Goggles_DJI folder)"), (DRN, "FPV_DRONE_DIR (the extracted DJI_4k_v01 folder)")):
    if not os.path.isdir(os.path.join(d, "Mesh")):
        raise SystemExit("set " + what)
os.makedirs(OUT, exist_ok=True)

EYE_RELIEF = 0.02        # m behind the goggles' face cushion
DRONE_TEX = 2048
BAKE_MARGIN = 8
manifest = []

# ------------------------------------------------------------------ helpers

def to_srgb(v):
    return np.where(v <= 0.0031308, v * 12.92, 1.055 * np.power(np.clip(v, 0, 1), 1 / 2.4) - 0.055)

def load(path):
    """A PNG's raw values (no colour conversion), as float RGBA rows bottom-up (Blender's order)."""
    img = bpy.data.images.load(path, check_existing=False)
    img.colorspace_settings.name = 'Non-Color'
    w, h = img.size
    px = np.array(img.pixels[:], dtype=np.float32).reshape(h, w, 4)
    bpy.data.images.remove(img)
    return px

def resize(px, size):
    h, w = px.shape[:2]
    if (w, h) == (size, size):
        return px
    img = bpy.data.images.new("rs", w, h, alpha=True, float_buffer=True)
    img.colorspace_settings.name = 'Non-Color'
    img.pixels[:] = px.ravel()
    img.scale(size, size)
    out = np.array(img.pixels[:], dtype=np.float32).reshape(size, size, 4)
    bpy.data.images.remove(img)
    return out

def save(px, name):
    h, w = px.shape[:2]
    img = bpy.data.images.new(name, w, h, alpha=True)
    img.colorspace_settings.name = 'Non-Color'
    img.pixels[:] = np.clip(px, 0, 1).ravel()
    img.filepath_raw = os.path.join(OUT, name + ".png")
    img.file_format = 'PNG'
    img.save()
    bpy.data.images.remove(img)
    print("WROTE", name, w)

def texture_set(name, base, normal, ao, rough, metal, size, flip_green, normal_size=None):
    d = resize(base, size); d[..., 3] = 1
    save(d, name + "_Diffuse")                                  # already sRGB-encoded in the file
    n = resize(normal, normal_size or size)
    if flip_green:
        n[..., 1] = 1 - n[..., 1]
    n[..., :3] = to_srgb(n[..., :3]); n[..., 3] = 1
    save(n, name + "_Normal")
    o = max(256, size // 2)
    orm = np.ones((o, o, 4), dtype=np.float32)
    orm[..., 0] = resize(ao, o)[..., 0] if ao is not None else 1.0
    orm[..., 1] = resize(rough, o)[..., 0]
    orm[..., 2] = resize(metal, o)[..., 0]
    orm[..., :3] = to_srgb(orm[..., :3])
    save(orm, name + "_ORM")

def world_copy(objs, name, matrix=Matrix.Identity(4)):
    """One new object: the meshes of `objs` with their world transforms and `matrix` baked in, joined."""
    copies = []
    for o in objs:
        me = o.data.copy()
        me.transform(o.matrix_world)
        me.transform(matrix)
        c = bpy.data.objects.new(name, me)
        bpy.context.scene.collection.objects.link(c)
        copies.append(c)
    for o in bpy.context.view_layer.objects:
        o.select_set(o in copies)
    bpy.context.view_layer.objects.active = copies[0]
    if len(copies) > 1:
        bpy.ops.object.join()
    out = bpy.context.view_layer.objects.active
    out.name = name
    return out

def export(obj, name):
    for o in bpy.context.view_layer.objects:
        o.select_set(o == obj)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.wm.obj_export(filepath=os.path.join(OUT, name + ".obj"), export_selected_objects=True, export_materials=True,
                          export_material_groups=False, export_uv=True, export_normals=True, forward_axis='NEGATIVE_Z',
                          up_axis='Y', global_scale=1.0, path_mode='STRIP')
    mtl = os.path.join(OUT, name + ".mtl")
    if os.path.exists(mtl):
        os.remove(mtl)
    co = np.array([tuple(v.co) for v in obj.data.vertices])
    ue = np.stack([co[:, 0], -co[:, 1], co[:, 2]], 1) * 100
    print(f"WROTE {name}.obj  UE X[{ue[:,0].min():.2f}..{ue[:,0].max():.2f}] Y[{ue[:,1].min():.2f}..{ue[:,1].max():.2f}] "
          f"Z[{ue[:,2].min():.2f}..{ue[:,2].max():.2f}]")

def frame(m3, origin):
    """4x4: Blender world point B -> GML-importable Blender coords of UE = m3 (B - origin) in metres."""
    flip = Matrix(((1, 0, 0), (0, -1, 0), (0, 0, 1)))   # GML: UE = (x, -y, z) * 100, so pre-apply its inverse
    r = (flip @ m3).to_4x4()
    return r @ Matrix.Translation(-origin)

def fmt(v):
    return " ".join(f"{x:.3f}" for x in v)

# ------------------------------------------------------------------ goggles

bpy.ops.wm.open_mainfile(filepath=os.path.join(GOG, "Mesh", "DJI_Goggles_V2.blend"))
O = bpy.data.objects
mount = [O["Mount_G24_Body"], O["Mount_G24_Body2"]]                     # on the helmet
rack = O["Mount_G24_Rack_S"]                                             # the short rack and what hangs off it
fold = [rack, O["Mount_Slider_G24.001"], O["Mount_Custom_DJI.001"], O["Goggles_V2_DJI.001"]]
gog = O["Goggles_V2_DJI.001"]

hinge = rack.matrix_world.translation.copy()       # the rack turns about its knuckle; its origin sits on it
gw = np.array([tuple(gog.matrix_world @ v.co) for v in gog.data.vertices])
cushion = gw[:, 1].max()                           # the face side (the goggles look along -y)
rear = gw[gw[:, 1] > cushion - 0.01]
eye = Vector((0.0, cushion + EYE_RELIEF, (rear[:, 2].min() + rear[:, 2].max()) / 2))
# Blender -> camera axes: the wearer looks along -y with z up, so forward = -y and right = -x.
CAM =Matrix(((0, -1, 0), (-1, 0, 0), (0, 0, 1)))
h_ue = CAM @ (hinge - eye) * 100
manifest.append(f"goggles.hinge {fmt(h_ue)}")
print("goggles: hinge", fmt(h_ue), "cm from the eye (camera axes)")

for m in bpy.data.materials:
    if m.name == "MI_Goggles_V2_DJI": m.name = "MI_Goggles"
    if m.name == "MI_Mount_G24_Wilcox_Tan": m.name = "MI_G24"
for o in mount + fold:  # the black G24 material is unused; every G24 part shows tan
    for s in o.material_slots:
        if s.material and s.material.name.startswith("MI_Mount_G24"):
            s.material = bpy.data.materials["MI_G24"]
M = frame(CAM, hinge)
export(world_copy(mount, "goggles_mount", M), "goggles_mount")
export(world_copy(fold, "goggles_fold", M), "goggles_fold")

gt = os.path.join(GOG, "Textures", "PBR_Goggles_V2_DJI", "Goggles_V2_DJI_")
texture_set("Goggles", load(gt + "BaseColor.png"), load(gt + "DirectX_Normal.png"), load(gt + "AO.png"),
            load(gt + "Roughness.png"), load(gt + "Metalness.png"), 2048, False, 1024)
mt = os.path.join(GOG, "Textures", "PBR_Mount_G24_Wilcox", "Mount_G24_Wilcox_Tan_")
texture_set("G24", load(mt + "BaseColor.png"), load(mt + "DirectX_Normal.png"), load(mt + "AO.png"),
            load(mt + "Roughness.png"), load(mt + "Metalness.png"), 1024, False)

# The gear menu icon: the goggles from the front left, flipped down, on transparency.
for img in bpy.data.images:
    if img.filepath:
        sub = "PBR_Goggles_V2_DJI" if img.name.startswith("Goggles") else "PBR_Mount_G24_Wilcox"
        img.filepath = os.path.join(GOG, "Textures", sub, img.name)
        img.reload()
sc = bpy.context.scene
for o in list(sc.objects):
    o.hide_render = o not in mount + fold
sc.render.engine = 'CYCLES'
sc.cycles.device = 'CPU'
sc.cycles.samples = 48
sc.render.film_transparent = True
sc.render.resolution_x = sc.render.resolution_y = 256
sc.world = sc.world or bpy.data.worlds.new("w")
sc.world.use_nodes = True
sc.world.node_tree.nodes["Background"].inputs[1].default_value = 0.6
pts = [o.matrix_world @ Vector(c) for o in mount + fold for c in o.bound_box]
ctr = sum(pts, Vector()) / len(pts)
size = max((p - ctr).length for p in pts)
cam = bpy.data.objects.new("icon_cam", bpy.data.cameras.new("icon_cam"))
sc.collection.objects.link(cam)
sc.camera = cam
cam.data.type = 'ORTHO'
d = Vector((0.75, -1.0, 0.45)).normalized()
cam.location = ctr + d * size * 4
cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
# Framed to the goggles as the camera sees them, with a small margin (the mesh vertices, not the boxes).
to_cam = cam.rotation_euler.to_matrix().inverted()
seen = [to_cam @ (o.matrix_world @ v.co - ctr) for o in mount + fold for v in o.data.vertices]
half_w = max(abs(p.x) for p in seen)
half_h = max(abs(p.y) for p in seen)
cam.data.ortho_scale = 2 * max(half_w, half_h) * 1.08
sun = bpy.data.objects.new("icon_sun", bpy.data.lights.new("icon_sun", 'SUN'))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(50), 0, math.radians(30))
sc.collection.objects.link(sun)
sc.render.filepath = os.path.join(OUT, "Icon_Goggles.png")
bpy.ops.render.render(write_still=True)
print("WROTE Icon_Goggles")

# ------------------------------------------------------------------ drone

bpy.ops.wm.open_mainfile(filepath=os.path.join(DRN, "Mesh", "DIJI_4k.blend"))
O = bpy.data.objects
body, camera = O["DJI_Mini_4k_Body"], O["DJI_Mini_4k_Camera"]
props = {"FL": O["DJI_Mini_4k_Properller_Front_L"], "FR": O["DJI_Mini_4k_Properller_Front_R"],
         "BL": O["DJI_Mini_4k_Properller_Back_L"], "BR": O["DJI_Mini_4k_Properller_Back_R"]}
release, bomb, decals = O["DJI_Mini_4k_Release"], O["Diji_4k_Bomb"], O["DJI_Mini_4k_Decals"]

def bounds(o):
    w = np.array([tuple(o.matrix_world @ v.co) for v in o.data.vertices])
    return w.min(0), w.max(0)

bmin, bmax = bounds(body)
origin = Vector((bmin + bmax) / 2)
DRONE = Matrix(((-1, 0, 0), (0, 1, 0), (0, 0, 1)))  # Blender -> drone axes: the camera end (-x) forward, y right
def drone_ue(p):
    return DRONE @ (Vector(p) - origin) * 100

parts = [("drone_body", [body], origin), ("drone_camera", [camera], None)] + \
        [("drone_prop_" + k, [o], None) for k, o in props.items()]
pivots = {}
for name, objs, piv in parts:
    if piv is None:
        lo, hi = bounds(objs[0])
        piv = Vector((lo + hi) / 2)
    pivots[name] = piv
    if name != "drone_body":
        manifest.append(f"{name} {fmt(drone_ue(piv))}")
lo, hi = bounds(body)
manifest.append(f"drone.size {fmt([abs(x) for x in DRONE @ Vector(hi - lo) * 100])}")
manifest.append(f"drone.bottom {(lo[2] - origin.z) * 100:.3f}")
rlo, rhi = bounds(release)
blo, bhi = bounds(bomb)
bomb_centre = Vector((blo + bhi) / 2)
manifest.append(f"drone_bomb {fmt(drone_ue(bomb_centre))}")
manifest.append(f"drone.payload_bottom {(min(rlo[2], blo[2]) - origin.z) * 100:.3f}")

# One object holding body, camera and propellers, each face tagged with its part, for one shared layout.
tagged = []
for i, (name, objs, _) in enumerate(parts):
    c = world_copy(objs, name)
    c.data.attributes.new("part", 'INT', 'FACE').data.foreach_set("value", [i] * len(c.data.polygons))
    tagged.append(c)
for o in bpy.context.view_layer.objects:
    o.select_set(o in tagged)
bpy.context.view_layer.objects.active = tagged[0]
bpy.ops.object.join()
allp = bpy.context.view_layer.objects.active
allp.name = "drone_all"
me = allp.data
src_uv = me.uv_layers.active.name
me.uv_layers[src_uv].active_render = True     # the material keeps reading the atlas through this one
packed = me.uv_layers.new(name="Packed")
me.uv_layers.active = packed                  # bakes write through this one
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.uv.smart_project(angle_limit=math.radians(66), island_margin=0.003, area_weight=0.0, correct_aspect=True)
bpy.ops.object.mode_set(mode='OBJECT')

sc = bpy.context.scene
sc.render.engine = 'CYCLES'
sc.cycles.device = 'CPU'
sc.cycles.samples = 4
sc.render.bake.margin = BAKE_MARGIN

mat = me.materials[0]
nodes, links = mat.node_tree.nodes, mat.node_tree.links
out_node = next(n for n in nodes if n.type == 'OUTPUT_MATERIAL')
bsdf = next(n for n in nodes if n.type == 'BSDF_PRINCIPLED')
tex = {n.image.name.replace("DJI_4k_", "").replace(".png", ""): n for n in nodes if n.type == 'TEX_IMAGE' and n.image}
target = nodes.new("ShaderNodeTexImage")
emit = nodes.new("ShaderNodeEmission")

def new_target(name):
    img = bpy.data.images.new(name, DRONE_TEX, DRONE_TEX, alpha=True, float_buffer=True)
    img.colorspace_settings.name = 'Non-Color'
    return img

def pixels(img):
    px = np.array(img.pixels[:], dtype=np.float32).reshape(DRONE_TEX, DRONE_TEX, 4)
    bpy.data.images.remove(img)
    return px

def surface(link_from):
    for l in list(out_node.inputs["Surface"].links):
        links.remove(l)
    links.new(link_from, out_node.inputs["Surface"])

def bake_atlas(kind, source=None):
    img = new_target("bake_" + kind)
    target.image = img
    nodes.active = target
    for o in bpy.context.view_layer.objects:
        o.select_set(o == allp)
    bpy.context.view_layer.objects.active = allp
    sc.render.bake.use_selected_to_active = False
    sc.render.bake.use_clear = True
    if source:
        links.new(tex[source].outputs["Color"], emit.inputs["Color"])
        surface(emit.outputs["Emission"])
        bpy.ops.object.bake(type='EMIT')
    else:
        surface(bsdf.outputs["BSDF"])
        bpy.ops.object.bake(type='NORMAL', normal_space='TANGENT')
    print("BAKED", kind)
    return pixels(img)

color = bake_atlas("color", "BaseColor")
normal = bake_atlas("normal")
ao = bake_atlas("ao", "AO")
rough = bake_atlas("roughness", "Roughness")
metal = bake_atlas("metallic", "Metalness")

# The decals, onto the body from just above it: their colour and their opacity, blended over the atlas.
dcopy = world_copy([decals], "decals")
dmat = dcopy.data.materials[0]
dn, dl = dmat.node_tree.nodes, dmat.node_tree.links
dout = next(n for n in dn if n.type == 'OUTPUT_MATERIAL')
dtex = {n.image.name.replace("DJI_4k_Decal_", "").replace(".png", ""): n for n in dn if n.type == 'TEX_IMAGE' and n.image}
demit = dn.new("ShaderNodeEmission")
def bake_decal(source):
    for l in list(dout.inputs["Surface"].links):
        dl.remove(l)
    dl.new(dtex[source].outputs["Color"], demit.inputs["Color"])
    dl.new(demit.outputs["Emission"], dout.inputs["Surface"])
    img = new_target("decal_" + source)
    img.pixels[:] = np.zeros(DRONE_TEX * DRONE_TEX * 4, dtype=np.float32)
    target.image = img
    nodes.active = target
    for o in bpy.context.view_layer.objects:
        o.select_set(o in (allp, dcopy))
    bpy.context.view_layer.objects.active = allp
    sc.render.bake.use_selected_to_active = True
    sc.render.bake.use_clear = False
    sc.render.bake.cage_extrusion = 0.002
    sc.render.bake.max_ray_distance = 0.004
    sc.render.bake.margin = 0
    bpy.ops.object.bake(type='EMIT')
    sc.render.bake.margin = BAKE_MARGIN
    print("BAKED decal", source)
    return pixels(img)
dcol = bake_decal("BaseColor")
dalpha = bake_decal("Opacity")[..., :1]
print(f"decal coverage: {(dalpha > 0.5).sum()} texels")
color[..., :3] = color[..., :3] * (1 - dalpha) + dcol[..., :3] * dalpha
color[..., :3] = to_srgb(color[..., :3])
texture_set("Drone", color, normal, ao, rough, metal, DRONE_TEX, True, DRONE_TEX // 2)

# Split the baked object back into its parts, each around its own pivot.
for i, (name, _, _) in enumerate(parts):
    me2 = allp.data.copy()
    bm = bmesh.new()
    bm.from_mesh(me2)
    layer = bm.faces.layers.int.get("part")
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if f[layer] != i], context='FACES')
    bm.to_mesh(me2)
    bm.free()
    me2.uv_layers.remove(me2.uv_layers[src_uv])
    me2.attributes.remove(me2.attributes["part"])
    me2.transform(frame(DRONE, pivots[name]))
    me2.materials[0].name = "MI_Drone"
    o = bpy.data.objects.new(name, me2)
    sc.collection.objects.link(o)
    export(o, name)

for m in bpy.data.materials:
    if m.name == "MI_DJI_Bomb": m.name = "MI_Bomb"
export(world_copy([release], "drone_release", frame(DRONE, origin)), "drone_release")
export(world_copy([bomb], "drone_bomb", frame(DRONE, bomb_centre)), "drone_bomb")
bt = os.path.join(DRN, "Textures", "PBR_DJI_4k_Bomb_tx", "DJI_Bomb_")
texture_set("Bomb", load(bt + "BaseColor.png"), load(bt + "Normal.png"), load(bt + "AO.png"),
            load(bt + "Roughness.png"), load(bt + "Metalness.png"), 1024, True)

with open(os.path.join(OUT, "fpv.txt"), "w") as f:
    f.write("\n".join(manifest) + "\n")
print("WROTE fpv.txt")
print("\n".join(manifest))
