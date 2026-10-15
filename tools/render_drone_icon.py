# render_drone_icon.py - the drone's tactical-menu icon, from what prepare_assets.py wrote into Assets\.
#   blender.exe --background --python tools\render_drone_icon.py
# Writes Assets\Icon_Drone.png (256 x 256, transparent): the drone with its payload, from the front left,
# above, lit like the goggles' icon.
import bpy, math, os
from mathutils import Vector

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, "Assets")

# Part pivots, UE cm (fpv.txt). The OBJs come back into Blender as UE = (x, -y, z) * 100.
pivots = {}
with open(os.path.join(ASSETS, "fpv.txt")) as f:
    for line in f:
        k, *v = line.split()
        if len(v) == 3:
            pivots[k] = Vector((float(v[0]) / 100, -float(v[1]) / 100, float(v[2]) / 100))

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene

def material(name, tex):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    img = nt.nodes.new("ShaderNodeTexImage")
    img.image = bpy.data.images.load(os.path.join(ASSETS, tex + "_Diffuse.png"))
    nt.links.new(img.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.55
    return m

mats = {"MI_Drone": material("drone", "Drone"), "MI_Bomb": material("bomb", "Bomb")}
parts = []
for name, pivot in [("drone_body", None), ("drone_release", None), ("drone_camera", "drone_camera"), ("drone_bomb", "drone_bomb"),
                    ("drone_prop_FL", "drone_prop_FL"), ("drone_prop_FR", "drone_prop_FR"), ("drone_prop_BL", "drone_prop_BL"),
                    ("drone_prop_BR", "drone_prop_BR")]:
    before = set(sc.objects)
    bpy.ops.wm.obj_import(filepath=os.path.join(ASSETS, name + ".obj"), forward_axis='NEGATIVE_Z', up_axis='Y')
    for o in set(sc.objects) - before:
        if pivot:
            o.location = pivots[pivot]
        for slot in o.material_slots:
            slot.material = mats["MI_Bomb" if "bomb" in name or "release" in name else "MI_Drone"]
        parts.append(o)
bpy.context.view_layer.update()

sc.render.engine = 'CYCLES'
sc.cycles.device = 'CPU'
sc.cycles.samples = 48
sc.render.film_transparent = True
sc.render.resolution_x = sc.render.resolution_y = 256
sc.world = bpy.data.worlds.new("w")
sc.world.use_nodes = True
sc.world.node_tree.nodes["Background"].inputs[1].default_value = 1.4  # the body is dark grey: a brighter sky reads better as a tile
pts = [o.matrix_world @ v.co for o in parts for v in o.data.vertices]
ctr = sum(pts, Vector()) / len(pts)
size = max((p - ctr).length for p in pts)
cam = bpy.data.objects.new("icon_cam", bpy.data.cameras.new("icon_cam"))
sc.collection.objects.link(cam)
sc.camera = cam
cam.data.type = 'ORTHO'
d = Vector((0.9, 0.75, 0.6)).normalized()  # in front (UE +X) and to its left (UE -Y), above
cam.location = ctr + d * size * 4
cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
to_cam = cam.rotation_euler.to_matrix().inverted()
seen = [to_cam @ (p - ctr) for p in pts]
cam.data.ortho_scale = 2 * max(max(abs(p.x) for p in seen), max(abs(p.y) for p in seen)) * 1.08
sun = bpy.data.objects.new("icon_sun", bpy.data.lights.new("icon_sun", 'SUN'))
sun.data.energy = 4.5
sun.rotation_euler = (math.radians(50), 0, math.radians(30))
sc.collection.objects.link(sun)
sc.render.filepath = os.path.join(ASSETS, "Icon_Drone.png")
bpy.ops.render.render(write_still=True)
print("WROTE Icon_Drone")
