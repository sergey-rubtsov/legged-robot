# A script to create a hierarchical armature and add constraints to animate.
# Firstly import the c3d with a prefix, it is chosen "GT_", this makes it easier to clean up.

import bpy
context = bpy.context
scene = context.scene
prefix = "GT_"
# select all imported with prefix GT_
obs = [o for o in scene.objects if o.name.startswith(prefix)]
root = "RootDog"

# clean up unwanted objects could look for flat action sum.

text = bpy.data.texts.get("C3D_bones")
if not text:
    text = bpy.data.texts.new("C3D_bones")

text.clear()

bones = {}
for o in obs:
    if o.location.length < 0.0000001:
        scene.objects.unlink(o)
        bpy.data.objects.remove(o)
    else:
        name = o.name[3:]
        print(bones)
        bones[name] = {"parent":None,
                       "tail":root}

names = sorted(bones.keys())
text.write("%s\n" % root)
for name in names:
    if name == root:
        continue
    text.write("\t%s\n" % name)