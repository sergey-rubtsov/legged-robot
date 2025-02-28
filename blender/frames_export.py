# root object should be selected

import bpy
import numpy as np

FRAME_DURATION = 0.01667
LABEL = "Export Frames v0.7"

def get_last_frame_number():
    return bpy.context.scene.frame_end


def get_pose_frames(od_armature_name='root', frame=1):
    bone_list = [
        'hip_a', 'u_leg_a', 'l_leg_a',
        'hip_b', 'u_leg_b', 'l_leg_b',
        'hip_c', 'u_leg_c', 'l_leg_c',
        'hip_d', 'u_leg_d', 'l_leg_d'
    ]
    ob = bpy.context.object
    bpy.context.scene.frame_set(frame)
    od_armature = bpy.context.scene.objects[od_armature_name]
    b = od_armature.pose.bones
    w = ob.convert_space(
        pose_bone=b['root'],
        matrix=b['root'].matrix,
        from_space='POSE',
        to_space='WORLD'
    )
    result = []
    result.extend(w.translation)
    mw = ob.convert_space(
        pose_bone=b['root'],
        matrix=b['root'].matrix,
        from_space='POSE',
        to_space='LOCAL'
    )
    result.append(mw.to_quaternion().x)
    result.append(mw.to_quaternion().y)
    result.append(mw.to_quaternion().z)
    result.append(mw.to_quaternion().w)

    for name in bone_list:
        result.append(ob.convert_space(
            pose_bone=b[name],
            matrix=b[name].matrix,
            from_space='POSE',
            to_space='LOCAL',
        ).to_euler().y)

    return result


def write_frames(context, filepath, frames, frame_duration=FRAME_DURATION):
    f = open(filepath, 'w', encoding='utf-8')
    f.write("{\n")
    f.write("\"LoopMode\": \"Wrap\",\n")
    f.write("\"FrameDuration\": " + str(frame_duration) + ",\n")
    f.write("\"EnableCycleOffsetPosition\": true,\n")
    f.write("\"EnableCycleOffsetRotation\": true,\n")
    f.write("\n")

    f.write("\"Frames\":\n")

    f.write("[")
    for i in range(frames.shape[0]):
        curr_frame = frames[i]

        if i != 0:
            f.write(",")
        f.write("\n  [")

        for j in range(frames.shape[1]):
            curr_val = curr_frame[j]
            if j != 0:
                f.write(", ")
            f.write("%.5f" % curr_val)

        f.write("]")

    f.write("\n]")
    f.write("\n}")
    f.close()

    return {'FINISHED'}

from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class ExportFrames(Operator, ExportHelper):
    bl_idname = "export_frames.frames_data"
    bl_label = LABEL

    first_frame = bpy.props.IntProperty(name="First frame", default=0)
    last_frame = bpy.props.IntProperty(name="Last frame", default=get_last_frame_number())
    frame_duration = bpy.props.FloatProperty(name="Frame duration", default=FRAME_DURATION)

    filename_ext = ".txt"

    filter_glob = StringProperty(
        default="*.txt",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    def execute(self, context):
        frames = np.zeros([self.last_frame - self.first_frame, 19])
        for i in range(self.first_frame, self.last_frame):
            frames[i - self.first_frame] = np.asarray(get_pose_frames(frame=i))
        return write_frames(context, self.filepath, frames, self.frame_duration)


def menu_func_export(self, context):
    self.layout.operator(ExportFrames.bl_idname, text=LABEL)


def register():
    bpy.utils.register_class(ExportFrames)
    bpy.types.INFO_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(ExportFrames)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)


def print(data):
    for window in bpy.context.window_manager.windows:
        screen = window.screen
        for area in screen.areas:
            if area.type == 'CONSOLE':
                override = {'window': window, 'screen': screen, 'area': area}
                bpy.ops.console.scrollback_append(override, text=str(data), type="OUTPUT")


if __name__ == "__main__":
    register()
