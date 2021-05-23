import pybullet as p
from bvhtoolbox import BvhTree
from bvhtoolbox.bvhtransforms import get_affines,\
                           get_euler_angles, \
                           get_quaternions,\
                           get_translations, \
                           get_rotation_matrices,\
                           get_motion_data, \
                           set_motion_data,\
                           prune

#           'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
#     3     'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
#     4     'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
#     5     'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
#     6     'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
#     7     'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
#     8     'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
#     9     'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)

# Convert quaternion [x,y,z,w] to Euler [roll, pitch, yaw]
# Get the wxyz quaternion representations of a joint for all frames.
if __name__ == '__main__':
    # Testing
    bvh_filepath = "rumba.bvh"
    with open(bvh_filepath) as file_handle:
        mocap = BvhTree(file_handle.read())
    bvh_q = get_quaternions(mocap, 'mixamorig:Neck')[0]
    print(p.getEulerFromQuaternion([bvh_q[1], bvh_q[2], bvh_q[3], bvh_q[0]]))
    # for axes in ([  'sxyz', 'sxzx', 'syxz', 'szxz', 'rzyx', 'rxzx', 'rzxy', 'rzxz',
    #                 'sxyx', 'syzx', 'syxy', 'szyx', 'rxyx', 'rxzy', 'ryxy', 'rxyz',
    #                 'sxzy', 'syzy', 'szxy', 'szyz', 'ryzx', 'ryzy', 'ryxz', 'rzyz']):
    #     print(p.getEulerFromQuaternion(get_quaternions(mocap, 'mixamorig:Hips', axes)[0]))
    # mat_hips = get_affines(mocap, 'mixamorig:Hips')
    # mat_hips_rots = get_rotation_matrices(mocap, 'mixamorig:Hips')
    # mat_hips_trans = get_translations(mocap, 'mixamorig:Hips')
    # euler = get_euler_angles(mocap, 'mixamorig:Hips')
    # quater = get_quaternions(mocap, 'mixamorig:Hips')
    # motion = get_motion_data(mocap)

    # (3.0921082583325017, -0.019445393874877107, -0.020307202552926864)
    # (3.092117598379723, 0.000961790429071288, -0.0008856058515594315)
    # (3.092503162499718, -0.01944138403756884, -0.020311041415677706)
    # (3.0726730507130666, -0.0010044107068085846, -0.02028235126898117)
    # (3.0921082583325017, -0.019445393874877107, -0.020307202552926864)
    # (3.0921184445879435, -0.0010044107068085846, -0.0008369573941040685)
    # (3.092503162499718, -0.01944138403756884, -0.020311041415677706)
    # (3.0726668727793793, -0.0003948295391099242, -0.020303364415336516)
    # (3.1406306817394065, -0.01942158771735355, 0.02918653450906648)
    # (3.092512873452837, -0.020445804474120057, -0.02028658167981726)
    # (-3.1411968843198523, -0.06892577542945737, -0.020330621526597457)
    # (3.092503050371013, -0.020422192679494806, -0.019324590651007688)
    # (-3.141197749422577, -0.01944138403756884, 0.029173353841613965)
    # (3.092512873452837, -0.020445804474120057, -0.02028658167981726)
    # (-3.140585852723319, -0.06891956805712621, -0.020351684654866163)
    # (3.092503050371013, -0.020422192679494806, -0.019324590651007688)
    # (3.0920989165278, -0.019421587717353552, -0.01934523070253996)
    # (3.092117603938921, 0.0008369569719255935, -0.0010044110586021353)
    # (3.0921081384826095, -0.020426007757490143, -0.019320558027231677)
    # (3.1124061190807266, -0.01942158771735355, 0.0009619718503869023)
    # (3.0920989165278, -0.019421587717353552, -0.01934523070253996)
    # (3.0921184501470464, 0.0008856054419485703, 0.0009617908062362009)
    # (3.0921081384826095, -0.020426007757490143, -0.019320558027231677)
    # (3.112419299748179, -0.01944138403756884, -0.0003949041672164668)

