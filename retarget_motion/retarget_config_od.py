import numpy as np

URDF_FILENAME = "../quadruped_envs/urdf/od.urdf"

REF_POS_SCALE = 0.6
INIT_POS = np.array([0, 0, 0])
INIT_ROT = np.array([0, 0, 1, 0])

# a - front left
# b - front right
# c - back left
# d - back right
#
#         a   b   c   d
# hips    0   3   6   9
# uleg    1   4   7   10
# lleg    2   5   8   11

SIM_LOW_LEG_JOINT_IDS = [
    5,   # b right hand
    12,  # d right foot
    2,   # a left hand
    8    # c left foot
]
SIM_UP_LEG_JOINT_IDS = [
    4,   # b right arm
    10,  # d right knee
    1,   # a left arm
    7    # c left knee
]
SIM_HIP_JOINT_IDS = [
    3,  # b
    9,  # d
    0,  # a
    6   # c
]

SIM_ROOT_OFFSET = np.array([0, 0, -0.005])
SIM_FOOT_OFFSET = [
    np.array([0.0, -0.1, 0.0]),
    np.array([0.0, -0.1, 0.0]),
    np.array([0.0, 0.1, 0.0]),
    np.array([0.0, 0.1, 0.0])
]
SIM_LOW_LEG_OFFSET_LOCAL = [
    np.array([0.0, 0, 0.0]),
    np.array([0.0, 0, 0.0]),
    np.array([0.0, 0, 0.0]),
    np.array([0.0, 0, 0.0])
]

SIM_LOW_LEG_ROT = [
    np.array([0.0, 0, 0.0]),
    np.array([0.0, 0, 0.0]),
    np.array([0.0, 0, 0.0]),
    np.array([0.0, 0, 0.0])
]

DEFAULT_JOINT_POSE = np.array([0, 0, 1.25,
                               0, 0, 1.25,
                               0, 0, 1.25,
                               0, 0, 1.25])

JOINT_DAMPING = [0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01
                 ]

FORWARD_DIR_OFFSET = np.array([0, 0, 0.0])
