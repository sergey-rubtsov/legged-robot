import numpy as np

URDF_FILENAME = "../quadruped_envs/urdf/od.urdf"

REF_POS_SCALE = 0.825
INIT_POS = np.array([0, 0, 0.26])
INIT_ROT = np.array([0, 0, 0, 1.0])

SIM_TOE_JOINT_IDS = [
    5,  # right hand
    11,  # right foot
    2,  # left hand
    8,  # left foot
]
SIM_HIP_JOINT_IDS = [3, 9, 0, 6]
SIM_ROOT_OFFSET = np.array([0, 0, 0.0])
SIM_TOE_OFFSET_LOCAL = [
    np.array([-0.02, 0.0, 0.0]),
    np.array([-0.02, 0.0, 0.01]),
    np.array([-0.02, 0.0, 0.0]),
    np.array([-0.02, 0.0, 0.01])
]

DEFAULT_JOINT_POSE = np.array([0, 0.8, 1.25,
                               0, 0.8, 1.25,
                               0, 0.8, 1.25,
                               0, 0.8, 1.25])

JOINT_DAMPING = [0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01]

FORWARD_DIR_OFFSET = np.array([0, 0, 0.025])
