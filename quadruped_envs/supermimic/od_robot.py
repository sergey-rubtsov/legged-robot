"""Differentiable (pure-PyTorch) kinematics for the OD quadruped.

Geometry constants below are read directly from quadruped_envs/urdf/od.urdf (hip joint
origins, and the link_l_leg_* joint origins relative to their parent link_u_leg_* link).
Joint ordering matches what PyBullet actually assigns to this URDF -- verified empirically
with p.getJointInfo(), NOT the raw <joint> declaration order in the file, which differs:
PyBullet orders joints depth-first per leg, giving index layout

    [hip_a, u_leg_a, l_leg_a, hip_b, u_leg_b, l_leg_b, hip_c, u_leg_c, l_leg_c,
     hip_d, u_leg_d, l_leg_d]

i.e. for leg i in [0=a, 1=b, 2=c, 3=d], joint_pos[3*i : 3*i+3] = (hip/abduction about the
base x-axis, upper-leg/hip-pitch about y, knee about y). This is also the order used by
od_imitation_env's action space (ACTION_CONFIG) and by the reference motion files in
quadruped_envs/data/motions/od/*.txt (verified against imitation_task.py, which reads
joint state straight from pybullet.getJointStateMultiDof in this same index order).

The knee angle only reorients the lower-leg link, not its position (its own joint origin
is the translation; URDF joint rotation is applied about a point coincident with the
child link's origin), so it does not appear in the position FK below. link_l_leg_*'s
inertial origin is exactly (0, 0, 0) in od.urdf, so this joint-origin FK is exact (not an
approximation) for the point PyBullet's getLinkState(...)[0] (link CoM) reports for the
reward computation in imitation_task.py -- see quadruped_envs/supermimic/mdp.py.
"""
import torch

NUM_LEGS = 4
NUM_JOINTS = 12
LEG_NAMES = ("a", "b", "c", "d")  # front-left, front-right, back-left, back-right

# Hip joint origin in the robot base frame, per leg (from od.urdf <joint name="link_hip_*">).
HIP_OFFSETS = torch.tensor([
    [0.1939, 0.09, 0.0],  # a: front-left
    [0.1939, -0.09, 0.0],  # b: front-right
    [-0.1939, 0.09, 0.0],  # c: back-left
    [-0.1939, -0.09, 0.0],  # d: back-right
])

# link_l_leg_* joint origin in the link_u_leg_* (upper-leg) local frame, per leg.
LOWER_LEG_OFFSETS = torch.tensor([
    [-0.1004, 0.05695, -0.124],  # a
    [-0.1004, -0.05695, -0.124],  # b
    [-0.09902, 0.05695, -0.12527],  # c
    [-0.09902, -0.05695, -0.12527],  # d
])


def _rot_x(theta: torch.Tensor) -> torch.Tensor:
    """Batched rotation matrices about the x-axis. theta: [...] -> [..., 3, 3]."""
    c, s = torch.cos(theta), torch.sin(theta)
    one = torch.ones_like(theta)
    zero = torch.zeros_like(theta)
    row0 = torch.stack([one, zero, zero], dim=-1)
    row1 = torch.stack([zero, c, -s], dim=-1)
    row2 = torch.stack([zero, s, c], dim=-1)
    return torch.stack([row0, row1, row2], dim=-2)


def _rot_y(theta: torch.Tensor) -> torch.Tensor:
    """Batched rotation matrices about the y-axis. theta: [...] -> [..., 3, 3]."""
    c, s = torch.cos(theta), torch.sin(theta)
    one = torch.ones_like(theta)
    zero = torch.zeros_like(theta)
    row0 = torch.stack([c, zero, s], dim=-1)
    row1 = torch.stack([zero, one, zero], dim=-1)
    row2 = torch.stack([-s, zero, c], dim=-1)
    return torch.stack([row0, row1, row2], dim=-2)


def foot_positions_base_frame(joint_pos: torch.Tensor) -> torch.Tensor:
    """Forward kinematics of the 4 foot (lower-leg link) positions in the robot base frame.

    Args:
      joint_pos: [..., 12] joint angles in PyBullet's native order for this URDF (see
        module docstring): [hip_a, upper_a, knee_a, hip_b, upper_b, knee_b, ...].

    Returns:
      [..., 4, 3] foot positions in the base/root frame, one row per leg in LEG_NAMES order.
    """
    batch_shape = joint_pos.shape[:-1]
    legs = joint_pos.reshape(*batch_shape, NUM_LEGS, 3)
    theta_ab = legs[..., 0]
    theta_hip = legs[..., 1]
    # theta_knee = legs[..., 2]  # does not affect foot position, only orientation.

    hip_offsets = HIP_OFFSETS.to(joint_pos.device, joint_pos.dtype)
    lower_leg_offsets = LOWER_LEG_OFFSETS.to(joint_pos.device, joint_pos.dtype)

    r_hip = _rot_x(theta_ab)  # [..., 4, 3, 3]
    r_upper = _rot_y(theta_hip)  # [..., 4, 3, 3]

    lower_offset_in_hip_frame = torch.einsum("...ij,...j->...i", r_upper, lower_leg_offsets)
    lower_offset_in_base_frame = torch.einsum("...ij,...j->...i", r_hip, lower_offset_in_hip_frame)

    return hip_offsets + lower_offset_in_base_frame
