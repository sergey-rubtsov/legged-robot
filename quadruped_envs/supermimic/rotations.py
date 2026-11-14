"""Batched, differentiable quaternion ops in [x, y, z, w] order (PyBullet / this repo's
convention -- matches motion_imitation's pose3d.py and pybullet_utils.transformations, which
the existing imitation_task.py reward builds on). Reimplemented in torch so gradients can
flow through them inside SuperMimic's imagined BPTT rollouts; formulas are taken directly
from those two modules so results match the real simulator's reward bit-for-bit (mod float
precision) -- see quadruped_envs/supermimic/mdp.py's validation against imitation_task.py.
"""
import torch


def quat_mul(q1: torch.Tensor, q0: torch.Tensor) -> torch.Tensor:
    """Hamilton product q1 * q0, both [..., 4] in xyzw order."""
    x1, y1, z1, w1 = q1.unbind(-1)
    x0, y0, z0, w0 = q0.unbind(-1)
    return torch.stack([
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
    ], dim=-1)


def quat_conj(q: torch.Tensor) -> torch.Tensor:
    """Conjugate (= inverse for unit quaternions) of q: [..., 4] xyzw."""
    xyz, w = q[..., :3], q[..., 3:4]
    return torch.cat([-xyz, w], dim=-1)


def quat_rotate_point(point: torch.Tensor, quat: torch.Tensor) -> torch.Tensor:
    """Rotate point ([..., 3]) by unit quaternion quat ([..., 4] xyzw): q * p * q^-1,
    using the standard closed-form expansion (equivalent to the full quaternion-multiply
    chain in pose3d.QuaternionRotatePoint but cheaper and friendlier to autograd)."""
    u = quat[..., :3]
    w = quat[..., 3:4]
    uv = torch.cross(u, point, dim=-1)
    uuv = torch.cross(u, uv, dim=-1)
    return point + 2.0 * (w * uv + uuv)


def quat_about_axis(angle: torch.Tensor, axis: torch.Tensor) -> torch.Tensor:
    """Quaternion for a rotation of `angle` (radians, [...]) about `axis` ([..., 3], unit
    length, same leading batch shape as `angle`)."""
    half = angle * 0.5
    xyz = axis * torch.sin(half).unsqueeze(-1)
    w = torch.cos(half).unsqueeze(-1)
    return torch.cat([xyz, w], dim=-1)


def quat_angle(q: torch.Tensor, eps: float = 1e-8) -> torch.Tensor:
    """Rotation angle (radians, unsigned, in [0, 2*pi)) encoded by unit quaternion q.
    Mirrors pose3d.QuaternionToAxisAngle's angle output (axis is not needed by any reward
    term in this codebase, only the magnitude)."""
    axis_norm = torch.linalg.norm(q[..., :3], dim=-1).clamp_min(eps)
    half_angle = torch.atan2(axis_norm, q[..., 3])
    return 2.0 * half_angle


def normalize_rotation_angle(theta: torch.Tensor) -> torch.Tensor:
    """Wrap an angle (radians) to [-pi, pi], matching motion_util.normalize_rotation_angle."""
    return torch.remainder(theta + torch.pi, 2 * torch.pi) - torch.pi


def calc_heading(q: torch.Tensor) -> torch.Tensor:
    """Heading (yaw) angle of quaternion q about the world z-axis, matching
    motion_util.calc_heading: the direction the local +x axis points in, projected onto
    the world xy-plane."""
    ref_dir = torch.zeros_like(q[..., :3])
    ref_dir[..., 0] = 1.0
    rot_dir = quat_rotate_point(ref_dir, q)
    return torch.atan2(rot_dir[..., 1], rot_dir[..., 0])


def calc_heading_rot(q: torch.Tensor) -> torch.Tensor:
    """Quaternion representing only the heading (yaw) component of q."""
    heading = calc_heading(q)
    z_axis = torch.zeros_like(q[..., :3])
    z_axis[..., 2] = 1.0
    return quat_about_axis(heading, z_axis)


def quat_slerp(q0: torch.Tensor, q1: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
    """Spherical interpolation between unit quaternions q0, q1 ([..., 4] xyzw) at t in [0, 1]
    (shape [...], broadcasting against q0/q1's batch dims), matching
    pybullet_utils.transformations.quaternion_slerp's shortest-path behavior."""
    dot = (q0 * q1).sum(-1, keepdim=True)
    q1 = torch.where(dot < 0, -q1, q1)
    dot = dot.abs().clamp(max=1.0)
    theta = torch.acos(dot)
    s = torch.sin(theta)
    t = t.unsqueeze(-1)
    w0 = torch.where(s < 1e-6, 1 - t, torch.sin((1 - t) * theta) / (s + 1e-12))
    w1 = torch.where(s < 1e-6, t, torch.sin(t * theta) / (s + 1e-12))
    return torch.nn.functional.normalize(w0 * q0 + w1 * q1, dim=-1)


# ---------------------------------------------------------------------------------------
# Matrix-based ops for the world model (SuperMimic's S = (p, R, v, w, q, qd) uses a rotation
# MATRIX for R, not a quaternion: so3_exp/so3_log give smooth, singularity-free small-angle
# integration for a network predicting rotation *increments*, and matrices compose without
# renormalization. These are ported from supermimic_vibed/supermimic/rotations.py, which uses
# wxyz quaternions -- adapted to this project's xyzw convention (only quat_to_mat/mat_to_quat
# care about component order; the rest is convention-agnostic matrix algebra).
# ---------------------------------------------------------------------------------------

def quat_to_mat(q: torch.Tensor) -> torch.Tensor:
    """xyzw unit quaternion [..., 4] -> rotation matrix [..., 3, 3]."""
    q = torch.nn.functional.normalize(q, dim=-1)
    x, y, z, w = q.unbind(-1)
    return torch.stack((
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
    ), dim=-1).reshape(q.shape[:-1] + (3, 3))  # fmt: skip


def mat_to_quat(m: torch.Tensor) -> torch.Tensor:
    """Robust rotation matrix [..., 3, 3] -> xyzw unit quaternion [..., 4] (w >= 0)."""
    m00, m01, m02 = m[..., 0, 0], m[..., 0, 1], m[..., 0, 2]
    m10, m11, m12 = m[..., 1, 0], m[..., 1, 1], m[..., 1, 2]
    m20, m21, m22 = m[..., 2, 0], m[..., 2, 1], m[..., 2, 2]
    qw = torch.sqrt(torch.clamp(1 + m00 + m11 + m22, min=1e-10)) / 2
    qx = torch.sqrt(torch.clamp(1 + m00 - m11 - m22, min=1e-10)) / 2
    qy = torch.sqrt(torch.clamp(1 - m00 + m11 - m22, min=1e-10)) / 2
    qz = torch.sqrt(torch.clamp(1 - m00 - m11 + m22, min=1e-10)) / 2
    cands = torch.stack((
        torch.stack(((m21 - m12) / (4 * qw), (m02 - m20) / (4 * qw), (m10 - m01) / (4 * qw), qw), -1),
        torch.stack((qx, (m01 + m10) / (4 * qx), (m02 + m20) / (4 * qx), (m21 - m12) / (4 * qx)), -1),
        torch.stack(((m01 + m10) / (4 * qy), qy, (m12 + m21) / (4 * qy), (m02 - m20) / (4 * qy)), -1),
        torch.stack(((m02 + m20) / (4 * qz), (m12 + m21) / (4 * qz), qz, (m10 - m01) / (4 * qz)), -1),
    ), dim=-2)  # fmt: skip
    idx = torch.stack((qw, qx, qy, qz), -1).argmax(-1)
    q = torch.gather(cands, -2, idx[..., None, None].expand(idx.shape + (1, 4))).squeeze(-2)
    q = torch.nn.functional.normalize(q, dim=-1)
    return q * torch.where(q[..., 3:4] < 0, -1.0, 1.0)


def skew(v: torch.Tensor) -> torch.Tensor:
    """[..., 3] -> skew-symmetric cross-product matrix [..., 3, 3]."""
    x, y, z = v.unbind(-1)
    o = torch.zeros_like(x)
    return torch.stack((o, -z, y, z, o, -x, -y, x, o), -1).reshape(v.shape[:-1] + (3, 3))


def so3_exp(v: torch.Tensor) -> torch.Tensor:
    """Rotation vector [..., 3] -> matrix [..., 3, 3] (Rodrigues, smooth at zero)."""
    theta2 = (v * v).sum(-1, keepdim=True)
    theta = torch.sqrt(theta2 + 1e-12)
    small = theta2 < 1e-8
    a = torch.where(small, 1 - theta2 / 6, torch.sin(theta) / theta)
    b = torch.where(small, 0.5 - theta2 / 24, (1 - torch.cos(theta)) / (theta2 + 1e-12))
    K = skew(v)
    eye = torch.eye(3, device=v.device, dtype=v.dtype).expand(K.shape)
    return eye + a[..., None] * K + b[..., None] * (K @ K)


def quat_to_rotvec(q: torch.Tensor) -> torch.Tensor:
    """xyzw unit quaternion -> rotation vector [..., 3], stable near angle 0 and pi."""
    q = q * torch.where(q[..., 3:4] < 0, -1.0, 1.0)
    v = q[..., :3]
    sin_half = torch.linalg.norm(v, dim=-1, keepdim=True)
    half = torch.atan2(sin_half, q[..., 3:4])
    scale = torch.where(sin_half < 1e-6, 2.0 / torch.clamp(q[..., 3:4], min=1e-6), 2 * half / (sin_half + 1e-12))
    return v * scale


def so3_log(R: torch.Tensor) -> torch.Tensor:
    """Rotation matrix [..., 3, 3] -> rotation vector [..., 3], via the quaternion."""
    return quat_to_rotvec(mat_to_quat(R))


def rot_angle_sq(Ra: torch.Tensor, Rb: torch.Tensor) -> torch.Tensor:
    """Squared geodesic angle between two rotation matrices, differentiable (acos of trace)."""
    tr = (Ra * Rb).sum((-1, -2))  # trace(Ra^T Rb)
    cos = torch.clamp((tr - 1) / 2, -1 + 1e-6, 1 - 1e-6)
    return torch.acos(cos) ** 2


def yaw_mat(R: torch.Tensor) -> torch.Tensor:
    """Heading-only (yaw about world z) rotation matrix extracted from R, from its
    projected local-x axis -- the matrix analog of calc_heading_rot."""
    fx, fy = R[..., 0, 0], R[..., 1, 0]
    n = torch.sqrt(fx * fx + fy * fy + 1e-9)
    c, s = fx / n, fy / n
    o, one = torch.zeros_like(c), torch.ones_like(c)
    return torch.stack((c, -s, o, s, c, o, o, o, one), -1).reshape(R.shape[:-2] + (3, 3))


def mat_to_6d(R: torch.Tensor) -> torch.Tensor:
    """First two columns of R, flattened: the 6D continuous rotation representation used
    for network inputs (avoids quaternion double-cover / Euler-angle discontinuities)."""
    return R[..., :, :2].reshape(R.shape[:-2] + (6,))
