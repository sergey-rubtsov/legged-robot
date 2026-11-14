"""Differentiable reward/termination for the OD quadruped imitation task.

Ports quadruped_envs/imitation_wrapper_env/imitation_task.py's DeepMimic-style reward and
imitation_terminal_conditions.py's drift/rotation check into pure, batched PyTorch tensor
ops over an explicit state tuple, so SuperMimic's training loop can backpropagate through
them across imagined world-model rollouts. Weights/scales are copied verbatim from
ImitationTask's defaults; see validate_against_sim.py for a step-by-step numerical check
against the real PyBullet-computed reward.

Deliberately NOT ported here (these need the real simulator or the motion library, not
just the robot's own state, so they stay at the env/collection level -- mirroring how
SuperMimic's own mdp.py only state-bases the termination conditions it can):
  - non-foot ground contact termination
  - "motion clip is over" termination
  - the observation's K-future-reference-frames goal construction (quadruped_envs/supermimic/motion.py)
"""
from collections import namedtuple
from dataclasses import dataclass

import torch

from quadruped_envs.supermimic import od_robot, rotations as rot

NUM_JOINTS = 12
NUM_LEGS = 4

# Reward weights/scales, copied from ImitationTask's constructor defaults.
POSE_WEIGHT = 0.35
VELOCITY_WEIGHT = 0.15
END_EFFECTOR_WEIGHT = 0.2
ROOT_POSE_WEIGHT = 0.2
ROOT_VELOCITY_WEIGHT = 0.05

POSE_ERR_SCALE = 5.0
VELOCITY_ERR_SCALE = 0.1
END_EFFECTOR_ERR_SCALE = 40.0
END_EFFECTOR_HEIGHT_ERR_SCALE = 3.0
ROOT_POSE_ERR_SCALE = 20.0
ROOT_VELOCITY_ERR_SCALE = 2.0

# Radians of joint-position-target range per unit of normalized action [-1,1], used ONLY when
# training with position (PD) control instead of this project's original direct-torque
# convention -- see train.py's --control_mode and env.py's worker for where a normalized action
# actually gets converted via `target_q = default_q + action * ACTION_POS_SCALE` before being
# sent to the real env (SimpleNormalizer's own get_action is a pass-through, it does NOT do this
# conversion). Formula ported from blinkmimic (a more mature SuperMimic follow-up project)'s own
# convention (its own comment: "the action scale is 0.25 * effort_limit / stiffness, as in
# BeyondMimic"), substituted with THIS project's own effort limit (od_motor.py's
# DEFAULT_TORQUE_LIMIT=25 Nm) and PD stiffness (od.py's K_P, which is itself a RAW gain that
# gets multiplied by TORQUE_MULTIPLIER before becoming an effective Nm/rad stiffness -- see
# od.py's own module note on why). An established convention, not an arbitrary guess, but still
# an UNVALIDATED starting point for this specific robot -- same "needs its own tuning pass"
# status as every other hand-calibrated constant in this file.
from quadruped_envs import od as _od
from quadruped_envs.imitation_wrapper_env import od_motor as _od_motor
ACTION_POS_SCALE = 0.25 * _od_motor.DEFAULT_TORQUE_LIMIT / (_od.K_P * _od_motor.TORQUE_MULTIPLIER)
del _od, _od_motor

# Recovery-specific commandable target range -- ACTION_POS_SCALE above is calibrated for ORDINARY
# gait tracking, where joint excursions from a nominal standing pose are genuinely small (+-18
# degrees); re-enabling recovery training (2026-11-10) surfaced that this band makes it
# IMPOSSIBLE for the policy to command, e.g., a fully extended push-off leg to actually right
# itself from lying on its back -- confirmed via quadruped_envs/urdf/od.urdf, not assumed: hip
# joints (indices 0, 3, 6, 9 -- MOTOR_NAMES order in od.py) are `revolute` with a real mechanical
# limit (-1.31, 2.443 rad); every other joint (upper/lower leg) is `continuous` -- no mechanical
# limit in the URDF at all, free to spin. "Use everything the robot can physically do" (the
# user's own framing) means giving recovery a target range matching this directly -- the real
# hip limits, and a full +-pi rotation for the continuous joints (not literally unbounded, just
# because a target more than one full turn away from the joint's current angle is meaningless --
# PD position control doesn't know how many times a continuous joint has already spun around).
import math as _math
RECOVERY_Q_LOWER = torch.tensor([-1.31, -_math.pi, -_math.pi] * 4)
RECOVERY_Q_UPPER = torch.tensor([2.443, _math.pi, _math.pi] * 4)
RECOVERY_Q_CENTER = (RECOVERY_Q_LOWER + RECOVERY_Q_UPPER) / 2
RECOVERY_Q_HALF_RANGE = (RECOVERY_Q_UPPER - RECOVERY_Q_LOWER) / 2
del _math

# Theoretical per-step reward ceiling (each of compute_reward's 5 terms is an exp(-err) in (0,1],
# weighted sum maxes out here at zero tracking error) -- used by env.py's reset-bin sampling as
# G_max (= MAX_STEP_REWARD/(1-gamma)) to normalize the softmax over per-bin return estimates.
# Ported from blinkmimic (a more mature SuperMimic follow-up project)'s own MAX_STEP_REWARD
# pattern (their mdp.py), which uses an exact ceiling derived from ITS reward weights rather
# than an assumed round number -- same idea applied to this project's own 5-term reward.
MAX_STEP_REWARD = POSE_WEIGHT + VELOCITY_WEIGHT + END_EFFECTOR_WEIGHT + ROOT_POSE_WEIGHT + ROOT_VELOCITY_WEIGHT

# Action-rate penalty weight (not in ImitationTask's original reward -- added 2026-11-07 after
# observing the deployed policy command physically-impossible torque swings: od_motor.py's
# TORQUE-mode actuator model is a pure instantaneous `torque = action * 25 Nm` with no motor
# inductance/current-slew-rate or reflected-rotor-inertia modeled, and nothing in the original
# reward discouraged exploiting that -- measured directly on a live checkpoint: mean per-tick
# action swing on gallop alone implies an 18.98 Nm/5ms average torque change, p95 hitting the
# full 50 Nm swing (-25->+25 Nm in one 5ms tick), and root angular speed up to 58.5 rad/s during
# recovery -- no real actuator of this size could do either. This penalty is the SOFT fix (makes
# smoothness part of what the policy is rewarded for); a HARD fix (an actual torque-slew-rate
# limit or added armature inertia in od_motor.py, which would also change what the world model
# learns, not just the reward target) is a separate, bigger change not made here. 0.05 is a
# conservative starting point (quadratic in actionspace delta, so it's a small tax on the
# moderate jerk already needed for fast gaits like gallop but a real tax on p95-style
# full-range spikes) -- not yet empirically calibrated, same "start conservative, validate
# before trusting" pattern as this file's other scale constants.
ACTION_RATE_WEIGHT = 0.05

# Termination thresholds, copied from imitation_terminal_condition's defaults.
DIST_FAIL_THRESHOLD = 0.02
ROT_FAIL_THRESHOLD = 0.5 * torch.pi


@dataclass
class State:
    """One (possibly batched) robot state. All tensors share leading batch dims.

    root_pos, root_lin_vel, root_ang_vel: [..., 3]
    root_rot: [..., 4], xyzw
    joint_pos, joint_vel: [..., 12], PyBullet's native per-leg order -- see od_robot.py.
    """
    root_pos: torch.Tensor
    root_rot: torch.Tensor
    root_lin_vel: torch.Tensor
    root_ang_vel: torch.Tensor
    joint_pos: torch.Tensor
    joint_vel: torch.Tensor


# (p, R, v, w, q, qd): root position / rotation MATRIX / world-frame linear & angular
# velocity, joint pos/vel -- the state convention supermimic_vibed's WorldModel/Policy/Critic
# use (R as a matrix, not a quaternion: so3_exp/so3_log give smooth small-angle integration
# for a network predicting rotation increments, and matrices compose without renormalizing).
# `State` above (xyzw quaternion) stays the boundary representation for reward/termination
# and real-sim I/O; convert between the two with mat_state_from/mat_state_to.
MatState = namedtuple("MatState", "p R v w q qd")


def mat_state_from(s: State) -> MatState:
    return MatState(s.root_pos, rot.quat_to_mat(s.root_rot), s.root_lin_vel, s.root_ang_vel, s.joint_pos, s.joint_vel)


def mat_state_to(s: MatState) -> State:
    return State(s.p, rot.mat_to_quat(s.R), s.v, s.w, s.q, s.qd)


STATE_DIM = 3 + 4 + 3 + 3 + NUM_JOINTS + NUM_JOINTS  # p, quat(xyzw), v, w, q, qd


def flatten_state(S: MatState) -> torch.Tensor:
    """MatState -> flat tensor for replay-buffer storage (quaternion, not the full 3x3
    matrix, both for compactness and so stored rotations stay exactly unit-norm)."""
    p, R, v, w, q, qd = S
    return torch.cat((p, rot.mat_to_quat(R), v, w, q, qd), -1)


def unflatten_state(x: torch.Tensor) -> MatState:
    j = NUM_JOINTS
    return MatState(x[..., 0:3], rot.quat_to_mat(x[..., 3:7]), x[..., 7:10], x[..., 10:13],
                    x[..., 13:13 + j], x[..., 13 + j:13 + 2 * j])


def _heading_frame_xy(pos: torch.Tensor, root_pos: torch.Tensor, inv_heading_rot: torch.Tensor) -> torch.Tensor:
    """Express `pos` relative to `root_pos`, rotated into the heading-aligned frame."""
    return rot.quat_rotate_point(pos - root_pos, inv_heading_rot)


def pose_and_velocity_reward(sim: State, ref: State):
    pose_err = ((ref.joint_pos - sim.joint_pos) ** 2).sum(dim=-1)
    vel_err = ((ref.joint_vel - sim.joint_vel) ** 2).sum(dim=-1)
    pose_reward = torch.exp(-POSE_ERR_SCALE * pose_err)
    vel_reward = torch.exp(-VELOCITY_ERR_SCALE * vel_err)
    return pose_reward, vel_reward


def end_effector_reward(sim: State, ref: State):
    feet_sim = sim.root_pos.unsqueeze(-2) + rot.quat_rotate_point(
        od_robot.foot_positions_base_frame(sim.joint_pos), sim.root_rot.unsqueeze(-2))
    feet_ref = ref.root_pos.unsqueeze(-2) + rot.quat_rotate_point(
        od_robot.foot_positions_base_frame(ref.joint_pos), ref.root_rot.unsqueeze(-2))

    inv_heading_sim = rot.quat_conj(rot.calc_heading_rot(sim.root_rot)).unsqueeze(-2)
    inv_heading_ref = rot.quat_conj(rot.calc_heading_rot(ref.root_rot)).unsqueeze(-2)

    rel_sim = _heading_frame_xy(feet_sim, sim.root_pos.unsqueeze(-2), inv_heading_sim)
    rel_ref = _heading_frame_xy(feet_ref, ref.root_pos.unsqueeze(-2), inv_heading_ref)

    diff_xy = rel_ref[..., :2] - rel_sim[..., :2]
    diff_height = feet_ref[..., 2] - feet_sim[..., 2]

    per_foot_err = (diff_xy ** 2).sum(dim=-1) + END_EFFECTOR_HEIGHT_ERR_SCALE * diff_height ** 2
    end_eff_err = per_foot_err.sum(dim=-1)
    return torch.exp(-END_EFFECTOR_ERR_SCALE * end_eff_err)


def root_pose_reward(sim: State, ref: State):
    pos_diff = ref.root_pos - sim.root_pos
    pos_err = (pos_diff ** 2).sum(dim=-1)

    rot_diff = rot.quat_mul(ref.root_rot, rot.quat_conj(sim.root_rot))
    rot_err = rot.normalize_rotation_angle(rot.quat_angle(rot_diff)) ** 2

    root_pose_err = pos_err + 0.5 * rot_err
    return torch.exp(-ROOT_POSE_ERR_SCALE * root_pose_err)


def root_velocity_reward(sim: State, ref: State):
    vel_err = ((ref.root_lin_vel - sim.root_lin_vel) ** 2).sum(dim=-1)
    ang_vel_err = ((ref.root_ang_vel - sim.root_ang_vel) ** 2).sum(dim=-1)
    root_velocity_err = vel_err + 0.1 * ang_vel_err
    return torch.exp(-ROOT_VELOCITY_ERR_SCALE * root_velocity_err)


def recovery_progress_reward(sim: State) -> torch.Tensor:
    """Dense shaping reward for the recovery task (see imitation_task.py's
    recovery_motion_names): 0 when flat on its back, 1 when upright, scaling SMOOTHLY in
    between via the body's local up-axis alignment with world z -- unlike compute_reward's
    terms, which are an exp(-scale * squared_error) that's numerically ~0 for any orientation
    error beyond roughly 60 degrees (confirmed: exp(-20*0.5*(60deg in rad)^2) ~= 1.7e-5, see
    project memory). A recovery episode starting ~180 degrees misaligned gets essentially zero
    learning signal from those terms until it's ALREADY most of the way to standing -- added
    to compute_reward's total (not replacing it) specifically for recovery rows in train.py's
    reward_term, this gives a non-vanishing gradient across the whole journey, while the
    original terms still take over for final pose/standing quality once close to upright."""
    R = rot.quat_to_mat(sim.root_rot)
    up_dot = R[..., 2, 2]  # local z-axis . world z-axis: -1 on back, +1 upright
    return (up_dot + 1.0) * 0.5


def action_rate_penalty(a: torch.Tensor, a_prev: torch.Tensor) -> torch.Tensor:
    """Mean squared per-joint action change between consecutive control steps -- see
    ACTION_RATE_WEIGHT's module note for why this exists. Non-negative; subtract
    ACTION_RATE_WEIGHT * this from the reward total (not folded into compute_reward's own exp-
    weighted terms, since this is a penalty on the ACTION, not a tracking-error term on state)."""
    return ((a - a_prev) ** 2).mean(dim=-1)


def compute_reward(sim: State, ref: State) -> dict:
    """Returns the 5 reward terms plus their ImitationTask-weighted sum ("total")."""
    pose_r, vel_r = pose_and_velocity_reward(sim, ref)
    end_eff_r = end_effector_reward(sim, ref)
    root_pose_r = root_pose_reward(sim, ref)
    root_vel_r = root_velocity_reward(sim, ref)

    total = (POSE_WEIGHT * pose_r + VELOCITY_WEIGHT * vel_r
             + END_EFFECTOR_WEIGHT * end_eff_r + ROOT_POSE_WEIGHT * root_pose_r
             + ROOT_VELOCITY_WEIGHT * root_vel_r)

    return {
        "pose": pose_r, "velocity": vel_r, "end_effector": end_eff_r,
        "root_pose": root_pose_r, "root_velocity": root_vel_r, "total": total,
    }


def _tracking_fail(sim: State, ref: State, dist_threshold: float, rot_threshold: float) -> torch.Tensor:
    """Shared raw root-drift/rotation fail check, parameterized by threshold -- used both for
    the real ENTRY threshold (compute_termination's own DIST_FAIL_THRESHOLD/ROT_FAIL_THRESHOLD)
    and, with stricter (smaller) thresholds, as the EXIT condition for the general distress
    state machine below. Keeping one implementation means the two can never silently drift
    apart."""
    pos_diff = ref.root_pos[..., 1:3] - sim.root_pos[..., 1:3]  # (y, z) only, matches
    # imitation_terminal_condition's root_pos_ref[1:3] - root_pos_sim[1:3].
    pos_fail = (pos_diff ** 2).sum(dim=-1) > dist_threshold

    rot_diff = rot.quat_mul(ref.root_rot, rot.quat_conj(sim.root_rot))
    rot_angle = rot.normalize_rotation_angle(rot.quat_angle(rot_diff))
    rot_fail = rot_angle.abs() > rot_threshold
    return pos_fail | rot_fail


# Generalized "distress" state machine (2026-11-12): replaces tying the wide-action-range/no-
# hard-termination treatment to a specific FILE (recovery_motion_names) with tying it to actual
# measured tracking failure, during ANY gait -- see project memory for the full design
# discussion. A naive "is_recovery = currently failing" (using DIST_FAIL_THRESHOLD/
# ROT_FAIL_THRESHOLD for BOTH entry and exit) collapses compute_termination's own
# `fail & ~is_recovery` to always-False -- hard termination would never fire for ANYONE, since
# the override condition becomes identical to the trigger condition. The exit thresholds here
# are deliberately much STRICTER (a real recovery, not just barely-under-the-fail-line) to give
# genuine hysteresis: once distressed, an episode stays protected until it's actually back under
# control, not just technically no-longer-failing -- avoiding both the collapse above and rapid
# flicker right at one boundary.
DISTRESS_EXIT_DIST_THRESHOLD = DIST_FAIL_THRESHOLD / 4
DISTRESS_EXIT_ROT_THRESHOLD = ROT_FAIL_THRESHOLD / 4


def distress_enter(sim: State, ref: State) -> torch.Tensor:
    """True where tracking error just crossed the normal hard-fail threshold -- the ENTRY side
    of the hysteresis band. Identical condition to compute_termination's own un-overridden fail
    check, by construction (shares _tracking_fail)."""
    return _tracking_fail(sim, ref, DIST_FAIL_THRESHOLD, ROT_FAIL_THRESHOLD)


def distress_exit(sim: State, ref: State) -> torch.Tensor:
    """True where tracking error is back under the stricter EXIT threshold -- i.e. genuinely
    recovered, not just barely under the fail line."""
    return ~_tracking_fail(sim, ref, DISTRESS_EXIT_DIST_THRESHOLD, DISTRESS_EXIT_ROT_THRESHOLD)


def compute_termination(sim: State, ref: State, is_recovery: torch.Tensor = None) -> torch.Tensor:
    """Hard (boolean) termination from root drift/rotation only -- see module docstring
    for what's intentionally left out (contact, motion-over).

    is_recovery: optional bool tensor, broadcastable to the batch shape. True rows never fail
    this check -- originally meant specifically for a scripted recovery episode (see
    imitation_task.py's recovery_motion_names), but as of 2026-11-12 this is also how the general
    per-transition "distressed" flag (env.py's worker, buffer.py's stored field) gets applied --
    same mechanism, now triggered by actually measured tracking failure during ANY gait, not just
    a specific file. Either way, the idea is the same: a row already far from the reference
    isn't failed a second time for still being far from it, it's given a chance to recover."""
    fail = _tracking_fail(sim, ref, DIST_FAIL_THRESHOLD, ROT_FAIL_THRESHOLD)
    if is_recovery is not None:
        fail = fail & ~is_recovery
    return fail


def compute_soft_termination_prob(sim: State, ref: State, sharpness: float = 100.0,
                                  is_recovery: torch.Tensor = None) -> torch.Tensor:
    """Differentiable surrogate for compute_termination: survival probability in [0, 1],
    smoothly approaching 0 past the hard thresholds (sigmoid-softened, matching the pattern
    SuperMimic's own mdp.py uses so gradients flow through termination inside imagined
    rollouts). 1 - this is a soft "done" probability.

    sharpness=100 (was 50, briefly tried 200): with DIST_FAIL_THRESHOLD=0.02, 50 gave
    sigmoid(50*0.02)=0.731 survival per step even at EXACTLY zero tracking error -- not a
    training bug, but this formula (not SuperMimic's own, which uses a *learned* termination
    head fit to real termination labels, see mdp.py's module note) has no data-driven
    calibration, so a well-tracking policy was never able to clear ~0.73/step, which compounds
    to a ~0.023 ceiling on imag_survival over H=12 regardless of policy quality (confirmed by
    hand -- see project memory). 200 raised the ceiling to ~0.80 as predicted, but a 3000-iter
    pace_run3 validation showed the policy visibly exploiting the loosened pose tolerance:
    splayed/contorted poses on real rollouts, lower sim_rew (~0.05-0.18 vs ~0.3-0.45 for
    sharpness=50 at a comparable iteration count) despite positive net displacement -- almost
    certainly tumbling/flailing forward rather than walking, confirmed by eye on rendered
    frames. 100 is a deliberately more conservative middle ground: sigmoid(100*0.02)=0.881 at
    zero error, ceiling 0.881^12~=0.22 -- 10x pace_run1/2's headroom but far less than 200's,
    aiming to keep enough pose-tracking pressure that the policy doesn't learn degenerate gaits.
    Needs its own short validation run + visual check before trusting it either. The hard
    threshold itself (and thus real-episode termination behavior, which doesn't use this
    function) is unchanged by sharpness; rot_survival was already saturated near 1 at this
    scale and is unaffected. Not yet validated with a full run.

    is_recovery: optional bool tensor, broadcastable to the batch shape -- see
    compute_termination's docstring. True rows get survival forced to 1 (never contributes to
    "done"), since starting far from the reference is the recovery task's premise, not a
    failure to survive."""
    pos_diff = ref.root_pos[..., 1:3] - sim.root_pos[..., 1:3]
    pos_err = (pos_diff ** 2).sum(dim=-1)
    pos_survival = torch.sigmoid(sharpness * (DIST_FAIL_THRESHOLD - pos_err))

    rot_diff = rot.quat_mul(ref.root_rot, rot.quat_conj(sim.root_rot))
    rot_angle = rot.normalize_rotation_angle(rot.quat_angle(rot_diff)).abs()
    rot_survival = torch.sigmoid(sharpness * (ROT_FAIL_THRESHOLD - rot_angle))

    survival = pos_survival * rot_survival
    if is_recovery is not None:
        survival = torch.where(is_recovery, torch.ones_like(survival), survival)
    return survival


# ---------------------------------------------------------------------------------------
# World-model / policy / critic input features (networks.py), operating on MatState. Roles
# and shapes mirror supermimic_vibed's mdp.py, resized to the OD robot's 12 joints and 4
# tracked feet instead of 29 joints / 14 BeyondMimic bodies, and built from THIS project's
# own reward's error signals (compute_reward, above) rather than SuperMimic's own
# BeyondMimic-style per-body tracking formula -- Phase 1 deliberately ported the existing,
# already-validated reward instead of switching formulas, so the critic sees the same
# errors the reward is actually computed from.
#
# Sim-context vector xi (plan.md Phase 5.2, added after Phase 1's initial "not needed to prove
# the core algorithm can track a gait" deferral): per-env domain randomization context, fed to
# both WorldModel (so it conditions its dynamics prediction on the actual randomized physics)
# and Policy (so it can adapt its control to the current randomization) -- same role as the
# paper's own xi. Simplified vs. the original's 33-dim [friction, 29 joint offsets, 3 CoM
# offset]: THIS port's xi = [friction (1), 12 per-joint action-space offsets] = 13-dim, CoM
# offset dropped -- PyBullet's changeDynamics has no direct "shift the local inertial frame
# origin" parameter the way Isaac Lab's set_coms_index does (would need an external-force-proxy
# hack to approximate it credibly), and the per-joint offset is applied as a small constant bias
# added to the ACTION before scaling to torque (env.py's workers), not a pybullet-level "position
# offset" -- makes sense for THIS robot's torque control (the reference's G1 uses position-target
# control, where an "offset" is a literal joint calibration error in angle-space; the closest
# torque-control analogue is a constant control-signal bias, which is what's implemented).
# `default_q` is the all-zero pose (od.py's INIT_MOTOR_ANGLES convention).
# ---------------------------------------------------------------------------------------

XI_DIM = 1 + NUM_JOINTS  # friction, per-joint action-space offset -- see note above
INV_DIM = 1 + 6 + 3 + 3 + NUM_JOINTS + NUM_JOINTS  # height, 6D tilt, body-frame v/w, q, qd
OBS_DIM = NUM_JOINTS + NUM_JOINTS + 3 + 3 + NUM_JOINTS + 6 + XI_DIM  # q-q0, qd, body-w, gravity, a_prev, rel root ori, xi
CRITIC_EXTRA = NUM_JOINTS + NUM_LEGS * 3 + 3 + 6  # q err, per-foot pos err, root pos err, rel root ori

DEFAULT_Q = torch.zeros(NUM_JOINTS)

# Per-component noise half-widths (q, qd, body-w, gravity, a_prev, rel-ori, xi) added to the
# policy's observation during training, in the spirit of the paper's Table A.1 sensor noise --
# a reasonable default, not tuned against real sensor characteristics. xi gets zero noise: it's
# the simulation's own actual randomized context (privileged information during training), not
# a sensor reading -- same treatment as a_prev.
OBS_NOISE = torch.cat((
    torch.full((NUM_JOINTS,), 0.01), torch.full((NUM_JOINTS,), 0.1),
    torch.full((3,), 0.05), torch.full((3,), 0.02),
    torch.zeros(NUM_JOINTS), torch.full((6,), 0.05),
    torch.zeros(XI_DIM),
))


def invariant_features(S: MatState) -> torch.Tensor:
    """Heading- and position-invariant state features: height, tilt (6D), body-frame
    velocities, joint pos/vel -- the world model's state input (with the action appended)."""
    p, R, v, w, q, qd = S
    Ht = rot.yaw_mat(R).transpose(-1, -2)
    Rt = R.transpose(-1, -2)
    return torch.cat((
        p[..., 2:3],
        rot.mat_to_6d(Ht @ R),
        (Rt @ v.unsqueeze(-1)).squeeze(-1),
        (Rt @ w.unsqueeze(-1)).squeeze(-1),
        q, qd,
    ), dim=-1)


def feet_world(S: MatState) -> torch.Tensor:
    """[..., 4, 3] world-frame foot positions -- the MatState-based twin of the quaternion FK
    inside end_effector_reward, for critic_features below."""
    feet_base = od_robot.foot_positions_base_frame(S.q)
    return S.p.unsqueeze(-2) + (S.R.unsqueeze(-3) @ feet_base.unsqueeze(-1)).squeeze(-1)


def obs(S: MatState, a_prev: torch.Tensor, ref_R: torch.Tensor, xi: torch.Tensor,
       default_q: torch.Tensor = None) -> torch.Tensor:
    """Noise-free proprioceptive observation plus reference-relative root orientation and the
    sim-context vector xi -- the policy's per-timestep input. Add OBS_NOISE (scaled, see
    add_noise) during training. xi is required (not optional/defaulted) since dropping it
    silently would desync OBS_DIM from what the Policy network was actually built for -- pass
    torch.zeros(..., XI_DIM) explicitly at any call site that genuinely has no randomization
    context available."""
    if default_q is None:
        default_q = DEFAULT_Q.to(S.q.device, S.q.dtype)
    p, R, v, w, q, qd = S
    Rt = R.transpose(-1, -2)
    g_b = -R[..., 2, :]  # R^T @ (0, 0, -1): body-frame projected gravity
    rel = rot.mat_to_6d(Rt @ ref_R)
    return torch.cat((q - default_q, qd, (Rt @ w.unsqueeze(-1)).squeeze(-1), g_b, a_prev, rel, xi), dim=-1)


def add_noise(o: torch.Tensor, noise: torch.Tensor = None) -> torch.Tensor:
    if noise is None:
        noise = OBS_NOISE.to(o.device, o.dtype)
    return o + (2 * torch.rand_like(o) - 1) * noise


def critic_features(S: MatState, ref: MatState) -> torch.Tensor:
    """Critic input: invariant state plus the robot-vs-reference errors this port's own
    reward is built from -- joint-angle error, per-foot position error (heading-frame xy +
    height, as in end_effector_reward), root position error, and relative root orientation."""
    p, R, v, w, q, qd = S
    rp, rR, rv, rw, rq, rqd = ref
    Ht = rot.yaw_mat(R).transpose(-1, -2)

    feet, ref_feet = feet_world(S), feet_world(ref)
    inv_heading = rot.quat_conj(rot.calc_heading_rot(rot.mat_to_quat(R))).unsqueeze(-2)
    inv_heading_ref = rot.quat_conj(rot.calc_heading_rot(rot.mat_to_quat(rR))).unsqueeze(-2)
    rel_feet = rot.quat_rotate_point(feet - p.unsqueeze(-2), inv_heading)
    rel_ref_feet = rot.quat_rotate_point(ref_feet - rp.unsqueeze(-2), inv_heading_ref)
    e_feet = torch.cat((rel_ref_feet[..., :2] - rel_feet[..., :2],
                        ref_feet[..., 2:3] - feet[..., 2:3]), dim=-1).flatten(-2)

    e_root = (Ht @ (rp - p).unsqueeze(-1)).squeeze(-1)
    rel = rot.mat_to_6d(R.transpose(-1, -2) @ rR)
    return torch.cat((invariant_features(S), rq - q, e_feet, e_root, rel), dim=-1)
