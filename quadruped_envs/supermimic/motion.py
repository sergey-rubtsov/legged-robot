"""Reference-motion lookup for the OD quadruped, built on top of the project's existing,
already-correct motion_imitation.utilities.motion_data.MotionData (interpolation, loop-wrap
cycle offsets, finite-difference velocities). Only the non-differentiable, precomputed
"ground truth" side of the MDP needs this -- nothing here is backpropagated through; see
quadruped_envs/supermimic/mdp.py for the differentiable reward/termination that consumes
its output.
"""
import numpy as np
import torch
from motion_imitation.utilities import motion_data

from quadruped_envs.supermimic.mdp import MatState, State
from quadruped_envs.supermimic import rotations as rot

# The real env's control step (od_gym_config.SimulationParameters defaults: sim_time_step_s=
# 0.005, num_action_repeat=1, unchanged by build_imitation_env), i.e. the dt reference time
# actually advances by per RL step -- NOT the motion file's own FrameDuration (0.01667s for
# pace.txt): see BakedMotion's docstring for why that means continuous-time lookups, not an
# integer frame counter.
CONTROL_DT = 0.005


class ReferenceMotion:
    """Wraps one or more DeepMimic-JSON motion clips (quadruped_envs/data/motions/od/*.txt)."""

    def __init__(self, motion_files, device="cpu", dtype=torch.float32):
        self._motions = [motion_data.MotionData(f) for f in motion_files]
        self._device = device
        self._dtype = dtype

    def __len__(self):
        return len(self._motions)

    def duration(self, motion_id: int = 0) -> float:
        return self._motions[motion_id].get_duration()

    def is_over(self, time: float, motion_id: int = 0) -> bool:
        return self._motions[motion_id].is_over(time)

    def state_at(self, time: float, motion_id: int = 0) -> State:
        """Reference state at a single point in time, as an unbatched (shape [D]) State."""
        motion = self._motions[motion_id]
        pose = motion.calc_frame(time)
        vel = motion.calc_frame_vel(time)

        root_pos = motion.get_frame_root_pos(pose)
        root_rot = motion.get_frame_root_rot(pose)
        joint_pos = motion.get_frame_joints(pose)
        root_lin_vel = motion.get_frame_root_vel(vel)
        root_ang_vel = motion.get_frame_root_ang_vel(vel)
        joint_vel = motion.get_frame_joints_vel(vel)

        t = lambda x: torch.as_tensor(np.asarray(x), device=self._device, dtype=self._dtype)
        return State(
            root_pos=t(root_pos), root_rot=t(root_rot),
            root_lin_vel=t(root_lin_vel), root_ang_vel=t(root_ang_vel),
            joint_pos=t(joint_pos), joint_vel=t(joint_vel),
        )

    def batch_state_at(self, times, motion_ids=None) -> State:
        """Reference states for a batch of (time, motion_id) pairs, stacked into a State
        whose tensors have leading shape [len(times)]."""
        if motion_ids is None:
            motion_ids = [0] * len(times)
        states = [self.state_at(t, m) for t, m in zip(times, motion_ids)]
        return State(*(torch.stack(field) for field in zip(*(
            (s.root_pos, s.root_rot, s.root_lin_vel, s.root_ang_vel, s.joint_pos, s.joint_vel)
            for s in states
        ))))


# Per-frame goal layout: dp(3), rot6d(6), root_v_local(3), root_w_local(3), q(12), qd(12).
# GOAL_Q_OFFSET named explicitly (not just folded into the GOAL_FRAME_DIM sum) since
# networks.py's Policy reads the reference joint-position sub-vector directly out of a raw
# goal tensor for its optional ref_pd bias (see networks.py's module note) -- keeping the
# offset computed here, in one place, means a layout change can't silently desync the two.
GOAL_DP_DIM, GOAL_ROT6D_DIM, GOAL_V_DIM, GOAL_W_DIM, GOAL_Q_DIM, GOAL_QD_DIM = 3, 6, 3, 3, 12, 12
GOAL_Q_OFFSET = GOAL_DP_DIM + GOAL_ROT6D_DIM + GOAL_V_DIM + GOAL_W_DIM
GOAL_FRAME_DIM = GOAL_DP_DIM + GOAL_ROT6D_DIM + GOAL_V_DIM + GOAL_W_DIM + GOAL_Q_DIM + GOAL_QD_DIM


class BakedMotion:
    """Vectorized (batched-over-time) version of a single MotionData clip's `calc_frame` /
    `calc_frame_vel`, for the training loop's goal/reference lookups -- `MotionData` itself
    only takes a scalar Python time, and calling it in a Python loop per sample would be the
    throughput bottleneck for a batch of (up to a few hundred/thousand) different times per
    imagined BPTT step. Bakes the clip's native discrete frames into tensors once, then
    reimplements MotionData's exact interpolation + cycle-offset formulas in torch -- see
    validate_motion.py for a cross-check against `ReferenceMotion.state_at` (which wraps the
    real `MotionData` directly) confirming this reimplementation matches it.

    Unlike supermimic_vibed's `MotionLib` (which advances an integer frame counter by
    exactly 1 per RL step, because its control rate exactly matches the resampled clip's
    frame rate), this project's real env steps at 0.005s while `pace.txt` is recorded at
    0.01667s -- one RL step is a fraction of a frame, so lookups here stay in CONTINUOUS TIME
    (interpolated), matching how the real `ImitationTask` actually drives its reference model.
    """

    def __init__(self, motion_file: str, device="cpu", dtype=torch.float32):
        m = motion_data.MotionData(motion_file)
        self.device, self.dtype = device, dtype
        self.frame_dt = m.get_frame_duration()
        self.num_frames = m.get_num_frames()
        self.duration = m.get_duration()
        self.enable_loop = m.enable_loop()
        self.enable_cycle_pos = m._enable_cycle_offset_pos
        self.enable_cycle_rot = m._enable_cycle_offset_rot
        self.cycle_delta_heading = float(m._cycle_delta_heading)

        t = lambda a: torch.as_tensor(np.asarray(a), device=device, dtype=dtype)
        frames = np.stack([m.get_frame(f) for f in range(self.num_frames)])
        vels = np.stack([m.get_frame_vel(f) for f in range(self.num_frames)])
        self.root_p = t(frames[:, :3])
        self.root_quat = t(frames[:, 3:7])
        self.q = t(frames[:, 7:])
        self.root_v = t(vels[:, :3])
        self.root_w = t(vels[:, 3:6])
        self.qd = t(vels[:, 6:])
        self.cycle_delta_pos = t(m._cycle_delta_pos)

    def _phase(self, time: torch.Tensor) -> torch.Tensor:
        phase = time / self.duration
        return phase - torch.floor(phase) if self.enable_loop else phase.clamp(0.0, 1.0)

    def _cycle_count(self, time: torch.Tensor) -> torch.Tensor:
        count = torch.floor(time / self.duration).long()
        return count if self.enable_loop else count.clamp(0, 1)

    def _blend_idx(self, time: torch.Tensor):
        F, dur = self.num_frames, self.duration
        phase = self._phase(time)
        f0 = (phase * (F - 1)).long().clamp(0, F - 1)
        f1 = (f0 + 1).clamp(max=F - 1)
        norm_time = phase * dur
        t0, t1 = f0.to(self.dtype) * self.frame_dt, f1.to(self.dtype) * self.frame_dt
        blend = ((norm_time - t0) / (t1 - t0).clamp_min(1e-8)).clamp(0.0, 1.0)
        if not self.enable_loop:
            clamp_lo, clamp_hi = time <= 0, time >= dur
            f0 = torch.where(clamp_lo, torch.zeros_like(f0), torch.where(clamp_hi, torch.full_like(f0, F - 1), f0))
            f1 = torch.where(clamp_lo, torch.zeros_like(f1), torch.where(clamp_hi, torch.full_like(f1, F - 1), f1))
            blend = torch.where(clamp_lo | clamp_hi, torch.zeros_like(blend), blend)
        return f0, f1, blend

    def _cycle_offset(self, n: torch.Tensor):
        """Net root translation/rotation accumulated over `n` (>=0, per-sample) full loops of
        the clip -- matches MotionData._calc_cycle_offset_pos/_calc_cycle_offset_rot exactly.
        `n` is small in practice (an episode rarely spans more than a handful of loops), so a
        Python loop up to n.max() (not batch size) building a small lookup table, then a
        gather, is cheap and keeps this a direct, checkable port of the original's formula."""
        z = torch.tensor([0.0, 0.0, 1.0], device=self.device, dtype=self.dtype)
        max_n = int(n.max().item()) if n.numel() else 0
        pos_table = [torch.zeros(3, device=self.device, dtype=self.dtype)]
        rot_table = [torch.tensor([0.0, 0.0, 0.0, 1.0], device=self.device, dtype=self.dtype)]
        acc_pos = pos_table[0].clone()
        delta_pos = self.cycle_delta_pos if self.enable_cycle_pos else torch.zeros_like(self.cycle_delta_pos)
        for i in range(max_n):
            heading_i = torch.full((), i * self.cycle_delta_heading, device=self.device, dtype=self.dtype)
            rot_i = rot.quat_about_axis(heading_i, z) if self.enable_cycle_rot else rot_table[0]
            acc_pos = acc_pos + rot.quat_rotate_point(delta_pos, rot_i)
            pos_table.append(acc_pos.clone())
            if self.enable_cycle_rot:
                heading_next = torch.full((), (i + 1) * self.cycle_delta_heading, device=self.device, dtype=self.dtype)
                rot_table.append(rot.quat_about_axis(heading_next, z))
            else:
                rot_table.append(rot_table[0])
        pos_table, rot_table = torch.stack(pos_table), torch.stack(rot_table)
        return pos_table[n], rot_table[n]

    def frame_state(self, time: torch.Tensor) -> MatState:
        """Reference MatState at a batch of continuous times [B] (any shape, any range --
        loops/clamps like MotionData does)."""
        f0, f1, blend = self._blend_idx(time)
        b = blend.unsqueeze(-1)
        p = (1 - b) * self.root_p[f0] + b * self.root_p[f1]
        quat = rot.quat_slerp(self.root_quat[f0], self.root_quat[f1], blend)
        q = (1 - b) * self.q[f0] + b * self.q[f1]
        v = (1 - b) * self.root_v[f0] + b * self.root_v[f1]
        w = (1 - b) * self.root_w[f0] + b * self.root_w[f1]
        qd = (1 - b) * self.qd[f0] + b * self.qd[f1]

        n = self._cycle_count(time)
        pos_off, rot_off = self._cycle_offset(n)
        p = rot.quat_rotate_point(p, rot_off) + pos_off
        quat = rot.quat_mul(rot_off, quat)
        v = rot.quat_rotate_point(v, rot_off)
        w = rot.quat_rotate_point(w, rot_off)

        return MatState(p, rot.quat_to_mat(quat), v, w, q, qd)

    def goal(self, time: torch.Tensor, K: int, control_dt: float) -> torch.Tensor:
        """Goal features from the reference only: K future frames, `control_dt` apart, drawn
        relative to the current reference frame -- mirrors supermimic_vibed's
        `MotionLib.goal`. Position/orientation are expressed relative to the CURRENT
        heading-only frame; velocities are each future frame's own local-frame velocity
        (not current-heading-relative) -- matching the original exactly."""
        B = time.shape[0]
        future_t = time.unsqueeze(-1) + control_dt * torch.arange(1, K + 1, device=time.device, dtype=time.dtype)
        fut = self.frame_state(future_t.reshape(-1))
        cur = self.frame_state(time)

        p_fut, R_fut = fut.p.view(B, K, 3), fut.R.view(B, K, 3, 3)
        v_fut, w_fut = fut.v.view(B, K, 3), fut.w.view(B, K, 3)
        q_fut, qd_fut = fut.q.view(B, K, -1), fut.qd.view(B, K, -1)

        Ht = rot.yaw_mat(cur.R).transpose(-1, -2)  # (B,3,3)
        dp = torch.einsum("bij,bkj->bki", Ht, p_fut - cur.p.unsqueeze(1))
        rel_R = torch.einsum("bij,bkjl->bkil", Ht, R_fut)
        rot6d = rot.mat_to_6d(rel_R)

        Rt_fut = R_fut.transpose(-1, -2)
        v_local = torch.einsum("bkij,bkj->bki", Rt_fut, v_fut)
        w_local = torch.einsum("bkij,bkj->bki", Rt_fut, w_fut)

        feats = torch.cat((dp, rot6d, v_local, w_local, q_fut, qd_fut * 0.1), dim=-1)  # (B,K,39)
        return feats.reshape(B, -1)


class MultiMotion:
    """Dispatches `frame_state`/`goal` across several BakedMotion clips by a per-sample motion
    index, so one batched call can mix rows drawn from different reference motions -- needed to
    train jointly across a multi-gait library (one world model amortizing shared dynamics
    across motions, per the SuperMimic paper's core finding) instead of a single clip. Index i
    in `motion_files` must match the SAME order passed to env.py's TrackingSim (and so to each
    worker's build_imitation_env), since that's what the real simulator's own
    `ImitationTask._active_motion_id` indexes into -- see train.py's module note.

    Groups the batch by distinct index value (small and fixed, e.g. <=5 gaits), routes each
    group to its own BakedMotion, and scatters results back into the original row order. With
    a single motion file this reduces to plain per-call dispatch (no grouping overhead beyond
    one mask)."""

    def __init__(self, motion_files, device="cpu", dtype=torch.float32):
        self.motions = [BakedMotion(f, device=device, dtype=dtype) for f in motion_files]
        self.device, self.dtype = device, dtype

    def frame_state(self, motion_idx: torch.Tensor, time: torch.Tensor) -> MatState:
        """`motion_idx` and `time` must have the SAME shape (e.g. both (B,), or both (B,hist)
        for a history lookup) -- boolean-masks elementwise, so `time`'s shape (not just its
        batch dim) is what `out` is allocated against, matching plain BakedMotion.frame_state's
        own any-shape support."""
        out = None
        for i, bm in enumerate(self.motions):
            mask = motion_idx == i
            if not mask.any():
                continue
            sub = bm.frame_state(time[mask])
            if out is None:
                out = tuple(torch.zeros(time.shape + f.shape[1:], device=self.device, dtype=f.dtype)
                           for f in sub)  # fmt: skip
            for field, sub_field in zip(out, sub):
                field[mask] = sub_field
        return MatState(*out)

    def goal(self, motion_idx: torch.Tensor, time: torch.Tensor, K: int, control_dt: float) -> torch.Tensor:
        out = None
        for i, bm in enumerate(self.motions):
            mask = motion_idx == i
            if not mask.any():
                continue
            sub = bm.goal(time[mask], K, control_dt)
            if out is None:
                out = torch.zeros((time.shape[0],) + sub.shape[1:], device=self.device, dtype=sub.dtype)
            out[mask] = sub
        return out
