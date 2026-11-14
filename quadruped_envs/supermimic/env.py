"""Vectorized real-rollout collection: N independent PyBullet processes, one OD robot each.

SuperMimic's `collect()` step needs batched real-env transitions. Isaac Lab/MuJoCo-Warp can
batch thousands of environments on one GPU; there's no equivalent for PyBullet, so this runs
N independent worker *processes* instead, each holding its own `build_imitation_env` instance
(own DIRECT-mode PyBullet connection -- these don't share any GPU/display state, so plain
multiprocessing is enough). N should be small, matched to the machine's performance cores
(think 8-32) -- this is the fundamental ceiling versus the paper's 4096 GPU-simulated envs.

Return-based reset-bin sampling (SuperMimic paper Appendix D.1: 50% uniform / 40% softmax on
negative return / 10% decaying average) lives here in TrackingSim, mirroring the original
reference implementation's own TrackingSim (supermimic_vibed/supermimic/env.py) class
boundary: it already receives, every step, the (t, motion_id, reward, done, ep_step) needed to
attribute a finished episode's return to the (motion, time-range) bin it started from (via
ImitationTask.build_bin_layout/bin_index_for) and update a running per-bin EMA -- no new data
needs to flow from worker to main for this. Only the resulting bin_probs distribution needs to
be pushed back down to each worker (see set_bin_probs below), which then uses it to bias real
resets (ImitationTask.set_bin_probs) instead of picking motion+time uniformly at random. Off by
default (bin_duration=None) -- zero behavior change unless explicitly enabled.

When `motion_files` has more than one entry, each of the N worker envs independently samples
its own clip on every reset -- this is ImitationTask's own existing multi-motion support
(`_sample_ref_motion`), nothing added here. `read_both()` reports which clip is currently
active (`task._active_motion_id`) alongside the state, so the training loop can look up the
matching clip in its own vectorized copy (motion.py's `MultiMotion`) for imagined rollouts --
without this, multi-motion collection would silently score/imagine every env against whatever
single motion file train.py's `BakedMotion` happened to load.

Uses plain `multiprocessing` (Pipe + Process), not stable_baselines3's SubprocVecEnv: SB3 is
not part of this project's installed, verified dependency set (see requirements.txt / Phase 0
notes) and pulling it in just for one utility class would reintroduce exactly the dependency
fragility Phase 0 spent effort removing. The worker/pipe pattern below is the same one
SB3's SubprocVecEnv uses, just self-contained and scoped to what this project needs.
"""
import multiprocessing as mp
import os

import numpy as np
import torch

from quadruped_envs.supermimic.mdp import MAX_STEP_REWARD, State

NUM_JOINTS = 12


def _softmax(x: np.ndarray) -> np.ndarray:
    x = x - x.max()  # numerical stability, doesn't change the result
    e = np.exp(x)
    return e / e.sum()


def _read_state(pyb, model_id):
    root_pos, root_rot = pyb.getBasePositionAndOrientation(model_id)
    lin_vel, ang_vel = pyb.getBaseVelocity(model_id)
    joint_pos = np.empty(NUM_JOINTS, dtype=np.float32)
    joint_vel = np.empty(NUM_JOINTS, dtype=np.float32)
    for j in range(NUM_JOINTS):
        js = pyb.getJointStateMultiDof(model_id, j)
        joint_pos[j] = js[0][0]
        joint_vel[j] = js[1][0]
    return (np.asarray(root_pos, dtype=np.float32), np.asarray(root_rot, dtype=np.float32),
            np.asarray(lin_vel, dtype=np.float32), np.asarray(ang_vel, dtype=np.float32),
            joint_pos, joint_vel)


XI_FRICTION_RANGE = (0.5, 1.2)  # lateral friction scale, loosely matching the reference's own 0.3-1.2
XI_JOINT_OFFSET_RANGE = (-0.05, 0.05)  # action-space units, added to the commanded action -- see mdp.py's note


def _worker(remote, motion_files, robot_class, enable_randomizer, seed,
           recovery_motion_names, recovery_init_prob, recovery_sample_prob, bin_duration,
           motor_control_mode, adaptive_distress):
    # Imported inside the worker (spawn start method -> fresh interpreter per process).
    from quadruped_envs.od_imitation_env import build_imitation_env
    from motion_imitation.robots import robot_config

    env = build_imitation_env(
        motion_files=motion_files, num_parallel_envs=1, enable_rendering=False,
        enable_randomizer=enable_randomizer, robot_class=robot_class,
        recovery_motion_names=recovery_motion_names, recovery_init_prob=recovery_init_prob,
        recovery_sample_prob=recovery_sample_prob,
        bin_duration=bin_duration, motor_control_mode=motor_control_mode,
    )
    env.seed(seed)
    # Whether a normalized [-1,1] action needs converting into a joint-position target before
    # reaching the real env -- see mdp.py's ACTION_POS_SCALE module note. The buffer/policy/
    # world-model side ALWAYS works in normalized action units regardless of control mode (same
    # as how `joint_offset` below is applied in normalized units too) -- this conversion happens
    # only at this one real-env boundary.
    is_position_mode = motor_control_mode == robot_config.MotorControlMode.POSITION
    if is_position_mode:
        from quadruped_envs.supermimic.mdp import (ACTION_POS_SCALE, DEFAULT_Q,
                                                    RECOVERY_Q_CENTER, RECOVERY_Q_HALF_RANGE)
        default_q_np = DEFAULT_Q.numpy().copy()
        recovery_center_np = RECOVERY_Q_CENTER.numpy().copy()
        recovery_half_range_np = RECOVERY_Q_HALF_RANGE.numpy().copy()

    pyb = env._pybullet_client
    task = env._task
    sim_model = env.robot.quadruped
    ground = env.get_ground()
    rng = np.random.default_rng(seed)

    # ---- generalized "distress" state machine (2026-11-12, see mdp.py's module note) ----
    # Opt-in alternative to the file-based `task._is_recovery_motion()` check: ties the wide-
    # action-range/no-hard-termination treatment to ACTUAL measured tracking failure during ANY
    # gait, not just a dedicated recovery file.
    #
    # MUST hook `task._terminal_condition` itself (not just `_is_recovery_motion`), and MUST
    # decide entry INLINE, using the exact state `done()` itself is currently evaluating --
    # tried the simpler approach first (an external is_distressed flag, updated from env.step()'s
    # own return value, consulted via a patched _is_recovery_motion) and it never triggered once
    # in 5000+ iterations plus a dedicated high-noise smoke test. Root cause: _terminal_condition
    # runs INSIDE env.step(), before that call returns control to this worker -- by the time this
    # code could react to "that step just failed" and update an external flag, done() had already
    # used the STALE pre-step flag value (still False, since nothing had failed yet going INTO
    # that step) and ended the episode via the ordinary unpatched path. There is no way to
    # "pre-know" the first failing step is about to fail. Patching _terminal_condition directly
    # fixes this: the wrapper below computes entry using the SAME live call the original function
    # already makes, atomically, so there's no stale-read window.
    #
    # Also folds in real ground contact (non-foot touching the ground) -- confirmed via reading
    # imitation_terminal_conditions.py that the REAL terminal_condition checks this FIRST, before
    # root position/rotation drift, and it's how most real falls actually get detected (the body/
    # knee hits the ground well before root drift necessarily crosses DIST_FAIL_THRESHOLD/
    # ROT_FAIL_THRESHOLD). mdp.py's distress_enter/distress_exit deliberately leave contact out
    # (the world model doesn't simulate it, so the analytic/imagined-rollout side can't use it
    # either -- see mdp.py's module note) -- that's still correct for the training side; this
    # worker-side wrapper is the one place real contact needs to feed in too.
    if adaptive_distress:
        from quadruped_envs.supermimic.mdp import State as _MdpState, distress_exit

        foot_link_ids = env.robot.GetFootLinkIDs()

        def _to_mdp_state(fields6):
            return _MdpState(
                root_pos=torch.as_tensor(fields6[0], dtype=torch.float32).unsqueeze(0),
                root_rot=torch.as_tensor(fields6[1], dtype=torch.float32).unsqueeze(0),
                root_lin_vel=torch.as_tensor(fields6[2], dtype=torch.float32).unsqueeze(0),
                root_ang_vel=torch.as_tensor(fields6[3], dtype=torch.float32).unsqueeze(0),
                joint_pos=torch.as_tensor(fields6[4], dtype=torch.float32).unsqueeze(0),
                joint_vel=torch.as_tensor(fields6[5], dtype=torch.float32).unsqueeze(0),
            )

        def _real_contact_fail():
            if env.env_step_counter == 0:
                return False  # matches imitation_terminal_condition's own post-reset grace step
            for c in pyb.getContactPoints(bodyA=sim_model, bodyB=ground):
                if c[3] not in foot_link_ids:
                    return True
            return False

        # Mutable holder (not a plain bool) so the closure below always sees the LATEST value --
        # same role as `nonlocal`, needed because this is read AND written from inside a function
        # defined once and handed off to `task`, not called directly from this scope.
        distress = {"active": False}

        def _patched_terminal_condition(env_arg):
            if distress["active"]:
                # Already distressed: exit only once genuinely back under control (stricter
                # thresholds, see mdp.py's DISTRESS_EXIT_* note) AND no bad contact right now --
                # otherwise keep forgiving real failures, same as a scripted recovery episode
                # already does via is_recovery. Never hard-terminate while in this state.
                state = read_both()
                recovered = (not _real_contact_fail()
                            and bool(distress_exit(_to_mdp_state(state[:6]), _to_mdp_state(state[6:12])).item()))  # fmt: skip
                distress["active"] = not recovered
                return False
            # Not currently distressed: ask the ORIGINAL check (same live state, same thresholds
            # as always) or real contact -- if either would have failed, enter distress NOW,
            # forgiving this exact failure instead of ending the episode on it.
            if _real_contact_fail() or _orig_terminal_condition(env_arg):
                distress["active"] = True
                return False
            return False

        _orig_terminal_condition = task._terminal_condition
        task._terminal_condition = _patched_terminal_condition
    # body IDs are stable for the env's lifetime: build_imitation_env's internal first
    # reset() already did the one-time hard reset (world + robot + ref-model construction);
    # every later reset() here is a soft reset (enable_hard_reset defaults to False).
    ref_model = task.get_ref_model()

    # ---- sim-context xi = [friction, 12 per-joint action-space offsets] (plan.md Phase 5.2) ----
    # Friction is fixed for this worker's whole lifetime (mirrors the reference implementation's
    # own xi[:,0], set once in __init__, never touched again in reset()) -- re-apply via
    # changeDynamics on the ground body (lateralFriction is a per-body/link property, not an
    # xi-vector entry PyBullet reads every step, so this one-time call is all that's needed).
    friction = float(rng.uniform(*XI_FRICTION_RANGE))
    pyb.changeDynamics(ground, -1, lateralFriction=friction)
    joint_offset = np.zeros(NUM_JOINTS, dtype=np.float32)

    def resample_joint_offset():
        nonlocal joint_offset
        joint_offset = rng.uniform(*XI_JOINT_OFFSET_RANGE, size=NUM_JOINTS).astype(np.float32)

    def current_xi():
        return np.concatenate(([friction], joint_offset)).astype(np.float32)

    def read_both():
        # `task._get_motion_time()` is the exact continuous reference time ImitationTask used
        # for this state's reward/termination -- the training loop needs it to look up the
        # same instant in its own (vectorized) copy of the reference motion, see motion.py.
        # `task._active_motion_id` indexes into `motion_files` in the same order
        # `_load_ref_motions` iterated them -- the training loop uses it to pick the matching
        # BakedMotion out of its own MultiMotion (see motion.py), needed because multiple
        # motion files makes each of these N worker envs independently sample its own clip.
        return (_read_state(pyb, sim_model) + _read_state(pyb, ref_model)
               + (task._get_motion_time(), task._active_motion_id, current_xi()))  # fmt: skip

    def do_reset():
        env.reset()
        resample_joint_offset()  # re-randomized every episode, mirroring the reference's own xi[:,1:30]
        if adaptive_distress:
            distress["active"] = False  # fresh episode: no hysteresis memory from the PREVIOUS one

    do_reset()  # initial episode's joint offset (the hard reset above only set friction)

    try:
        while True:
            cmd, data = remote.recv()
            if cmd == "reset":
                do_reset()
                remote.send(read_both())
            elif cmd == "step":
                # Read s0 (the state the caller's policy actually acted on) BEFORE stepping --
                # PyBullet's state persists across calls within this process, so this is
                # exactly what self.state() + env.step(a) would read first in a single-process
                # design (matches supermimic_vibed's own TrackingSim.step() ordering).
                ep_step0 = env.env_step_counter
                pre = read_both()
                max_episode_steps = env._max_episode_steps
                # Sanitize before it ever reaches PyBullet: a NaN/Inf torque command (seen in
                # practice from a policy evaluating a far-out-of-distribution observation, e.g.
                # an "on its back" recovery state it was never trained against) can crash
                # PyBullet's native solver outright -- no Python exception, just a dead worker
                # process -- rather than raising a catchable error. Clipping/zeroing here is
                # cheap insurance for every step, not just recovery ones.
                action = np.nan_to_num(np.asarray(data, dtype=np.float32), nan=0.0, posinf=1.0, neginf=-1.0)
                # Apply this episode's joint-offset bias to the commanded action (simulating a
                # per-joint actuator calibration error) BEFORE it reaches the real sim -- the
                # policy/world-model only ever see the clean `action` (stored in the buffer) and
                # the resulting real dynamics, same as the reference's own offset-on-the-target
                # pattern, just applied to our torque-control action instead of a position target.
                effective_action = np.clip(action + joint_offset, -1.0, 1.0)
                # Which condition picks the wide-range/no-hard-fail treatment: the original
                # file-identity check (task._is_recovery_motion(), untouched), OR'd with the
                # general measured-distress flag when --adaptive_distress is on (the `distress`
                # dict and its hook into task._terminal_condition are only set up at all when the
                # flag is on, see above). `distressed_this_step` is what gets reported back for
                # this transition (the value ENTERING this step, used for THIS action) -- the
                # patched _terminal_condition may still flip `distress["active"]` further during
                # env.step() below (that's for the NEXT step's action, not this one).
                distressed_this_step = task._is_recovery_motion() or (adaptive_distress and distress["active"])
                if is_position_mode and distressed_this_step:
                    # Recovery/distress needs the robot's FULL real joint range (see mdp.py's
                    # RECOVERY_Q_* module note) to even represent a self-righting push-off,
                    # not the narrow ordinary-gait band below.
                    sim_action = recovery_center_np + effective_action * recovery_half_range_np
                elif is_position_mode:
                    sim_action = default_q_np + effective_action * ACTION_POS_SCALE
                else:
                    sim_action = effective_action
                _, real_reward, done, info = env.step(sim_action)
                # Pure step-count timeout, distinct from a real task-level failure -- read BEFORE
                # do_reset() resets the counter. env_step_counter is post-increment at this point
                # (ImitationWrapperEnv.step() increments it before returning), so ">=" correctly
                # catches "this step was the one that hit the cap" the same way the wrapper's own
                # `done |= (self.env_step_counter >= self._max_episode_steps)` does.
                timeout = env.env_step_counter >= max_episode_steps
                post_step = read_both()  # s1: the outcome of this action
                # Belt-and-suspenders: even a sanitized action can, through the physics solver
                # itself (e.g. a stiff contact response at an unusual pose), drive the sim state
                # to NaN/Inf without crashing outright -- force a reset rather than let that
                # corrupted state feed into the next step (where IT would be the thing that
                # finally crashes the worker). Not a timeout, so it still counts as `term` below.
                if not all(np.all(np.isfinite(x)) for x in post_step[:6]):
                    done = True  # a real, unconditional failure -- bypasses distress entirely,
                    # same as it always has; `distress["active"]` was already updated (if at all)
                    # live, INSIDE the env.step() call above via the patched _terminal_condition.
                # `term`: a REAL failure (hard task termination, or the NaN-safety override
                # above) as opposed to merely running out of the episode-length-curriculum's
                # step budget -- see env.py's _update_bins for why this distinction matters for
                # reset-bin sampling's return estimate (ported from blinkmimic's term/boot split).
                term = bool(done) and not timeout
                if done:
                    do_reset()
                post = read_both()  # state to continue collecting from next call
                ep_step_post = env.env_step_counter  # 0 if just reset, else ep_step0 + 1
                remote.send((pre, post_step, float(real_reward), bool(done), post, ep_step0, ep_step_post, term,
                            distressed_this_step))  # fmt: skip
            elif cmd == "set_bin_probs":
                task.set_bin_probs(data)
                remote.send(None)
            elif cmd == "render":
                remote.send(env.render(mode="rgb_array"))
            elif cmd == "close":
                env.close()
                remote.close()
                break
            else:
                raise NotImplementedError(cmd)
    except KeyboardInterrupt:
        env.close()


def _stack_states(rows, device, dtype) -> State:
    """rows: list of (root_pos, root_rot, lin_vel, ang_vel, joint_pos, joint_vel) numpy tuples."""
    cols = list(zip(*rows))
    t = lambda a: torch.as_tensor(np.stack(a), device=device, dtype=dtype)
    return State(root_pos=t(cols[0]), root_rot=t(cols[1]), root_lin_vel=t(cols[2]),
                root_ang_vel=t(cols[3]), joint_pos=t(cols[4]), joint_vel=t(cols[5]))


class TrackingSim:
    """N parallel real OD-robot rollouts, one PyBullet process each."""

    def __init__(self, motion_files, num_envs, robot_class=None, enable_randomizer=False,
                seed=0, device="cpu", dtype=torch.float32,
                recovery_motion_names=frozenset(), recovery_init_prob=0.0,
                recovery_sample_prob=0.0,
                bin_duration=None, gamma=0.99, bin_return_decay=0.8,
                bin_softmax_sharpness=25.0, bin_push_every=10, motor_control_mode=None,
                adaptive_distress=False):
        if robot_class is None:
            from quadruped_envs import od
            robot_class = od.OD

        self.num_envs = num_envs
        self.device = device
        self.dtype = dtype
        self.ep_step = np.zeros(num_envs, dtype=np.int64)

        ctx = mp.get_context("spawn")
        self._remotes, work_remotes = zip(*[ctx.Pipe() for _ in range(num_envs)])
        self._procs = [
            ctx.Process(target=_worker, daemon=True,
                       args=(work_remotes[i], motion_files, robot_class, enable_randomizer, seed + i,
                            recovery_motion_names, recovery_init_prob, recovery_sample_prob,
                            bin_duration, motor_control_mode, adaptive_distress))
            for i in range(num_envs)
        ]
        for p in self._procs:
            p.start()

        # ---- return-based reset-bin sampling (SuperMimic paper Appendix D.1) ----
        self._bin_enabled = bin_duration is not None
        if self._bin_enabled:
            from motion_imitation.utilities import motion_data
            from quadruped_envs.imitation_wrapper_env.imitation_task import bin_index_for, bin_offsets, build_bin_layout
            self._bin_index_for = bin_index_for
            durations = [motion_data.MotionData(f).get_duration() for f in motion_files]
            self._bin_duration = bin_duration
            self._bin_offsets = bin_offsets(durations, bin_duration)
            num_bins = sum(n for _, n in self._bin_offsets)
            self._gamma = gamma
            self._bin_return_decay = bin_return_decay
            self._bin_sharpness = bin_softmax_sharpness
            self._bin_push_every = bin_push_every
            self._g_max = MAX_STEP_REWARD / (1.0 - gamma)
            self.bin_G = np.zeros(num_bins)
            # Cumulative sum of squared softmax-focus, float64 (official SuperMimic repo,
            # src/supermimic/runners/reset_sampling.py's ReturnSampler._distribution -- ported
            # 2026-11-11 after confirming our EMA-based keep term, bin_w_avg, was our own
            # invention with no official precedent). Unlike an EMA (which forgets old state),
            # this never decays: a bin that got heavy, CONCENTRATED focus while it was hard
            # keeps earning "keep" share in proportion to effort/effort.sum() even after its
            # value improves and current focus moves elsewhere -- durable protection against
            # RE-forgetting something hard-won, not just against one bin starving the rest (that
            # part is still mainly the 50% uniform floor below, present in both versions).
            # float64 so small per-bin terms stay exact after many thousands of accumulations.
            self.bin_effort = np.zeros(num_bins, dtype=np.float64)
            self.start_bin = np.zeros(num_envs, dtype=np.int64)
            self.ep_ret = np.zeros(num_envs)
            self.ep_disc_ret = np.zeros(num_envs)
            self._steps_since_bin_push = 0

            # Recovery's bins are excluded from bin-sampling's return-weighted softmax entirely
            # (see imitation_task.py's recovery_sample_prob docstring for why: its persistently
            # low return would otherwise make the softmax assign it an ever-growing share of
            # resets as other gaits improve, eventually starving the shared world model/policy
            # of everything else -- confirmed in practice, 2026-11-08/09). Recovery instead gets
            # a separate FIXED reset rate (recovery_sample_prob, applied directly in
            # ImitationTask._reset_ref_motion, bypassing this probability array for those
            # resets) -- this mask just makes sure _push_bin_probs never ALSO hands recovery
            # resets out through the bin-sampling path on top of that fixed share.
            recovery_motion_idx = {i for i, f in enumerate(motion_files)
                                   if os.path.basename(f) in recovery_motion_names}  # fmt: skip
            self._recovery_bin_mask = np.zeros(num_bins, dtype=bool)
            if recovery_motion_idx:
                for motion_idx, (offset, n_bins) in enumerate(self._bin_offsets):
                    if motion_idx in recovery_motion_idx:
                        self._recovery_bin_mask[offset:offset + n_bins] = True

    def reset(self) -> State:
        for remote in self._remotes:
            remote.send(("reset", None))
        rows = [remote.recv() for remote in self._remotes]  # each: 15-tuple (sim 6 + ref 6 + t + motion_id + xi)
        self.ep_step[:] = 0
        sim = _stack_states([row[:6] for row in rows], self.device, self.dtype)
        t = torch.tensor([row[12] for row in rows], device=self.device, dtype=self.dtype)
        motion_id = torch.tensor([row[13] for row in rows], device=self.device, dtype=torch.long)
        xi = torch.as_tensor(np.stack([row[14] for row in rows]), device=self.device, dtype=self.dtype)
        return sim, t, motion_id, xi

    def step(self, actions: torch.Tensor):
        """actions: [num_envs, 12]. Returns both the state the actions were actually chosen
        from (s0: "sim"/"ref"/"t") and the resulting state (s1: "sim1"/"ref1"/"t1"), plus
        reward/done/ep_step (all at s0) and the post-auto-reset state/time to continue
        collecting from next call -- matching supermimic_vibed's own TrackingSim.step()
        convention (self.state() read before AND after stepping), so an episode boundary
        doesn't require the caller to special-case anything."""

        def unpack(rows):
            sim = _stack_states([r[:6] for r in rows], self.device, self.dtype)
            ref = _stack_states([r[6:12] for r in rows], self.device, self.dtype)
            t = torch.tensor([r[12] for r in rows], device=self.device, dtype=self.dtype)
            motion_id = torch.tensor([r[13] for r in rows], device=self.device, dtype=torch.long)
            xi = torch.as_tensor(np.stack([r[14] for r in rows]), device=self.device, dtype=self.dtype)
            return sim, ref, t, motion_id, xi

        actions_np = actions.detach().cpu().numpy()
        for remote, a in zip(self._remotes, actions_np):
            remote.send(("step", a))
        results = [remote.recv() for remote in self._remotes]

        sim, ref, t, motion_id, xi = unpack([r[0] for r in results])
        sim1, ref1, t1, _motion_id1, _xi1 = unpack([r[1] for r in results])
        reward = torch.tensor([r[2] for r in results], device=self.device, dtype=self.dtype)
        done = torch.tensor([r[3] for r in results], device=self.device, dtype=torch.bool)
        next_sim, next_ref, next_t, next_motion_id, next_xi = unpack([r[4] for r in results])
        ep_step0 = torch.tensor([r[5] for r in results], dtype=torch.long)
        self.ep_step = np.array([r[6] for r in results])  # post-step (post-reset if done), as before

        if self._bin_enabled:
            term = np.array([r[7] for r in results], dtype=bool)
            self._update_bins(t.cpu().numpy(), motion_id.cpu().numpy(), ep_step0.cpu().numpy(),
                              reward.cpu().numpy(), done.cpu().numpy(), term)

        # False for every row unless --adaptive_distress is on (see env.py's _worker module
        # note) -- harmless, unused placeholder in that case, same as every other run's data.
        distressed = torch.tensor([r[8] for r in results], device=self.device, dtype=torch.bool)

        return {
            "sim": sim, "ref": ref, "t": t, "motion_id": motion_id, "xi": xi,
            "sim1": sim1, "ref1": ref1, "t1": t1,
            "reward": reward, "done": done, "ep_step": ep_step0, "distressed": distressed,
            "next_sim": next_sim, "next_ref": next_ref, "next_t": next_t, "next_motion_id": next_motion_id,
            "next_xi": next_xi,
        }

    def _update_bins(self, t, motion_id, ep_step0, reward, done, term):
        """Return-based reset-bin bookkeeping (SuperMimic paper Appendix D.1), ported from the
        original reference implementation's TrackingSim.step() (supermimic_vibed/supermimic/
        env.py:209-229).

        History: a first attempt here used the plain accumulated-reward SUM as G for every
        episode regardless of outcome, which scales with episode length -- since almost every
        episode in this project runs to the same ~fixed curriculum length regardless of policy
        quality (unlike the original, where a bad policy's episode is genuinely SHORT), that
        made G differences between a "good" and "bad" bin enormous (observed: pace_run12
        catastrophically forgetting its converged gaits within ~1000 iterations, world-model
        loss climbing from ~0.15 to 16+ as data collection collapsed onto gallop/recovery almost
        exclusively). A second attempt replaced it with a length-invariant
        `mean_reward/(1-gamma)` estimate for every episode -- stable, but discarded the actual
        per-step discounted trajectory entirely and treated a real hard failure identically to a
        healthy episode that simply ran out of curriculum step budget.

        Current version (2026-11-08, re-derived from `blinkmimic` -- a more mature SuperMimic
        follow-up project -- after rereading its own TrackingSim; see its env.py's `term`/`boot`
        split): properly distinguishes the two cases via `term` (a REAL task failure, passed in
        from env.py's worker) vs. a mere curriculum-timeout:
          G = ep_disc_ret + boot
          boot = 0                                          if term (real failure: no credit
                                                               for a future that didn't happen)
          boot = (gamma**T) * (ep_ret/T) / (1-gamma)         otherwise (steady-state continuation
                                                               value, discounted in from step T)
        `ep_disc_ret` is the properly gamma-discounted sum of REAL reward actually earned this
        episode (not just a plain sum); `ep_ret/T` is the episode's own mean reward, used only
        as the post-episode continuation ESTIMATE, not as the reported return itself -- unlike
        the previous version, which used mean_reward/(1-gamma) even for real failures. `G_max`
        (`self._g_max`, used by `_push_bin_probs`'s softmax) is similarly now the paper's own
        `MAX_STEP_REWARD/(1-gamma)` theoretical ceiling (mdp.py) rather than an assumed round
        number.
        """
        fresh = ep_step0 == 0
        if fresh.any():
            for i in np.nonzero(fresh)[0]:
                self.start_bin[i] = self._bin_index_for(self._bin_offsets, int(motion_id[i]), float(t[i]), self._bin_duration)
            self.ep_ret[fresh] = 0.0
            self.ep_disc_ret[fresh] = 0.0

        self.ep_disc_ret += (self._gamma ** ep_step0) * reward
        self.ep_ret += reward

        done_idx = np.nonzero(done)[0]
        if len(done_idx) > 0:
            T = ep_step0[done_idx] + 1  # steps actually taken this episode (ep_step0 is 0-indexed)
            boot = np.where(term[done_idx], 0.0,
                           (self._gamma**T) * (self.ep_ret[done_idx] / T) / (1.0 - self._gamma))  # fmt: skip
            G = self.ep_disc_ret[done_idx] + boot
            # Looped (not vectorized fancy-indexing) so two envs finishing in the same bin on the
            # same step both actually update bin_G -- numpy's `arr[dup_idx] = values` silently
            # drops all but the last write for a repeated index, which plain assignment here
            # would hit at num_envs=16 vs. a few dozen bins often enough to matter.
            for idx, i in enumerate(done_idx):
                b = self.start_bin[i]
                self.bin_G[b] = self._bin_return_decay * self.bin_G[b] + (1 - self._bin_return_decay) * G[idx]

        self._steps_since_bin_push += 1
        if done.any() and self._steps_since_bin_push >= self._bin_push_every:
            self._push_bin_probs()
            self._steps_since_bin_push = 0

    def _push_bin_probs(self):
        soft = _softmax(-self._bin_sharpness * self.bin_G / self._g_max)
        self.bin_effort += soft.astype(np.float64) ** 2
        keep = (self.bin_effort / self.bin_effort.sum()).astype(soft.dtype)
        num_bins = len(self.bin_G)
        probs = 0.5 / num_bins + 0.4 * soft + 0.1 * keep
        if self._recovery_bin_mask.any():
            # Recovery gets its reset share entirely through the fixed recovery_sample_prob
            # path instead (see __init__'s note) -- zero its bins out here so this return-
            # weighted distribution never ALSO hands it resets on top of that.
            probs = np.where(self._recovery_bin_mask, 0.0, probs)
        probs = probs / probs.sum()  # guard against float drift
        for remote in self._remotes:
            remote.send(("set_bin_probs", probs))
        for remote in self._remotes:
            remote.recv()

    def render_frames(self) -> np.ndarray:
        """One RGB frame per env (debugging/visualization only -- not used in training)."""
        for remote in self._remotes:
            remote.send(("render", None))
        return np.stack([remote.recv() for remote in self._remotes])

    def close(self):
        for remote in self._remotes:
            remote.send(("close", None))
        for p in self._procs:
            p.join(timeout=5)
