"""SuperMimic training for the OD quadruped (Algorithm A.1): collect real transitions -> fit
the world model -> BPTT policy + critic update over imagined rollouts. Ported from
supermimic_vibed/supermimic/train.py; see the module docstrings in mdp.py / networks.py /
env.py / motion.py / buffer.py for exactly what changed and why -- in short: no Isaac Lab (a
small multiprocess PyBullet collector instead, env.py), a simplified sim-context vector xi
(per-worker ground friction + per-episode joint action-space offset -- CoM-offset
randomization dropped, PyBullet has no direct API for it; see mdp.py's module note, Phase 5.2),
continuous reference time instead of an integer frame counter (the real env's control dt
doesn't evenly divide the motion file's frame duration), this project's own already-validated
reward instead of BeyondMimic's formula, and CPU-first (no torch.compile, no CUDA autocast --
see the plan for why: MPS has gaps for some of the linalg/quaternion ops used here, and at
these batch sizes CPU plain fp32 is the simpler, safer starting point).
"""
import argparse
import json
import math
import os
import re
import signal
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--num_envs", type=int, default=8)
    p.add_argument("--iterations", type=int, default=20_000)
    p.add_argument("--buffer_steps", type=int, default=1000)
    p.add_argument("--utd", type=float, default=0.4, help="updates per real env step")
    p.add_argument("--horizon", type=int, default=16, help="imagined BPTT horizon H (end-of-schedule value)")
    p.add_argument("--horizon_start", type=int, default=None,
                  help="H at iteration 0, linearly grown to --horizon over --lr_decay_iters (default: "
                       "half of --horizon, rounded down, min 4 -- short horizons are cheaper and easier "
                       "credit assignment early, before the world model/policy are any good; set equal "
                       "to --horizon to disable the schedule)")  # fmt: skip
    p.add_argument("--goal_frames", type=int, default=10, help="K future reference frames in the goal")
    p.add_argument("--hist", type=int, default=10, help="observation history length")
    p.add_argument("--wm_window", type=int, default=8)
    p.add_argument("--wm_batches", type=int, default=4)
    p.add_argument("--wm_batch", type=int, default=256)
    p.add_argument("--pi_batch", type=int, default=256)
    p.add_argument("--lr", type=float, default=2e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--lam", type=float, default=0.95)
    # 0.1 is SuperMimic's own default, tuned for its (differently-scaled) action convention.
    # THIS env's action in [-1, 1] maps to real torque via ODMotorModel's TORQUE_MULTIPLIER=50
    # (confirmed by reading od_motor.py's TORQUE-mode branch: applied_torque = 50 * action),
    # so action_noise=0.1 means +/-5 N*m of per-joint random torque noise EVERY step. Static
    # standing only needs ~4.5-14 N*m per joint (computed from the robot's actual mass/leg
    # Jacobian, 4-leg vs. 2-leg stance) -- 5 N*m of noise is the same order of magnitude as the
    # control torque itself, likely enough on its own to prevent the policy from ever
    # experiencing a quiet, balanced stance to build a gait from. 0.02 (~1 N*m std) is a more
    # proportionate starting point for pace_run1's successor run; not yet validated.
    p.add_argument("--action_noise", type=float, default=0.02,
                  help="action_noise floor/end-of-schedule value, see --action_noise_start")
    p.add_argument("--action_noise_start", type=float, default=None,
                  help="action_noise at iteration 0, cosine-decayed to --action_noise over "
                       "--lr_decay_iters, same schedule shape as the LR decay (default: 2.5x "
                       "--action_noise -- more exploration early, settling to the validated floor; "
                       "set equal to --action_noise to disable the schedule)")  # fmt: skip
    p.add_argument("--critic_epochs", type=int, default=2)
    p.add_argument("--critic_minibatches", type=int, default=2)
    p.add_argument("--no_critic", action="store_true")
    p.add_argument("--critic_lr_scale", type=float, default=0.3,
                   help="critic's own learning rate = --lr * this scale, applied independently "
                        "of wm/pi's shared lr_at() schedule. Part of the Phase 5.1 critic-"
                        "instability fix (see project memory): the critic's own TD-bootstrapped "
                        "targets depend on its own just-updated predictions (the 'deadly triad' "
                        "-- function approximation + bootstrapping + a shifting data "
                        "distribution), so a transient bias from a hard-to-predict state (e.g. "
                        "gallop, which the world model fits worse than other gaits) can compound "
                        "through repeated updates; a slower-moving critic damps how fast that "
                        "compounding can run away. 0.3 is a conservative starting point, not yet "
                        "empirically tuned beyond the controlled stress test used to validate "
                        "this fix exists for a reason at all.")
    p.add_argument("--critic_loss", type=str, default="huber", choices=["mse", "huber"],
                   help="huber (default, see F.huber_loss): caps the GRADIENT from any single "
                        "outlier GAE target to a constant magnitude instead of growing "
                        "proportionally with the error (MSE's failure mode) -- directly targets "
                        "the outlier-amplification mechanism behind Phase 5.1's critic "
                        "divergence (a hard-to-predict state's target lands far from the "
                        "critic's current estimate more often than normal, and under MSE that "
                        "outlier injects a proportionally huge gradient). 'mse' restores the "
                        "original pre-fix behavior, for an ablation/comparison run.")
    p.add_argument("--critic_huber_delta", type=float, default=5.0,
                   help="transition point between MSE-like (small-error) and linear-capped "
                        "(large-error) behavior for --critic_loss=huber, in value-target units "
                        "(roughly discounted-return scale, not per-step reward scale).")
    p.add_argument("--critic_value_clip", type=float, default=100.0,
                   help="hard clip applied to every critic prediction used as a VALUE (the "
                        "policy's bootstrap term and the GAE recursion's V(s_t+1)) and to the "
                        "resulting GAE regression targets themselves -- a safety net, not a "
                        "learned parameter. Derived from the reward scale: per-step reward is "
                        "roughly <=1, so the theoretical max discounted infinite-horizon return "
                        "at gamma=0.99 is 1/(1-0.99)=100 -- this value should essentially never "
                        "bind for a healthy critic, only cap a genuine runaway before it can "
                        "reach NaN/inf.")
    p.add_argument("--critic_tau", type=float, default=0.01,
                   help="Polyak-averaging rate for the target critic (critic_target_params = "
                        "tau*critic_params + (1-tau)*critic_target_params, applied once per "
                        "policy_update() call after that call's critic gradient steps). Not in "
                        "the original reference implementation -- added after observing real "
                        "critic-loss divergence in this port (see project memory). Smaller = "
                        "slower-moving, more stable target but slower to track real improvement; "
                        "0.01 is a conservative DDPG/SAC-typical starting point, not yet tuned "
                        "for this project specifically.")
    p.add_argument("--learned_termination", action="store_true",
                   help="use a learned classifier head on the WorldModel (paper's d_hat_t = "
                        "g_omega(s_hat_t, g_t)) instead of compute_soft_termination_prob's "
                        "hand-tuned analytic sigmoid, once the head's own validation accuracy "
                        "clears --term_warmup_acc. Trained every wm_update() via BCE against "
                        "the real analytic hard-termination label (compute_termination) on real "
                        "buffer transitions -- the LABEL stays analytic/authoritative, only the "
                        "thing used inside imagined rollouts becomes learned. Off by default; "
                        "additive (an old checkpoint's missing term_head loads fine via "
                        "strict=False) -- see networks.py's WorldModel docstring.")
    p.add_argument("--term_warmup_acc", type=float, default=0.95,
                   help="learned termination head's own balanced accuracy (on its own BCE "
                        "training batches) must clear this before its output starts being used "
                        "in reward_term() at all -- below it, the analytic "
                        "compute_soft_termination_prob is used unchanged, so an undertrained "
                        "head's garbage output can't poison the policy gradient early on.")
    p.add_argument("--term_blend_iters", type=int, default=2000,
                   help="once the accuracy gate clears, linearly blend from the analytic "
                        "formula to the learned head's output over this many iterations (0 = "
                        "switch instantly) rather than an abrupt swap.")
    p.add_argument("--wm_diag_every", type=int, default=0,
                   help="if >0, every N iterations dump per-transition world-model loss (tagged "
                        "by motion_id/is_recovery/is_immediately_post_reset/reward) to "
                        "<log_dir>/wm_diag.csv -- diagnostic only (plan.md Phase 5.1), off by "
                        "default (zero overhead when 0).")
    p.add_argument("--warmup_steps", type=int, default=64, help="real env steps before any update")
    p.add_argument("--wm_only_iters", type=int, default=200, help="world-model-only iterations at the start")
    p.add_argument("--lr_decay_iters", type=int, default=15_000)
    p.add_argument("--ckpt_every", type=int, default=200)
    p.add_argument("--ckpt_keep_recent", type=int, default=20,
                   help="always keep the most recent N numbered ckpt_NNNNNN.pt saves (fine-"
                        "grained rollback right after a bad change), on top of whatever "
                        "--ckpt_milestone_every keeps long-term. Older, non-milestone ones are "
                        "deleted automatically after each save -- otherwise they accumulate "
                        "forever and can fill the disk on a long run (confirmed: one run hit "
                        "90GB/1468 files on its own, three since-retired ones left ~325GB behind "
                        "before this was caught and the resulting ENOSPC crashed training).")
    p.add_argument("--ckpt_milestone_every", type=int, default=1000,
                   help="keep ckpt_NNNNNN.pt forever (never pruned) whenever N is a multiple of "
                        "this -- coarse long-term history alongside --ckpt_keep_recent's rolling "
                        "window. 0 disables milestones (only the recent window is kept).")
    p.add_argument("--log_every", type=int, default=20)
    p.add_argument("--motion_file", type=str,
                   default=os.path.join(REPO, "quadruped_envs", "data", "motions", "od", "pace.txt"),
                   help="comma-separated list of motion files to train on jointly -- each of the "
                        "--num_envs real envs independently samples one per episode reset (ImitationTask's "
                        "own existing multi-motion support), and one shared world model/policy trains "
                        "across all of them (see motion.py's MultiMotion)")  # fmt: skip
    p.add_argument("--log_dir", type=str,
                   default=os.path.join(REPO, "quadruped_envs", "supermimic", "logs", "run"))
    p.add_argument("--recovery_motion", type=str, default=None,
                   help="path to one of --motion_file's entries (e.g. recovery.txt, a copy of "
                        "idle.txt) to train as a reference-free 'get up from being on its back' "
                        "task: episodes sampling this motion sometimes start from a scripted "
                        "fallen-on-its-back pose instead of a normal reference-tracked reset, "
                        "and skip the usual fall-termination checks (both expected for this "
                        "task, not failures), while still being rewarded toward this motion's "
                        "own (e.g. standing-still) reference -- see imitation_task.py's "
                        "recovery_motion_names/ImitationTask.done(). No scripted recovery "
                        "trajectory is provided; the policy has to find its own way up.")
    p.add_argument("--recovery_init_prob", type=float, default=0.5,
                   help="probability, when --recovery_motion is sampled, of actually starting "
                        "from the fallen pose (vs. a normal reset for that same motion)")
    p.add_argument("--recovery_sample_prob", type=float, default=0.0,
                   help="fixed probability that a reset is forced onto --recovery_motion, "
                        "independent of --bin_duration's return-weighted bin sampling (which, "
                        "left alone, would otherwise keep handing recovery a growing share of "
                        "resets as its return stays persistently lower than converged gaits' -- "
                        "see imitation_task.py's recovery_sample_prob docstring; this is why "
                        "recovery.txt was dropped from the training mix entirely before. 0.0 "
                        "(default): no protection, recovery (if present) competes in the normal "
                        "bin-sampling pool like any other motion. ~0.08-0.12 is a reasonable "
                        "starting point when re-enabling recovery -- meaningful real training "
                        "exposure without letting it dominate.")
    p.add_argument("--adaptive_distress", action="store_true",
                   help="generalize the wide-action-range/no-hard-termination treatment from "
                        "'is this episode tracking --recovery_motion' to 'is tracking error "
                        "ACTUALLY large right now' (see env.py's _worker and mdp.py's "
                        "distress_enter/distress_exit module notes) -- applies during ANY gait, "
                        "not just a dedicated file, so the policy learns to recover from a real "
                        "stumble mid-pace/trot/gallop/etc. and continue the SAME gait, not just "
                        "'stand still'. Hysteresis (enters at the normal hard-fail threshold, "
                        "exits only once genuinely back under control) avoids both flicker and "
                        "the degenerate case of a naive single-threshold check (entry and 'exit' "
                        "being the same condition would disable hard termination for everyone). "
                        "Off by default: zero behavior change -- --recovery_motion's file-based "
                        "check still governs everything exactly as before. Deploy this as its "
                        "own isolated validation run, not on top of an already-live one -- it's "
                        "live for 100%% of resets (just conditionally triggered), not cordoned "
                        "off behind a small --recovery_sample_prob share like the file-based path.")
    p.add_argument("--bin_duration", type=float, default=None,
                   help="enable return-based reset-bin sampling (SuperMimic paper Appendix "
                        "D.1): splits every --motion_file entry's duration into bins this many "
                        "seconds long (1.0 matches the paper), and reset time+motion is sampled "
                        "50%% uniform / 40%% weighted toward bins with low historical return / "
                        "10%% a slowly-decaying average, instead of picking motion+time "
                        "uniformly at random -- concentrates real-env resets (and so buffer "
                        "data / imagined-rollout starts) on whichever (motion, time-range) is "
                        "currently hardest. None (default) keeps the original uniform behavior.")
    p.add_argument("--bin_push_every", type=int, default=10,
                   help="throttles how often updated bin probabilities are pushed to the real "
                        "workers (every N real steps that also saw at least one episode end)")
    p.add_argument("--bin_return_decay", type=float, default=0.8,
                   help="EMA decay for each bin's running discounted-return estimate (paper's "
                        "own value)")
    p.add_argument("--bin_sharpness", type=float, default=5.0,
                   help="softmax sharpness for weighting low-return bins. The paper's own "
                        "value (25) assumes per-step reward is tightly clustered near its max "
                        "across bins (their G1 motions are all broadly learnable); this port's "
                        "reward spans close to the FULL [0,1] range between a converged gait "
                        "(~0.5/step) and an unsolved one like early recovery (~0.02/step), and "
                        "since the softmax exponent reduces to -sharpness*mean_reward_per_step "
                        "(see env.py's _update_bins docstring), sharpness=25 there produced a "
                        "~160000x weight ratio between the best and worst bin -- observed to "
                        "cause real, fast catastrophic forgetting of converged gaits (pace_run12,"
                        " 2026-11-06: wm_loss climbed from ~0.15 to 16+ within ~1000 iterations "
                        "as real-data collection collapsed almost entirely onto the worst bin). "
                        "5.0 is a more conservative starting point (~11x ratio for the same "
                        "reward gap) pending its own short validation run, the same empirical-"
                        "calibration pattern already used for compute_soft_termination_prob's "
                        "sharpness constant -- don't assume it's final without checking.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--resume", type=str, default=None)
    p.add_argument("--device", type=str, default="cpu")
    p.add_argument("--control_mode", type=str, default="position", choices=["torque", "position"],
                   help="'position' (default as of pd_run1, 2026-11-09): the robot's REAL env "
                        "uses PD/position control (od_motor.py's ODMotorModel, already used for "
                        "every run's reset-settle phase regardless of this flag, now also used "
                        "for actual RL stepping) -- trains faster than torque control since the "
                        "PD law provides 'free' stability. 'torque': this project's original "
                        "convention (still supported for backward compat), action in [-1,1] * "
                        "od_motor.py's TORQUE_MULTIPLIER (=25 Nm) is applied as the joint torque "
                        "directly. In 'position' mode, a "
                        "normalized action converts to a joint-position target via "
                        "`default_q + action * ACTION_POS_SCALE` (mdp.py) before reaching the "
                        "real env; the policy/world-model/buffer side is unaffected (still "
                        "normalized [-1,1] units throughout). A REAL dynamics change (not just "
                        "a reward/physics-constant tweak) -- the world model has to learn an "
                        "entirely different action->state-transition mapping, so a checkpoint "
                        "trained under one --control_mode is NOT meaningfully resumable under "
                        "the other (no shape mismatch, loads fine, but the learned dynamics are "
                        "simply wrong) -- start a FRESH run when switching, don't --resume "
                        "across this flag. Motivation: PD/position control is widely reported "
                        "in the legged-RL literature to train faster/more stably than direct "
                        "torque (a PD loop gives bounded, smooth torque response to ANY action "
                        "for free, including a random/untrained one) -- see project memory for "
                        "the full reasoning, confirmed against blinkmimic's own actuator "
                        "convention.")
    p.add_argument("--pi_ref_pd", action="store_true",
                   help="adds a PD-style reference-tracking bias to the Policy's output, "
                        "adapted from blinkmimic (a more mature SuperMimic follow-up project)'s "
                        "constant 'ref_skip' bias for OUR torque-controlled actuator -- see "
                        "networks.py's Policy docstring for why a literal port doesn't make "
                        "sense here. --pi_ref_pd_kp/--pi_ref_pd_kd/--pi_ref_pd_clip tune it. "
                        "Changes the Policy's own forward() computation (not its parameter "
                        "shapes), so an old checkpoint still loads fine either way -- but "
                        "flipping this on a RESUMED run changes what the network was actually "
                        "trained to output on top of, same caveat as any other reward/dynamics "
                        "change made mid-run.")
    p.add_argument("--pi_ref_pd_kp", type=float, default=20.0,
                   help="PD stiffness (Nm/rad) for --pi_ref_pd -- an unvalidated starting guess "
                        "derived from this robot's own static-stance torque budget (~4.5-14 Nm "
                        "across a 4-leg/2-leg stance), not a full dynamics analysis.")
    p.add_argument("--pi_ref_pd_kd", type=float, default=0.5,
                   help="PD damping (Nm per rad/s) for --pi_ref_pd -- see --pi_ref_pd_kp's note.")
    p.add_argument("--pi_ref_pd_clip", type=float, default=0.3,
                   help="hard clamp (in action units, i.e. fraction of the +/-25 Nm torque "
                        "range) on the PD bias itself for --pi_ref_pd -- keeps the baseline "
                        "tracking term a modest nudge, leaving most of the torque range for "
                        "the network's own learned correction.")
    p.add_argument("--optimizer", type=str, default="adam", choices=["adam", "muon"],
                   help="'muon' (ported from blinkmimic, a more mature SuperMimic follow-up "
                        "project): Muon's Newton-Schulz-orthogonalized update on each network's "
                        "HIDDEN MLP weight matrices (every Linear except each MLP's first and "
                        "last), plain Adam on everything else (first/last Linear, all 1D "
                        "params) -- same --lr reused for both via Muon's "
                        "adjust_lr_fn='match_rms_adamw' (Liu et al. 2025), no separate tuning "
                        "needed. 'adam' (default) is the original, unchanged behavior. Switching "
                        "this on a --resume discards that optimizer's saved state (shapes are "
                        "incompatible between the two) and starts it fresh, same cost as any "
                        "other optimizer-state mismatch already handled here. CPU is NOT "
                        "viable for 'muon' -- confirmed directly, a single step measured ~40s "
                        "on CPU (vs ~25-35ms on MPS) for this project's networks; only use "
                        "'muon' with --device mps, including for smoke tests (a CPU one will "
                        "just look hung).")
    p.add_argument("--compile", action="store_true",
                   help="torch.compile() the hot-path tensor computations (WorldModel.step, "
                        "Policy.forward, Critic.forward/critic_target.forward) -- pure "
                        "performance, no change to the math. MPS support for torch.compile's "
                        "default 'inductor' backend is far less mature than CUDA's (or plain "
                        "eager-mode MPS, already used throughout this project) -- validate with "
                        "a short smoke test (same seed, compare losses against an uncompiled run, "
                        "and confirm a checkpoint saved/resumed under this flag round-trips "
                        "cleanly) before trusting it on a live run. Applied AFTER load_state_dict "
                        "on --resume specifically to sidestep any compiled-wrapper state_dict key "
                        "mismatch.")
    return p.parse_args()


def main():
    args = parse_args()
    if args.action_noise_start is None:
        args.action_noise_start = 2.5 * args.action_noise
    if args.horizon_start is None:
        args.horizon_start = max(4, args.horizon // 2)
    import torch
    import torch.nn as nn

    from quadruped_envs.supermimic.buffer import ReplayBuffer
    from quadruped_envs.supermimic.env import TrackingSim
    from quadruped_envs.supermimic.mdp import (
        ACTION_RATE_WEIGHT, CRITIC_EXTRA, INV_DIM, NUM_JOINTS, OBS_DIM,
        action_rate_penalty, add_noise as mdp_add_noise, compute_reward, compute_soft_termination_prob,
        compute_termination, critic_features, flatten_state, mat_state_from, mat_state_to,
        MatState, obs as mdp_obs, recovery_progress_reward, unflatten_state,
    )
    from quadruped_envs.supermimic.motion import CONTROL_DT, GOAL_FRAME_DIM, MultiMotion
    from quadruped_envs.supermimic.networks import Critic, Policy, WorldModel
    from quadruped_envs.supermimic.optim import MuonAdam

    dev = args.device
    torch.manual_seed(args.seed)
    os.makedirs(args.log_dir, exist_ok=True)
    json.dump(vars(args), open(os.path.join(args.log_dir, "args.json"), "w"), indent=2)

    motion_files = [f.strip() for f in args.motion_file.split(",")]
    motion = MultiMotion(motion_files, device=dev)
    recovery_index = motion_files.index(args.recovery_motion) if args.recovery_motion else None
    recovery_motion_names = {os.path.basename(args.recovery_motion)} if args.recovery_motion else frozenset()
    K, Hh, HZ = args.goal_frames, args.hist, args.horizon_start
    GOAL_DIM = GOAL_FRAME_DIM * K
    CRITIC_IN = INV_DIM + CRITIC_EXTRA + GOAL_DIM

    # goal_dim=GOAL_DIM always builds the term_head (cheap: a couple of small extra layers) so
    # --learned_termination can be toggled on a resumed run without needing a fresh checkpoint --
    # the head just goes untrained/unused until the flag turns it on. See networks.py's
    # WorldModel docstring for why this is additive/non-breaking unlike the planned sim-context xi.
    wm = WorldModel(goal_dim=GOAL_DIM).to(dev)
    ref_pd = None
    if args.pi_ref_pd:
        from quadruped_envs.imitation_wrapper_env.od_motor import TORQUE_MULTIPLIER
        ref_pd = (args.pi_ref_pd_kp, args.pi_ref_pd_kd, float(TORQUE_MULTIPLIER), args.pi_ref_pd_clip)
    pi = Policy(OBS_DIM, GOAL_DIM, hist=Hh, ref_pd=ref_pd).to(dev)
    critic = Critic(CRITIC_IN).to(dev)

    def make_optimizer(model, mlps, lr, betas=(0.9, 0.999)):
        if args.optimizer == "muon":
            return MuonAdam(model, mlps, lr=lr, betas=betas)
        return torch.optim.Adam(model.parameters(), lr=lr, betas=betas)

    opt_wm = make_optimizer(wm, [wm.net, wm.term_head], args.lr)
    opt_pi = make_optimizer(pi, [pi.enc, pi.dec], args.lr, betas=(0.9, 0.95))
    opt_v = make_optimizer(critic, [critic.net], args.lr)
    start_it, wall_offset, sim_steps = 0, 0.0, 0
    if args.resume:
        ck = torch.load(args.resume, map_location=dev)
        # strict=False: an old checkpoint predating term_head is missing those params -- they
        # just stay at their fresh random init, harmless until --learned_termination is passed.
        wm.load_state_dict(ck["wm"], strict=False)
        # strict=False: an old checkpoint predating --pi_ref_pd is missing the _ref_pd_default_q
        # buffer -- it just stays at its freshly-constructed value (DEFAULT_Q, a fixed, not
        # learned, constant), harmless since there's nothing to "warm up" there, unlike an
        # optimizer's momentum state. Same reasoning as wm.load_state_dict's own strict=False
        # a few lines up, just for a different additive feature.
        pi.load_state_dict(ck["pi"], strict=False)
        critic.load_state_dict(ck["critic"])

        def load_opt(opt, key, name):
            # Falls back to the freshly-constructed optimizer (just-initialized momentum for
            # everything, including params that WERE in the checkpoint) rather than crashing,
            # whenever the saved optimizer state doesn't match the current one's structure --
            # e.g. an old checkpoint predating term_head (opt_wm), or switching --optimizer
            # between adam and muon across a resume (incompatible state_dict shapes either
            # way). Same acceptable "re-warms over ~1000 iterations" cost already seen for
            # critic/opt_v after the earlier manual checkpoint-reconstruction incidents.
            try:
                opt.load_state_dict(ck[key])
            except (ValueError, KeyError) as e:
                print(f"[resume] {name} state incompatible with the current model/optimizer "
                      f"shape -- starting {name} fresh: {e}", flush=True)

        load_opt(opt_wm, "opt_wm", "opt_wm")
        load_opt(opt_pi, "opt_pi", "opt_pi")
        load_opt(opt_v, "opt_v", "opt_v")
        start_it, wall_offset, sim_steps = ck["iteration"], ck["wall_time"], ck["sim_steps"]

    # Polyak-averaged target critic (NOT in the original supermimic_vibed reference -- checked
    # directly, its train.py uses the one, continuously-updated critic for both bootstrapping
    # and its own regression target, same as this port originally did). Added here specifically
    # because that naive setup (a known "deadly triad" instability: function approximation +
    # bootstrapping + on-policy-ish sampling) was observed to genuinely diverge in this port --
    # critic loss climbing unboundedly (19->165 within ~100 iterations) shortly after the
    # recovery task's reward-shaping term shifted part of the reward distribution's scale, see
    # project memory. critic_target is used everywhere the original used the live `critic` to
    # produce a VALUE used as a bootstrap/target (the policy's J bootstrap, and the GAE target
    # computation's V) -- never trained by backprop, only Polyak-updated after each critic
    # update step, so it changes slowly and can't chase its own just-updated self. The live
    # `critic` is still what opt_v actually trains, and is still what gets checkpointed/resumed
    # as the "real" critic -- critic_target is reconstructed from it (tau=1 equivalent) on every
    # --resume, including from old checkpoints that predate this change.
    critic_target = Critic(CRITIC_IN).to(dev)
    critic_target.load_state_dict(critic.state_dict())
    critic_target.norm = critic.norm  # share, not copy -- input normalization isn't a "target" we want lagging
    for p in critic_target.parameters():
        p.requires_grad_(False)

    if args.compile:
        # Applied last, after all load_state_dict calls above -- state_dict()/load_state_dict()
        # on an OptimizedModule forward transparently to the wrapped module in current PyTorch,
        # but compiling only ONCE everything is already loaded sidesteps the question entirely
        # rather than relying on that. wm.step (not wm itself -- WorldModel has no forward(), the
        # hot path is the custom .step() method) is compiled as a bound method; pi/critic/
        # critic_target are plain nn.Module.forward hot paths, compiled as whole modules.
        print("[compile] torch.compile() enabled for wm.step / pi / critic / critic_target", flush=True)
        wm.step = torch.compile(wm.step)
        pi = torch.compile(pi)
        critic = torch.compile(critic)
        critic_target = torch.compile(critic_target)

    # Per-component floors on the world-model error normalization (pos, rot, lin vel, ang vel,
    # q, qd increments), so a component that's gone to ~0 variance doesn't blow up its own
    # normalized error term.
    ERR_FLOOR = torch.cat((
        torch.full((3,), 3e-3), torch.full((3,), 5e-3), torch.full((3,), 2e-2), torch.full((3,), 5e-2),
        torch.full((NUM_JOINTS,), 3e-3), torch.full((NUM_JOINTS,), 5e-2),
    )).to(dev)  # fmt: skip

    def wm_rollout(s, a, xi):
        S = unflatten_state(s[:, 0])
        preds = []
        for j in range(args.wm_window):
            S = wm.step(S, a[:, j], xi)  # xi is constant for this row across the whole window
            preds.append(S)
        return tuple(torch.stack([P[i] for P in preds], 1) for i in range(6))  # (B, k, ...)

    def wm_err(S_hat, s1, mask):
        std = torch.maximum(wm.out_norm.std(), ERR_FLOOR)
        e = (wm.state_error(S_hat, unflatten_state(s1)) / std) ** 2  # (B, k, D)
        e = e.mean(-1)  # (B, k), per-transition -- returned as-is for optional diagnostic use
        return (e * mask).sum() / s1.shape[0], e[:, -1], e

    def imag_step(S, o_hist, a_prev, t, g, motion_idx, xi):
        a = torch.clamp(pi(o_hist, g), -1.0, 1.0)
        S1 = wm.step(S, a, xi)  # xi is constant for this row across the whole imagined horizon
        t1 = t + CONTROL_DT
        ref1 = motion.frame_state(motion_idx, t1)
        o1 = mdp_add_noise(mdp_obs(S1, a, ref1.R, xi))
        o_hist = torch.cat((o_hist[:, 1:], o1.unsqueeze(1)), 1)
        return S1, a, o_hist, t1, motion.goal(motion_idx, t1, K, CONTROL_DT)

    # Learned-termination gating state (plan.md Phase 5.2): the head is trained every wm_update()
    # regardless, but its output isn't USED in reward_term until its own BCE-batch accuracy
    # clears term_warmup_acc -- an untrained head's garbage prediction must not poison the
    # policy gradient early on. Once cleared, blend linearly from the analytic formula to the
    # learned head over term_blend_iters rather than an abrupt swap. Plain dict (not a bare
    # variable) so the closures below can mutate it.
    term_state = {"acc_ema": 0.0, "cleared_it": None}

    def update_term_state(it, term_acc):
        if term_acc is None:
            return
        term_state["acc_ema"] = 0.98 * term_state["acc_ema"] + 0.02 * term_acc
        if term_state["cleared_it"] is None and term_state["acc_ema"] >= args.term_warmup_acc:
            term_state["cleared_it"] = it
            print(f"[learned_termination] accuracy gate cleared at it={it} (ema={term_state['acc_ema']:.4f}) "
                 f"-- blending in over the next {args.term_blend_iters} iterations", flush=True)  # fmt: skip

    def learned_term_blend(it):
        if not args.learned_termination or term_state["cleared_it"] is None or it is None:
            return 0.0
        if args.term_blend_iters <= 0:
            return 1.0
        return min(1.0, (it - term_state["cleared_it"]) / args.term_blend_iters)

    def reward_term(S, ref, a, a_prev, is_recovery=None, is_scripted_recovery=None, g=None, it=None):
        r = compute_reward(mat_state_to(S), mat_state_to(ref))["total"]
        r = r - ACTION_RATE_WEIGHT * action_rate_penalty(a, a_prev)
        # is_scripted_recovery (file-based ONLY, not generic distress) deliberately gates the
        # dense shaping BONUS -- is_recovery (file-based OR'd with --adaptive_distress's generic
        # distress) still gates termination-forgiveness just below. Learned the hard way
        # (2026-11-12): extending the bonus to generic distress too let a validation run's policy
        # exploit "stay distressed" as a strictly-dominant strategy (termination immunity AND
        # dense reward together, for free) -- imag_r climbed while sim_r stayed pinned near zero
        # and ~85% of all transitions ended up distressed. A normal mid-gait stumble doesn't need
        # this bonus anyway: the ordinary reward terms below are merely REDUCED there, not
        # vanished the way they are for a genuine ~180-degree-flipped scripted-recovery episode
        # (see recovery_progress_reward's own docstring for the exp(-scale*error^2) numerics).
        if is_scripted_recovery is not None and is_scripted_recovery.any():
            shaping = recovery_progress_reward(mat_state_to(S))
            r = torch.where(is_scripted_recovery, r + shaping, r)
        surv = compute_soft_termination_prob(mat_state_to(S), mat_state_to(ref), is_recovery=is_recovery)
        blend = learned_term_blend(it)
        if blend > 0 and g is not None:
            # NOT wrapped in no_grad: this runs inside policy_update()'s BPTT, so gradient must
            # flow through term_head back to the policy's actions, same as critic_target's own
            # frozen-but-differentiable usage just below in policy_update(). term_head's own
            # PARAMETERS don't accumulate gradients here regardless (wm.requires_grad_(False) is
            # already set for the whole policy_update() call), only upstream activations do.
            surv_learned = 1 - torch.sigmoid(wm.termination_logit(S, g))
            surv = (1 - blend) * surv + blend * surv_learned
        return r, 1 - surv  # (reward, soft "done" probability)

    def critic_input(S, ref, g):
        return torch.cat((critic_features(S, ref), g), -1)

    def lr_at(it: int) -> float:
        if it >= args.lr_decay_iters:
            return args.lr * 0.03
        return args.lr * (0.03 + 0.97 * 0.5 * (1 + math.cos(math.pi * it / args.lr_decay_iters)))

    def action_noise_at(it: int) -> float:
        """Cosine decay from --action_noise_start to the validated --action_noise floor, same
        schedule shape/length as lr_at -- more exploration while the policy is still bad early
        on, settling to the floor that's known not to drown out the control torque (see
        --action_noise's own help text) by the time the LR has also mostly decayed."""
        if it >= args.lr_decay_iters:
            return args.action_noise
        frac = 0.5 * (1 + math.cos(math.pi * it / args.lr_decay_iters))
        return args.action_noise + (args.action_noise_start - args.action_noise) * frac

    def horizon_at(it: int) -> int:
        """Linear growth from --horizon_start to --horizon over the same schedule length as the
        LR/action_noise decay -- short imagined rollouts are cheaper and give easier credit
        assignment while the world model/policy are still bad, longer ones matter more once
        there's something non-degenerate to assign credit over (see the paper's own H ablation,
        Fig. 4b/c: H=32 wins asymptotically but needs the critic to not be strictly worse than
        shorter H early)."""
        if it >= args.lr_decay_iters:
            return args.horizon
        frac = it / args.lr_decay_iters
        return max(args.horizon_start, min(args.horizon, round(args.horizon_start + (args.horizon - args.horizon_start) * frac)))  # fmt: skip

    def set_requires_grad(m: nn.Module, flag: bool):
        for p in m.parameters():
            p.requires_grad_(flag)

    def wm_update(it=None, diag_f=None):
        diag = diag_f is not None and it is not None and args.wm_diag_every > 0 and it % args.wm_diag_every == 0
        w = buffer.sample_windows(args.wm_batch, args.wm_window, diag=diag)
        # xi is a per-EPISODE constant (see env.py's _worker) -- w["xi"] is gathered per window
        # position like every other field, but all positions within a valid (mask==1) window
        # share the same value, so the window's first position is what wm_rollout needs to roll
        # forward under a single consistent xi for the whole k-step autoregressive prediction.
        S_hat = wm_rollout(w["s"], w["a"], w["xi"][:, 0])
        loss, e_last, e_full = wm_err(S_hat, w["s1"], w["mask"])
        term_acc = None
        if args.learned_termination:
            # Trained on the WM's own PREDICTED states (S_hat), not the real s1 -- matching how
            # it's actually USED during imagined rollouts (policy_update's reward_term never
            # sees a real state), so train/inference stay consistent. The LABEL is always the
            # real analytic hard-termination (compute_termination) evaluated on the matching
            # real reference at s1's time -- the ground truth this head is learning to predict,
            # same role as the real simulator's termination signal in the paper.
            with torch.no_grad():
                B_, k_ = w["t"].shape
                t1 = w["t"] + CONTROL_DT
                s1_state = unflatten_state(w["s1"])
                ref1 = motion.frame_state(w["motion_id"], t1)
                # Recovery rows must NOT get a positive ("terminated") label just because they
                # start ~180 degrees misaligned by design -- same is_recovery override already
                # applied everywhere else termination is computed (imitation_task.py's done(),
                # compute_soft_termination_prob above). Without this, the head would learn to
                # (correctly, but unhelpfully) predict "always terminated" for recovery states.
                # --adaptive_distress ORs in the generalized per-transition flag (see env.py's
                # worker) on top of the file-based one -- both can be active at once in principle
                # (not exercised together in practice, but neither excludes the other).
                is_recovery_label = (w["motion_id"] == recovery_index) if recovery_index is not None else None
                if args.adaptive_distress:
                    is_recovery_label = w["distressed"] if is_recovery_label is None else (is_recovery_label | w["distressed"])
                term_label = compute_termination(mat_state_to(s1_state), mat_state_to(ref1), is_recovery=is_recovery_label).float()
                # MultiMotion.goal() only supports 1-D (B,) motion_idx/time (unlike frame_state,
                # which handles arbitrary shapes) -- flatten to (B*k,) and reshape back.
                g1 = motion.goal(w["motion_id"].reshape(-1), t1.reshape(-1), K, CONTROL_DT).view(B_, k_, -1)
            term_logit = wm.termination_logit(S_hat, g1)
            bce = nn.functional.binary_cross_entropy_with_logits(term_logit, term_label, reduction="none")
            bce = (bce * w["mask"]).sum() / w["mask"].sum().clamp(min=1)
            loss = loss + bce
            with torch.no_grad():
                pred = (torch.sigmoid(term_logit) > 0.5).float()
                correct = ((pred == term_label).float() * w["mask"]).sum()
                term_acc = (correct / w["mask"].sum().clamp(min=1)).item()
        opt_wm.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(wm.parameters(), 1.0)
        opt_wm.step()
        m = w["mask"][:, -1]
        if diag:
            # Per-transition world-model loss, tagged by {gait, is_recovery,
            # is_immediately_post_reset, reward magnitude} -- see plan.md Phase 5.1's
            # diagnostic: is the instability's root a reset-teleport discontinuity the dynamics
            # head can't fit (would show as is_post_reset==1 rows dominating the high-error
            # tail), or a general hard-gait pattern spread across the episode (would show
            # is_recovery==1 rows elevated regardless of is_post_reset)? Written with
            # torch.no_grad() already in effect for e_full's consumers below since it's detached.
            with torch.no_grad():
                valid = w["mask"] > 0.5
                mid = w["motion_id"][valid].cpu().numpy()
                ep = w["ep_step"][valid].cpu().numpy()
                rew = w["r"][valid].cpu().numpy()
                err = e_full.detach()[valid].cpu().numpy()
                dis = w["distressed"][valid].cpu().numpy()
            for mi, epi, ri, ei, di in zip(mid, ep, rew, err, dis):
                is_rec = int(recovery_index is not None and int(mi) == recovery_index) or int(di)
                is_post_reset = int(epi == 0)
                diag_f.write(f"{it},{int(mi)},{is_rec},{is_post_reset},{ri:.4f},{ei:.6f}\n")
            diag_f.flush()
        return loss.detach() / args.wm_window, (e_last.detach() * m).sum() / m.sum().clamp(min=1), term_acc

    def policy_update(it=None):
        # pace.txt loops forever (no finite clip pool like supermimic_vibed's LAFAN1 set), so
        # unlike the original every buffered start is valid for an HZ-step rollout -- no
        # clip-end filtering needed.
        s, h = buffer.sample_starts(args.pi_batch, Hh)
        S = unflatten_state(s["s"])
        t_start, a_prev = s["t"], s["a_prev"]
        # Which motion.py clip each row's imagined rollout continues -- fixed for the rollout's
        # lifetime (buffer.sample_starts' history window never crosses a reset, so it's safe to
        # reuse this same per-row index for the history lookup below too).
        motion_idx = s["motion_id"]
        xi = s["xi"]  # per-episode constant -- same value for the whole imagined horizon below
        # Fixed for the imagined rollout's whole H-step horizon, same as motion_idx above -- not
        # recomputed per imagined step (that would need the imagined trajectory's own evolving
        # error and a carried hysteresis flag, a bigger change deferred unless this simpler,
        # seed-fixed version proves insufficient). See --adaptive_distress's help text.
        #
        # is_scripted_recovery (file-based only) vs is_recovery (file-based OR'd with generic
        # distress) are DELIBERATELY kept separate -- see reward_term's own note: a 2026-11-12
        # validation run showed generalizing recovery_progress_reward's shaping BONUS to generic
        # distress creates a reward-hacking exploit (termination-immunity + dense reward
        # together make "stay distressed" strictly better than actually recovering: imag_r
        # climbed 4.8->8.9 while sim_r stayed pinned near 0 and the distress rate climbed to
        # ~85% of all transitions). The termination-forgiveness side of is_recovery is still
        # extended to generic distress (that part IS the intended generalization); only the
        # shaping bonus stays restricted to the original scripted-recovery case, where it's
        # actually needed (the ordinary reward terms are genuinely ~0 for a 180-degree-flipped
        # episode, unlike a normal mid-gait stumble where they're merely reduced, not vanished).
        is_scripted_recovery = (motion_idx == recovery_index) if recovery_index is not None else None
        is_recovery = is_scripted_recovery
        if args.adaptive_distress:
            distressed_seed = s["distressed"].bool()
            is_recovery = distressed_seed if is_recovery is None else (is_recovery | distressed_seed)
        with torch.no_grad():
            hist_idx = motion_idx.unsqueeze(-1).expand_as(h["t"])
            xi_hist = xi.unsqueeze(1).expand(-1, h["t"].shape[1], -1)
            o_hist = mdp_obs(unflatten_state(h["s"]), h["a_prev"], motion.frame_state(hist_idx, h["t"]).R, xi_hist)
        o_hist = mdp_add_noise(o_hist)
        set_requires_grad(wm, False)
        set_requires_grad(critic, False)
        B = args.pi_batch
        t = t_start
        g0 = motion.goal(motion_idx, t, K, CONTROL_DT)
        g = g0
        S0 = S
        Ss, As, Aps, Ts, Gs = [], [], [], [], []
        for _ in range(HZ):
            Aps.append(a_prev)
            S, a_prev, o_hist, t, g = imag_step(S, o_hist, a_prev, t, g, motion_idx, xi)
            Ss.append(S), As.append(a_prev), Ts.append(t), Gs.append(g)
        flat = lambda xs: torch.stack(xs).flatten(0, 1)  # noqa: E731  (H*B, ...)
        S_all = MatState(*(flat([St[i] for St in Ss]) for i in range(6)))
        ref_all = motion.frame_state(motion_idx.repeat(HZ), flat(Ts))
        is_recovery_flat = is_recovery.repeat(HZ) if is_recovery is not None else None
        is_scripted_recovery_flat = is_scripted_recovery.repeat(HZ) if is_scripted_recovery is not None else None
        r, d = reward_term(S_all, ref_all, flat(As), flat(Aps), is_recovery=is_recovery_flat,
                          is_scripted_recovery=is_scripted_recovery_flat, g=flat(Gs), it=it)  # fmt: skip
        r, d = r.view(HZ, B), d.view(HZ, B)
        alive = torch.cumprod(1 - d, 0)  # c_{t+1}
        c_t = torch.cat((torch.ones(1, B, device=dev), alive[:-1]), 0)
        disc = args.gamma ** torch.arange(HZ, device=dev, dtype=torch.float32).unsqueeze(-1)
        J = (disc * c_t * r).sum(0)
        c = alive[-1]
        r_sum = (c_t * r).detach().sum(0)
        rews, dones = r.detach(), d.detach()
        if not args.no_critic:
            ref_last = motion.frame_state(motion_idx, t)
            v_bootstrap = critic_target(critic_input(S, ref_last, g)).clamp(-args.critic_value_clip, args.critic_value_clip)
            J = J + (args.gamma**HZ) * c * v_bootstrap
        loss = -J.mean()
        opt_pi.zero_grad(set_to_none=True)
        loss.backward()
        gn = nn.utils.clip_grad_norm_(pi.parameters(), 1.0)
        nn.utils.clip_grad_value_(pi.parameters(), 0.1)
        if torch.isfinite(gn):
            opt_pi.step()
        set_requires_grad(wm, True)
        set_requires_grad(critic, True)
        # ---------------- critic: GAE(lambda) targets over the imagined transitions ----------------
        v_loss = torch.zeros(())
        if not args.no_critic:
            with torch.no_grad():
                S_c = MatState(*(torch.cat((S0[i].unsqueeze(0), torch.stack([St[i].detach() for St in Ss])), 0).flatten(0, 1) for i in range(6)))  # fmt: skip
                t_c = torch.cat((t_start.unsqueeze(0), torch.stack(Ts)), 0).flatten()
                g_c = torch.cat((g0.unsqueeze(0), torch.stack(Gs)), 0).flatten(0, 1)
                ref_c = motion.frame_state(motion_idx.repeat(HZ + 1), t_c)
                X = critic_input(S_c, ref_c, g_c).view(HZ + 1, B, -1)
            critic.norm.update(X)
            with torch.no_grad():
                # Clip here, not just at the bootstrap use-site above: V feeds the GAE recursion
                # below, so an unclipped outlier would otherwise propagate into every target in
                # the window via the (1-lam)*V[t+1] + lam*R term, not just the final bootstrap.
                V = critic_target(X).clamp(-args.critic_value_clip, args.critic_value_clip)
                R = V[-1]
                targets = []
                for t_ in range(HZ - 1, -1, -1):
                    R = rews[t_] + args.gamma * (1 - dones[t_]) * ((1 - args.lam) * V[t_ + 1] + args.lam * R)
                    targets.append(R)
                targets = torch.stack(targets[::-1]).clamp(-args.critic_value_clip, args.critic_value_clip)  # (H, B)
            Xf, Yf = X[:-1].reshape(-1, X.shape[-1]), targets.reshape(-1)
            n = Xf.shape[0]
            mb = max(n // args.critic_minibatches, 1)
            for _ in range(args.critic_epochs):
                perm = torch.randperm(n, device=dev)
                for i in range(args.critic_minibatches):
                    idx = perm[i * mb : (i + 1) * mb]
                    if args.critic_loss == "huber":
                        # Caps the gradient from any single outlier target to a constant
                        # magnitude (linear region beyond --critic_huber_delta) instead of MSE's
                        # error-proportional blowup -- see --critic_loss's help text / project
                        # memory for why this targets Phase 5.1's specific divergence mechanism.
                        v_loss = nn.functional.huber_loss(critic(Xf[idx]), Yf[idx], delta=args.critic_huber_delta)
                    else:
                        v_loss = ((critic(Xf[idx]) - Yf[idx]) ** 2).mean()
                    opt_v.zero_grad(set_to_none=True)
                    v_loss.backward()
                    nn.utils.clip_grad_norm_(critic.parameters(), 1.0)
                    opt_v.step()
            with torch.no_grad():
                for p_t, p in zip(critic_target.parameters(), critic.parameters()):
                    p_t.mul_(1 - args.critic_tau).add_(args.critic_tau * p)
        return loss.detach(), v_loss.detach(), (r_sum / HZ / 0.1).mean(), gn.detach(), c.mean().detach()

    # ------------------------------------------------------------------------------------------
    motor_control_mode = None
    if args.control_mode == "position":
        from motion_imitation.robots import robot_config
        motor_control_mode = robot_config.MotorControlMode.POSITION
    sim = TrackingSim(motion_files, args.num_envs, seed=args.seed, device=dev,
                      recovery_motion_names=recovery_motion_names,
                      recovery_init_prob=args.recovery_init_prob,
                      recovery_sample_prob=args.recovery_sample_prob,
                      bin_duration=args.bin_duration, gamma=args.gamma,
                      bin_return_decay=args.bin_return_decay,
                      bin_softmax_sharpness=args.bin_sharpness,
                      bin_push_every=args.bin_push_every,
                      motor_control_mode=motor_control_mode,
                      adaptive_distress=args.adaptive_distress)
    buffer = ReplayBuffer(args.buffer_steps, args.num_envs, dev)
    obs_hist = torch.zeros(args.num_envs, Hh, OBS_DIM, device=dev)
    cur_sim, cur_t, cur_motion_id, cur_xi = sim.reset()
    a_prev = torch.zeros(args.num_envs, NUM_JOINTS, device=dev)

    def collect(n_steps: int, stats: dict, action_noise: float):
        nonlocal obs_hist, cur_sim, cur_t, cur_motion_id, cur_xi, a_prev, sim_steps
        for _ in range(n_steps):
            cur_ref = motion.frame_state(cur_motion_id, cur_t)
            S0_mat = mat_state_from(cur_sim)
            o = mdp_add_noise(mdp_obs(S0_mat, a_prev, cur_ref.R, cur_xi))
            fresh = torch.as_tensor(sim.ep_step == 0, device=dev)
            obs_hist = torch.where(fresh[:, None, None], o.unsqueeze(1).expand_as(obs_hist),
                                   torch.cat((obs_hist[:, 1:], o.unsqueeze(1)), 1))
            g = motion.goal(cur_motion_id, cur_t, K, CONTROL_DT)
            pi.obs_norm.update(o)
            pi.goal_norm.update(g)
            with torch.no_grad():
                a = torch.clamp(pi(obs_hist, g), -1.0, 1.0)
            a = torch.clamp(a + action_noise * torch.randn_like(a), -1.0, 1.0)
            out = sim.step(a)
            s0_mat, s1_mat = mat_state_from(out["sim"]), mat_state_from(out["sim1"])
            buffer.add({
                "s": flatten_state(s0_mat), "s1": flatten_state(s1_mat), "a": a, "a_prev": a_prev,
                "t": out["t"], "r": out["reward"], "ep_step": out["ep_step"], "done": out["done"],
                "motion_id": cur_motion_id, "xi": cur_xi, "distressed": out["distressed"],
            })  # fmt: skip
            wm.in_norm.update(wm.inputs(s0_mat, a, cur_xi))
            wm.out_norm.update(wm.targets(s0_mat, s1_mat))
            sim_steps += args.num_envs
            stats["r"].append(out["reward"].mean().item())
            stats["done"].append(out["done"].float().sum().item())
            a_prev = a
            cur_motion_id = out["next_motion_id"]
            cur_sim, cur_t = out["next_sim"], out["next_t"]
            cur_xi = out["next_xi"]

    def _raw_sd(m):
        """state_dict() of the UNCOMPILED module -- torch.compile's OptimizedModule wrapper
        prefixes every key with '_orig_mod.' (confirmed directly: inspected a checkpoint saved
        under --compile), which would silently break loading on any run without --compile (or a
        different PyTorch version's wrapper behavior). wm itself is never wrapped (only
        wm.step is, see --compile's setup) so it never needs this, but pi/critic/critic_target
        all go through here regardless of --compile so checkpoints stay byte-identical either way."""
        return (m._orig_mod if hasattr(m, "_orig_mod") else m).state_dict()

    def _prune_old_checkpoints(it: int):
        """Numbered ckpt_NNNNNN.pt files accumulate forever otherwise -- at --ckpt_every=50 over
        a multi-day run this silently fills the disk (confirmed: a live run hit 1468 files/90GB
        on this one log_dir alone, and three since-retired lineages together left ~325GB behind
        before this was caught -- the resulting ENOSPC corrupted an in-progress torch.save and
        killed the process outright). Keeps a rolling window of the most recent
        `--ckpt_keep_recent` saves (fine-grained rollback right after a bad change) plus every
        `--ckpt_milestone_every`th iteration forever (coarse long-term history) -- deletes
        everything else. `last.pt` (the one actually used by --resume) is never touched here."""
        pattern = re.compile(r"ckpt_(\d+)\.pt$")
        existing = []
        for f in os.listdir(args.log_dir):
            m = pattern.match(f)
            if m:
                existing.append(int(m.group(1)))
        for old_it in existing:
            is_recent = old_it > it - args.ckpt_keep_recent * args.ckpt_every
            is_milestone = args.ckpt_milestone_every > 0 and old_it % args.ckpt_milestone_every == 0
            if not is_recent and not is_milestone:
                try:
                    os.remove(os.path.join(args.log_dir, f"ckpt_{old_it:06d}.pt"))
                except OSError:
                    pass

    def save_checkpoint(it: int, t_start_wall: float):
        """Atomic write (tmp file + rename) so a kill/shutdown mid-save can never corrupt the
        last good checkpoint -- the old file stays intact until the new one is fully written."""
        ck = {"wm": wm.state_dict(), "pi": _raw_sd(pi), "critic": _raw_sd(critic),
              "opt_wm": opt_wm.state_dict(), "opt_pi": opt_pi.state_dict(), "opt_v": opt_v.state_dict(),
              "iteration": it, "wall_time": time.time() - t_start_wall, "sim_steps": sim_steps,
              "args": vars(args), "saved_at": time.strftime("%Y-%m-%d %H:%M:%S")}  # fmt: skip
        last = os.path.join(args.log_dir, "last.pt")
        torch.save(ck, last + ".tmp")
        os.replace(last + ".tmp", last)
        small = {k: ck[k] for k in ("pi", "wm", "iteration", "wall_time", "sim_steps", "args", "saved_at")}
        path = os.path.join(args.log_dir, f"ckpt_{it:06d}.pt")
        torch.save(small, path + ".tmp")
        os.replace(path + ".tmp", path)
        _prune_old_checkpoints(it)
        print(f"[checkpoint] saved at iteration {it} -> {last}", flush=True)

    # Resume with `--resume <log_dir>/last.pt` picks this back up: weights/optimizers are
    # restored above, only the (cheap-to-rebuild) replay buffer starts empty again.
    stop_requested = {"flag": False}

    def _request_stop(signum, frame):
        stop_requested["flag"] = True
        print(f"\n[signal {signum}] finishing the current iteration, then saving and exiting...", flush=True)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    stats = {"r": [], "done": []}
    # The replay buffer isn't part of the checkpoint (cheap to rebuild, see save_checkpoint's
    # docstring), so it's always empty here -- including on --resume, where start_it != 0.
    # Always warm it up to at least wm_window, or the first wm_update() below would sample
    # from an empty/too-small buffer.
    collect(max(args.warmup_steps, args.wm_window), stats, args.action_noise_start)
    log_f = open(os.path.join(args.log_dir, "train_log.csv"), "a")
    if start_it == 0:
        log_f.write("iteration,wall_h,sim_steps,wm_loss,wm_err8,pi_loss,v_loss,imag_rew,pi_gradnorm,imag_survival,sim_rew,it_per_s,lr,action_noise,horizon\n")  # fmt: skip
    wm_diag_f = None
    if args.wm_diag_every > 0:
        wm_diag_f = open(os.path.join(args.log_dir, "wm_diag.csv"), "a")
        if start_it == 0:
            wm_diag_f.write("iteration,motion_id,is_recovery,is_post_reset,reward,wm_loss\n")
    t_start_wall = time.time() - wall_offset
    t_log = time.time()
    step_acc = 0.0
    agg = {"wm": [], "wm8": [], "pi": [], "v": [], "ir": [], "gn": [], "surv": []}
    last_completed_it = start_it
    checkpoint_current = True  # last_completed_it already has a checkpoint on disk (or is start_it)
    try:
        for it in range(start_it, args.iterations):
            lr = lr_at(it)
            for o in (opt_wm, opt_pi):
                for gparam in o.param_groups:
                    gparam["lr"] = lr
            for gparam in opt_v.param_groups:
                gparam["lr"] = lr * args.critic_lr_scale
            action_noise = action_noise_at(it)
            HZ = horizon_at(it)
            step_acc += 1.0 / args.utd
            n = int(step_acc)
            step_acc -= n
            collect(n, stats, action_noise)
            for _ in range(args.wm_batches):
                l, e8, tacc = wm_update(it=it, diag_f=wm_diag_f)
                update_term_state(it, tacc)
            agg["wm"].append(l), agg["wm8"].append(e8)
            if it >= args.wm_only_iters:
                pl, vl, ir, gn, surv = policy_update(it=it)
                agg["pi"].append(pl), agg["v"].append(vl), agg["ir"].append(ir), agg["gn"].append(gn), agg["surv"].append(surv)  # fmt: skip
            if (it + 1) % args.log_every == 0:
                mean = lambda x: float(sum(float(v) for v in x) / max(len(x), 1))  # noqa: E731
                ips = args.log_every / (time.time() - t_log)
                t_log = time.time()
                wall_h = (time.time() - t_start_wall) / 3600
                row = [it + 1, wall_h, sim_steps, mean(agg["wm"]), mean(agg["wm8"]), mean(agg["pi"]), mean(agg["v"]),
                       mean(agg["ir"]), mean(agg["gn"]), mean(agg["surv"]), mean(stats["r"]), ips, lr,
                       action_noise, HZ]  # fmt: skip
                log_f.write(",".join(f"{x:.6g}" if isinstance(x, float) else str(x) for x in row) + "\n")
                log_f.flush()
                term_suffix = ""
                if args.learned_termination:
                    blend_now = learned_term_blend(it + 1)
                    term_suffix = f" | term_acc={term_state['acc_ema']:.3f} blend={blend_now:.2f}"
                print(f"it {it + 1} | {wall_h:.3f} h | wm {row[3]:.4f} e8 {row[4]:.4f} | pi {row[5]:.3f} v {row[6]:.4f} "
                     f"imag_r {row[7]:.3f} surv {row[9]:.3f} | sim_r {row[10]:.3f} | H={HZ} noise={action_noise:.3f} "
                     f"| {ips:.2f} it/s{term_suffix}", flush=True)  # fmt: skip
                stats = {k: [] for k in stats}
                agg = {k: [] for k in agg}
            last_completed_it = it + 1
            checkpoint_current = False
            if last_completed_it % args.ckpt_every == 0 or last_completed_it == args.iterations:
                save_checkpoint(last_completed_it, t_start_wall)
                checkpoint_current = True
            if stop_requested["flag"]:
                break
    finally:
        # Always leave a fresh checkpoint behind, even on Ctrl+C/SIGTERM or an unexpected
        # error -- cheap (a few seconds) relative to the iterations it could otherwise cost.
        if not checkpoint_current:
            save_checkpoint(last_completed_it, t_start_wall)
        sim.close()


if __name__ == "__main__":
    main()
