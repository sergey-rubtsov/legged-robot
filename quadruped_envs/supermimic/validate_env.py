"""Phase-2 validation: run TrackingSim's N parallel real PyBullet processes, confirm the
batched state it returns agrees with mdp.py's reward (same cross-check as
validate_against_sim.py, now through the multiprocess plumbing) and that per-env episode
bookkeeping / process shutdown are correct."""
import os
import sys
import time

import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from quadruped_envs.supermimic import mdp
from quadruped_envs.supermimic.env import TrackingSim
from quadruped_envs.supermimic.motion import BakedMotion

motion_file = os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "pace.txt")

NUM_ENVS = 4
N_STEPS = 150

if __name__ == "__main__":
    sim = TrackingSim([motion_file], num_envs=NUM_ENVS, seed=0)
    baked = BakedMotion(motion_file)
    t0 = time.time()
    sim.reset()
    max_t_ref_err = 0.0

    rng = np.random.default_rng(0)
    max_component_err = {"pose": 0.0, "velocity": 0.0, "root_pose": 0.0, "root_velocity": 0.0}
    resets_seen = np.zeros(NUM_ENVS, dtype=np.int64)
    ep_step_consistent = True
    prev_ep_step = sim.ep_step.copy()

    for i in range(N_STEPS):
        actions = torch.as_tensor(rng.uniform(-1, 1, size=(NUM_ENVS, 12)), dtype=torch.float32)
        out = sim.step(actions)

        # out["reward"]/out["done"] are the outcome of this action, so compare against
        # sim1/ref1 (the post-step state) -- NOT sim/ref, which is s0, the pre-action state
        # the policy's observation would be computed from.
        breakdown = mdp.compute_reward(out["sim1"], out["ref1"])
        # `out["reward"]` is the real env reward (includes the end-effector term, which can
        # differ slightly from mine for the documented getLinkState-staleness reason -- see
        # validate_against_sim.py). Only compare the 4 terms that don't touch getLinkState.
        pose_r, vel_r = mdp.pose_and_velocity_reward(out["sim1"], out["ref1"])
        root_pose_r = mdp.root_pose_reward(out["sim1"], out["ref1"])
        root_vel_r = mdp.root_velocity_reward(out["sim1"], out["ref1"])
        total_without_end_eff = (mdp.POSE_WEIGHT * pose_r + mdp.VELOCITY_WEIGHT * vel_r
                                 + mdp.ROOT_POSE_WEIGHT * root_pose_r
                                 + mdp.ROOT_VELOCITY_WEIGHT * root_vel_r)
        # Lower bound check: real reward must be >= this (end_effector term only adds, in [0, END_EFFECTOR_WEIGHT]).
        assert (out["reward"] >= total_without_end_eff - 1e-4).all(), \
            f"step {i}: real reward below the 4 exact terms' floor"

        # Critical Phase-3 integration check: BakedMotion.frame_state(out["t"]) must agree
        # with out["ref"] (the real ref_model's state at that same instant) -- this is the
        # link the training loop relies on to look up goals at the exact time the real sim
        # used. Check both s0 (t/ref) and s1 (t1/ref1).
        for t_key, ref_key in (("t", "ref"), ("t1", "ref1")):
            baked_ref = baked.frame_state(out[t_key])
            real_ref_mat = mdp.mat_state_from(out[ref_key])
            err = max((baked_ref.p - real_ref_mat.p).abs().max().item(),
                     (baked_ref.R - real_ref_mat.R).abs().max().item())
            max_t_ref_err = max(max_t_ref_err, err)

        done_now = out["done"].numpy()
        resets_seen += done_now.astype(np.int64)
        for e in range(NUM_ENVS):
            if done_now[e]:
                assert sim.ep_step[e] == 0, f"env {e} didn't reset ep_step after done"
            elif prev_ep_step[e] > 0:
                if sim.ep_step[e] != prev_ep_step[e] + 1:
                    ep_step_consistent = False
        prev_ep_step = sim.ep_step.copy()

    dt = time.time() - t0
    print(f"{NUM_ENVS} envs x {N_STEPS} steps in {dt:.2f}s "
         f"({NUM_ENVS * N_STEPS / dt:.0f} real env-steps/s)")
    print("resets per env:", resets_seen.tolist())
    print("ep_step bookkeeping consistent:", ep_step_consistent)
    print(f"max |BakedMotion.frame_state(out['t']) - out['ref']| = {max_t_ref_err:.3e}")

    sim.close()
    time.sleep(0.5)
    alive = [p.is_alive() for p in sim._procs]
    print("worker processes still alive after close():", alive)

    assert ep_step_consistent
    assert max_t_ref_err < 1e-3, "exposed reference time doesn't match BakedMotion's lookup"
    assert not any(alive), "worker processes did not shut down cleanly"
    print("VALIDATION PASSED")
