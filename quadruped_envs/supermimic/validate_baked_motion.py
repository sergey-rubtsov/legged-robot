"""Phase-3 validation: BakedMotion (vectorized, reimplements MotionData's interpolation +
cycle-offset formulas in torch for training-loop throughput) must match ReferenceMotion's
scalar state_at(t) (which wraps the real MotionData directly, already validated against the
live simulator in validate_motion.py) -- including across multiple loop cycles, since
pace.txt has a substantial per-cycle root translation + heading rotation
(EnableCycleOffsetPosition/Rotation both true)."""
import os
import sys

import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from quadruped_envs.supermimic import rotations as rot
from quadruped_envs.supermimic.motion import BakedMotion, ReferenceMotion

motion_file = os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "pace.txt")

if __name__ == "__main__":
    ref = ReferenceMotion([motion_file])
    baked = BakedMotion(motion_file)
    print(f"duration={baked.duration:.4f}s num_frames={baked.num_frames} "
         f"cycle_delta_pos={baked.cycle_delta_pos.tolist()} cycle_delta_heading={baked.cycle_delta_heading:.4f}")

    rng = np.random.default_rng(0)
    n = 64
    times = rng.uniform(0, baked.duration * 4.5, size=n).astype(np.float32)  # spans ~4.5 loops
    baked_states = baked.frame_state(torch.tensor(times))

    max_err = {"p": 0.0, "rot": 0.0, "v": 0.0, "w": 0.0, "q": 0.0, "qd": 0.0}
    for i, t in enumerate(times):
        s = ref.state_at(float(t))
        s_R = rot.quat_to_mat(s.root_rot)
        max_err["p"] = max(max_err["p"], (baked_states.p[i] - s.root_pos).abs().max().item())
        max_err["rot"] = max(max_err["rot"], (baked_states.R[i] - s_R).abs().max().item())
        max_err["v"] = max(max_err["v"], (baked_states.v[i] - s.root_lin_vel).abs().max().item())
        max_err["w"] = max(max_err["w"], (baked_states.w[i] - s.root_ang_vel).abs().max().item())
        max_err["q"] = max(max_err["q"], (baked_states.q[i] - s.joint_pos).abs().max().item())
        max_err["qd"] = max(max_err["qd"], (baked_states.qd[i] - s.joint_vel).abs().max().item())

    for k, v in max_err.items():
        print(f"max |baked.{k} - ReferenceMotion.{k}| = {v:.3e}")

    # goal() smoke test: shapes and finiteness only (no independent ground truth for this one).
    t_batch = torch.tensor(rng.uniform(0, baked.duration * 2, size=8).astype(np.float32))
    g = baked.goal(t_batch, K=10, control_dt=0.005)
    assert g.shape == (8, 10 * 39)
    assert torch.isfinite(g).all()
    print("goal() shape/finiteness OK:", tuple(g.shape))

    for k, v in max_err.items():
        assert v < 2e-3, f"{k} mismatch too large: {v}"
    print("VALIDATION PASSED")
