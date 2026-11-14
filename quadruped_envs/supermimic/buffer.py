"""Fixed-size replay buffer of real-rollout transitions, laid out as (time, env). Ported from
supermimic_vibed/supermimic/buffer.py nearly as-is (it's state-tuple-generic) -- the only
real change is replacing SuperMimic's discrete reference frame index `f` with this project's
continuous reference time `t` (the real env's control dt doesn't evenly divide the motion
file's frame duration -- see motion.py's module note). The sim-context field `xi` (dropped in
Phase 1, see mdp.py's module note) was added back in Phase 5.2.
"""
from __future__ import annotations

import torch

from quadruped_envs.supermimic.mdp import NUM_JOINTS, STATE_DIM, XI_DIM

FIELDS = {
    "s": (STATE_DIM, torch.float32),
    "s1": (STATE_DIM, torch.float32),
    "a": (NUM_JOINTS, torch.float32),
    "a_prev": (NUM_JOINTS, torch.float32),
    "t": ((), torch.float32),
    "r": ((), torch.float32),
    "ep_step": ((), torch.int32),
    "done": ((), torch.bool),
    # Index into the motion_files list this transition's reference clip was sampled from (see
    # env.py's module note) -- needed so an imagined rollout seeded from this buffer row
    # continues against the SAME clip it was really collected from, not whichever one happens
    # to be first when training on more than one motion at once.
    "motion_id": ((), torch.int64),
    # Sim-context: [friction, 12 per-joint action-space offsets] -- constant for a given real
    # env's CURRENT episode (friction is fixed for a worker's whole lifetime; joint offsets are
    # re-randomized every episode reset), see env.py's _worker for where it's actually sampled.
    "xi": (XI_DIM, torch.float32),
    # Whether the real worker treated this transition's action as wide-range/no-hard-fail at the
    # time it was taken -- see env.py's _worker module note on the generalized "distress" state
    # machine (--adaptive_distress). Always False when that flag is off, same as every prior run.
    "distressed": ((), torch.bool),
}


class ReplayBuffer:
    def __init__(self, T: int, N: int, device: str):
        self.T, self.N, self.device = T, N, device
        self.data = {}
        for k, (shape, dt) in FIELDS.items():
            shp = (T, N) + ((shape,) if isinstance(shape, int) else shape)
            self.data[k] = torch.zeros(shp, dtype=dt, device=device)
        self.ptr, self.size = 0, 0

    def add(self, tr: dict):
        for k in FIELDS:
            self.data[k][self.ptr] = tr[k]
        self.ptr = (self.ptr + 1) % self.T
        self.size = min(self.size + 1, self.T)

    def _row(self, age: torch.Tensor) -> torch.Tensor:
        """Map age-ordered time index (0 = oldest) to the storage row."""
        return (self.ptr - self.size + age) % self.T

    def sample_windows(self, B: int, k: int, diag: bool = False):
        """k-step windows; returns dict of (B, k, ...) tensors and a validity mask (B, k).
        diag=True also includes motion_id/ep_step/r (same (B,k) shape, same rows/env indexing,
        just slicing already-stored tensors) -- for wm_update's optional per-transition
        diagnostic logging (see train.py's --wm_diag_every), tagging each window transition by
        {gait, is_immediately_post_reset (ep_step==0), reward magnitude} to distinguish a
        reset-teleport-discontinuity failure mode from a general hard-gait one. Off by default
        since it's pure overhead (3 extra index_select-style gathers) when not diagnosing."""
        t0 = torch.randint(0, self.size - k + 1, (B,), device=self.device)
        env = torch.randint(0, self.N, (B,), device=self.device)
        ages = t0.unsqueeze(-1) + torch.arange(k, device=self.device)
        rows = self._row(ages)
        e = env.unsqueeze(-1).expand_as(rows)
        keys = ("s", "s1", "a", "done", "t", "motion_id", "xi", "distressed") + (("ep_step", "r") if diag else ())
        out = {key: self.data[key][rows, e] for key in keys}
        # transition j is valid if no episode ended at transitions 0..j-1
        d = out["done"].float()
        mask = torch.cumprod(torch.cat((torch.ones(B, 1, device=self.device), 1 - d[:, :-1]), -1), -1)
        out["mask"] = mask
        return out

    def sample_starts(self, B: int, hist: int):
        """Start points for imagination plus the indices of their observation history (oldest first)."""
        age = torch.randint(0, self.size, (B,), device=self.device)
        env = torch.randint(0, self.N, (B,), device=self.device)
        row = self._row(age)
        ep_step = self.data["ep_step"][row, env].long()
        back = torch.arange(hist - 1, -1, -1, device=self.device)  # hist-1 ... 0
        back = torch.minimum(back.unsqueeze(0), ep_step.unsqueeze(-1))  # stay inside the episode
        back = torch.minimum(back, age.unsqueeze(-1))  # stay inside the buffer
        hrows = self._row(age.unsqueeze(-1) - back)
        henv = env.unsqueeze(-1).expand_as(hrows)
        start = {k: self.data[k][row, env] for k in ("s", "a_prev", "t", "motion_id", "xi", "distressed")}
        hist_d = {k: self.data[k][hrows, henv] for k in ("s", "a_prev", "t")}
        return start, hist_d
