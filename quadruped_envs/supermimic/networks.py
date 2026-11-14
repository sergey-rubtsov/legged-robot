"""World model, policy (motion encoder + shared decoder), and critic -- ported from
supermimic_vibed/supermimic/networks.py (same architecture family, paper Table A.4), resized
for the OD robot's 12 joints via mdp.py's INV_DIM/OBS_DIM/NUM_JOINTS, and with the sim-context
vector xi dropped (see mdp.py's module note: no dynamics randomization in this first pass).

AMP_DTYPE defaults to None (plain fp32, no autocast): the original uses CUDA bf16 autocast for
the MLP matmuls, which doesn't apply here (CPU-first port, see train.py).
"""
from __future__ import annotations

import torch
import torch.nn as nn

from quadruped_envs.supermimic.mdp import INV_DIM, NUM_JOINTS, XI_DIM, MatState
from quadruped_envs.supermimic import rotations as rot


def mlp(inp: int, hidden: list, out: int, act=nn.ELU, layer_norm: bool = False) -> nn.Sequential:
    layers, d = [], inp
    for h in hidden:
        layers.append(nn.Linear(d, h))
        if layer_norm:
            layers.append(nn.LayerNorm(h))
        layers.append(act())
        d = h
    layers.append(nn.Linear(d, out))
    return nn.Sequential(*layers)


class RunningNorm(nn.Module):
    """Running mean/std normalizer; updated from data without gradients, frozen after ``max_count``."""

    def __init__(self, dim: int, max_count: float = 2e8, eps: float = 1e-2):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.zeros(()))
        self.max_count, self.eps = max_count, eps

    @torch.no_grad()
    def update(self, x: torch.Tensor):
        x = x.reshape(-1, x.shape[-1]).float()
        if self.count >= self.max_count:
            return
        n = x.shape[0]
        m, v = x.mean(0), x.var(0, unbiased=False)
        tot = self.count + n
        delta = m - self.mean
        self.mean += delta * n / tot
        self.var = (self.var * self.count + v * n + delta**2 * self.count * n / tot) / tot
        self.count = tot

    def forward(self, x):
        return (x - self.mean) / torch.sqrt(self.var + self.eps**2)

    def std(self):
        return torch.sqrt(self.var + self.eps**2)


WM_OUT = 3 + 3 + 3 + 3 + NUM_JOINTS + NUM_JOINTS  # dp, dtheta, dv, dw, dq, dqd

AMP_DTYPE: torch.dtype | None = None


def run_mlp(net: nn.Module, x: torch.Tensor) -> torch.Tensor:
    if AMP_DTYPE is None:
        return net(x)
    with torch.autocast("cuda", dtype=AMP_DTYPE):
        return net(x).float()


class WorldModel(nn.Module):
    """Deterministic dynamics s_{t+1} = f(s_t, a_t) predicting frame-local state increments.

    Also carries an optional learned termination head (paper's `d̂t = g_ω(ŝt, gt)`), trained via
    BCE against the real analytic hard-termination label (mdp.py's compute_termination) -- see
    train.py's --learned_termination flag and project memory (plan.md Phase 5.2) for why this
    replaces compute_soft_termination_prob's hand-tuned analytic sigmoid once trained and
    validated. A purely ADDITIVE change: an old checkpoint lacking term_head's params loads fine
    via strict=False (the head just starts randomly initialized), no breaking migration needed --
    unlike the planned sim-context vector xi, which changes self.in_dim itself.
    """

    def __init__(self, hidden=(2048, 2048, 2048), goal_dim: int = 0):
        super().__init__()
        self.in_dim = INV_DIM + NUM_JOINTS + XI_DIM  # invariant_features(S) + action + sim-context xi
        self.net = mlp(self.in_dim, list(hidden), WM_OUT, act=nn.SiLU, layer_norm=True)
        nn.init.zeros_(self.net[-1].weight)
        nn.init.zeros_(self.net[-1].bias)
        self.in_norm = RunningNorm(self.in_dim)
        self.out_norm = RunningNorm(WM_OUT, eps=1e-4)

        self.goal_dim = goal_dim
        if goal_dim > 0:
            term_in = INV_DIM + goal_dim
            self.term_head = mlp(term_in, [256, 128], 1, act=nn.SiLU)
            self.term_norm = RunningNorm(term_in)

    def inputs(self, S: MatState, a: torch.Tensor, xi: torch.Tensor) -> torch.Tensor:
        from quadruped_envs.supermimic.mdp import invariant_features
        return torch.cat((invariant_features(S), a, xi), -1)

    @staticmethod
    def targets(S: MatState, S1: MatState) -> torch.Tensor:
        """Frame-local increments from S to S1 (the regression targets)."""
        p, R, v, w, q, qd = S
        p1, R1, v1, w1, q1, qd1 = S1
        H = rot.yaw_mat(R)
        Rt = R.transpose(-1, -2)
        return torch.cat((
            (H.transpose(-1, -2) @ (p1 - p).unsqueeze(-1)).squeeze(-1),
            rot.so3_log(Rt @ R1),
            (Rt @ (v1 - v).unsqueeze(-1)).squeeze(-1),
            (Rt @ (w1 - w).unsqueeze(-1)).squeeze(-1),
            q1 - q,
            qd1 - qd,
        ), -1)

    def step(self, S: MatState, a: torch.Tensor, xi: torch.Tensor) -> MatState:
        p, R, v, w, q, qd = S
        y = run_mlp(self.net, self.in_norm(self.inputs(S, a, xi)))
        d = y * self.out_norm.std() + self.out_norm.mean
        H = rot.yaw_mat(R)
        dp, dth, dv, dw, dq, dqd = d.split((3, 3, 3, 3, NUM_JOINTS, NUM_JOINTS), -1)
        p1 = p + (H @ dp.unsqueeze(-1)).squeeze(-1)
        R1 = R @ rot.so3_exp(dth)
        v1 = v + (R @ dv.unsqueeze(-1)).squeeze(-1)
        w1 = w + (R @ dw.unsqueeze(-1)).squeeze(-1)
        return MatState(p1, R1, v1, w1, q + dq, qd + dqd)

    def termination_logit(self, S: MatState, g: torch.Tensor) -> torch.Tensor:
        """Raw (pre-sigmoid) termination logit from state S and goal g (same K-future-frame
        goal vector already fed to the Policy) -- paper's d_hat_t = g_omega(s_hat_t, g_t). Call
        sigmoid() on the result for a probability, or use with
        binary_cross_entropy_with_logits for training (more numerically stable than a separate
        sigmoid+BCE). Only valid if constructed with goal_dim > 0."""
        from quadruped_envs.supermimic.mdp import invariant_features
        x = torch.cat((invariant_features(S), g), -1)
        return run_mlp(self.term_head, self.term_norm(x)).squeeze(-1)

    @staticmethod
    def state_error(S_hat: MatState, S: MatState) -> torch.Tensor:
        """Frame-local state error components (same layout as ``targets``)."""
        p, R, v, w, q, qd = S
        ph, Rh, vh, wh, qh, qdh = S_hat
        H = rot.yaw_mat(R)
        Rt = R.transpose(-1, -2)
        return torch.cat((
            (H.transpose(-1, -2) @ (ph - p).unsqueeze(-1)).squeeze(-1),
            rot.so3_log(Rt @ Rh),
            (Rt @ (vh - v).unsqueeze(-1)).squeeze(-1),
            (Rt @ (wh - w).unsqueeze(-1)).squeeze(-1),
            qh - q,
            qdh - qd,
        ), -1)


class Policy(nn.Module):
    """a = dec(enc(g), o_{t-hist+1:t}) [+ ref_pd bias]; deterministic, latent z in R^32 bounded
    by tanh.

    ``ref_pd`` (optional, see train.py's --pi_ref_pd): a reference-tracking bias added to the
    decoder's own output, so the network only has to learn the CORRECTION to a baseline
    tracking behavior instead of tracking the reference from scratch -- ported from blinkmimic
    (a more mature SuperMimic follow-up project)'s `ref_skip`, but NOT a literal copy: their
    formula is `a = dec(...) + (q_ref - q0) / alpha`, a bias that's CONSTANT regardless of the
    robot's current state -- valid for them because their actuator is PD-POSITION control (the
    actuator's own PD loop supplies the state feedback a constant target implies). This
    project's actuator is direct TORQUE control (od_motor.py's TORQUE_MULTIPLIER), so a
    state-independent bias would just be a fixed offset torque with no actual tracking
    behavior at all. Adapted instead into an explicit PD law evaluated on the robot's own
    (noisy, observed) current state -- `bias = clip((kp*(q_ref - q) - kd*qd) / torque_scale,
    -clip, clip)` -- which is what actually reproduces the "free tracking signal" blinkmimic
    gets for free from its own PD actuator. `kp`/`kd` are real-but-UNVALIDATED starting
    guesses (derived from this robot's own static-stance torque budget, ~4.5-14 Nm across a
    4-leg/2-leg stance per the project's earlier torque-limit derivation -- not from a full
    dynamics analysis), same "needs its own empirical tuning pass" status as this project's
    other hand-calibrated constants."""

    def __init__(self, obs_dim: int, goal_dim: int, hist: int = 10, z_dim: int = 32, ref_pd=None):
        super().__init__()
        self.hist = hist
        self.enc = mlp(goal_dim, [2048, 1024, 512, 256], z_dim)
        self.dec = mlp(z_dim + obs_dim * hist, [2048, 1024, 512, 256], NUM_JOINTS)
        with torch.no_grad():
            self.dec[-1].weight.mul_(0.01)
            self.dec[-1].bias.zero_()
        self.obs_norm = RunningNorm(obs_dim)
        self.goal_norm = RunningNorm(goal_dim)
        self.ref_pd = ref_pd  # (kp, kd, torque_scale, clip) or None -- see class docstring
        if ref_pd is not None:
            from quadruped_envs.supermimic.mdp import DEFAULT_Q
            self.register_buffer("_ref_pd_default_q", DEFAULT_Q.clone())

    def forward(self, obs_hist: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """obs_hist: (B, hist, obs_dim) noisy observations, newest last."""
        z = torch.tanh(run_mlp(self.enc, self.goal_norm(goal)))
        o = self.obs_norm(obs_hist).flatten(-2)
        a = run_mlp(self.dec, torch.cat((z, o), -1))
        if self.ref_pd is not None:
            from quadruped_envs.supermimic.motion import GOAL_Q_OFFSET
            kp, kd, torque_scale, clip = self.ref_pd
            # obs_hist's own layout (mdp.py's obs()): [:NUM_JOINTS] = q-default_q,
            # [NUM_JOINTS:2*NUM_JOINTS] = qd -- read from the RAW (pre-obs_norm) history so the
            # PD law sees physically-meaningful units, not normalized ones.
            q = obs_hist[..., -1, :NUM_JOINTS] + self._ref_pd_default_q
            qd = obs_hist[..., -1, NUM_JOINTS:2 * NUM_JOINTS]
            q_ref = goal[..., GOAL_Q_OFFSET:GOAL_Q_OFFSET + NUM_JOINTS]
            bias = (kp * (q_ref - q) - kd * qd) / torque_scale
            a = a + bias.clamp(-clip, clip)
        return a


class Critic(nn.Module):
    def __init__(self, in_dim: int):
        super().__init__()
        self.net = mlp(in_dim, [256, 128], 1)
        self.norm = RunningNorm(in_dim)

    def forward(self, x):
        return run_mlp(self.net, self.norm(x)).squeeze(-1)
