"""Optimizer: Muon (hidden MLP matrices) + Adam (everything else), ported from blinkmimic (a
more mature SuperMimic follow-up project)'s `optim.py`'s `MuonAdam` class -- its own version also
handles CUDA-graph capture (a tensor learning rate, a hand-rolled gated step skipping bad
updates inside a captured graph) that's irrelevant here (this project runs eager-mode PyTorch on
CPU/MPS, no CUDA graphs) -- stripped down to just the portable idea: route each MLP's HIDDEN
Linear layers (every Linear except the first and last of each) to Muon's Newton-Schulz
orthogonalized update, route everything else (each MLP's first/last Linear, all 1D params --
biases, LayerNorm weights, RunningNorm buffers aren't parameters at all) to plain Adam, and
reuse ONE learning rate for both via Muon's `adjust_lr_fn="match_rms_adamw"` (rescales Muon's
step to Adam's own update RMS, so no separate Muon LR needs tuning -- Liu et al. 2025).

Confirmed before relying on it: `torch.optim.Muon` exists in this project's installed PyTorch
(2.14.1) and its Newton-Schulz orthogonalization runs on MPS without error -- not something to
assume, MPS op coverage is often incomplete for less common ops.

**CPU is NOT viable for this optimizer -- confirmed directly, don't test --optimizer muon on
--device cpu.** A single `opt.step()` call measured ~40 SECONDS on CPU for this project's
Policy network (batch 128) -- consistently, not just a one-time warm-up cost -- vs. ~25-35ms on
MPS (after a ~1.3s one-time warm-up). Muon's reference implementation is evidently GPU-first
(blinkmimic's own usage targets an RTX 4090 with CUDA graphs); whatever makes its CPU path this
slow is specific to Muon's own kernel, not this project's networks or PyTorch install in general
(plain Adam, and Muon on MPS, are both completely normal speed on the exact same networks). Any
smoke test of `--optimizer muon` MUST pass `--device mps`, or it will look hung, not just slow.
"""
from __future__ import annotations

import torch
import torch.nn as nn


def _linears(m: nn.Module) -> list[nn.Linear]:
    return [x for x in m.modules() if isinstance(x, nn.Linear)]


class MuonAdam:
    """Muon on the hidden-layer weight matrices of `mlps` (every Linear in each MLP except its
    first and last), Adam on everything else (first/last Linear of each MLP, all 1D params, and
    any parameter not belonging to one of `mlps` at all -- e.g. a module with no Linear-stack
    hidden layers to speak of). A near-drop-in replacement for a single `torch.optim.Adam`
    instance: same `lr`/`param_groups`/`zero_grad`/`step`/`state_dict`/`load_state_dict`
    interface, so train.py's existing per-iteration `gparam["lr"] = lr` schedule-setting loop
    works unchanged against `.param_groups`.
    """

    def __init__(self, model: nn.Module, mlps: list[nn.Module], lr: float, betas=(0.9, 0.999)):
        hidden = {id(l.weight) for m in mlps for l in _linears(m)[1:-1]}
        muon_params = [p for p in model.parameters() if id(p) in hidden]
        adam_params = [p for p in model.parameters() if id(p) not in hidden and p.requires_grad]
        # Muon requires at least one param; a model with no eligible hidden matrices (e.g. every
        # MLP passed has <=2 Linear layers total) would otherwise construct an empty optimizer.
        self._has_muon = len(muon_params) > 0
        if self._has_muon:
            self.muon = torch.optim.Muon(muon_params, lr=lr, weight_decay=0.0, adjust_lr_fn="match_rms_adamw")
        self.adam = torch.optim.Adam(adam_params, lr=lr, betas=betas)

    @property
    def param_groups(self):
        # A property, not a cached list: Optimizer.load_state_dict replaces each group dict
        # in place, so a list built once in __init__ would go stale after --resume.
        return ([g for g in self.muon.param_groups] if self._has_muon else []) + self.adam.param_groups

    def zero_grad(self, set_to_none: bool = True):
        if self._has_muon:
            self.muon.zero_grad(set_to_none=set_to_none)
        self.adam.zero_grad(set_to_none=set_to_none)

    def step(self):
        if self._has_muon:
            self.muon.step()
        self.adam.step()

    def state_dict(self):
        return {"muon": self.muon.state_dict() if self._has_muon else None, "adam": self.adam.state_dict()}

    def load_state_dict(self, sd: dict):
        if self._has_muon and sd.get("muon") is not None:
            self.muon.load_state_dict(sd["muon"])
        self.adam.load_state_dict(sd["adam"])
