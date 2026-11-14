"""
Overlay PPO/SAC (SB3) and SuperMimic training curves on one plot, to compare how fast
each actually learns -- in both real environment steps and wall-clock time.

Reward is put on a common footing as MEAN REWARD PER REAL ENV STEP:
  - SB3 side: each Monitor episode's (r, l) -> r / l, then rolling-mean smoothed.
  - SuperMimic side: train_log.csv's `sim_rew` column already IS a per-step mean
    (computed in train.py's collect() as the running mean of real per-step reward
    across all parallel workers since the last log) -- not an episodic return, so
    no division is needed, just the same rolling-mean smoothing for comparability.

Both sides read the exact same imitation reward terms with the same weights
(pose/velocity/end-effector/root-pose/root-velocity, see imitation_task.py vs
mdp.py's compute_reward) -- confirmed by direct comparison, not assumed. One known
asymmetry: the SuperMimic side also subtracts an action-rate penalty
(mdp.py's ACTION_RATE_WEIGHT) that the legacy PPO/SAC env does not apply, so its
reward will run slightly lower for equally-good tracking, not just converge slower.
Physics also differs (SuperMimic's default --control_mode position + joint armature +
sim-context domain randomization vs PPO/SAC's plain torque control) -- so a "SuperMimic
is Nx faster/slower" reading from this plot describes the two actual training setups as
configured, not an isolated algorithm-only effect. Treat it as a practical comparison,
not a controlled ablation.

Example:
    python3 -m scripts.compare_training_speed \
        --supermimic quadruped_envs/supermimic/logs/pd_full_run1:SuperMimic \
        --sb3 quadruped_envs/logs/ppo/OpenDynamicImitationEnv-v0_1:PPO \
        --sb3 quadruped_envs/logs/sac/OpenDynamicImitationEnv-v0_1:SAC

A `--sb3` run can list MULTIPLE experiment dirs, comma-separated, as one logical series:
    --sb3 "logs/sac/OpenDynamicImitationEnv-v0_1,logs/sac/OpenDynamicImitationEnv-v0_2:SAC"
This stitches a `train.py --trained-agent ...`-continued run back into one continuous curve --
the legacy zoo harness always starts a brand-new numbered experiment dir (with its own fresh
Monitor files/clock) on every invocation, even when continuing training from a checkpoint, so
without stitching, a resumed run's chart would start back over at x=0/0 steps instead of
continuing from where the previous chunk left off.
"""
import argparse
import os

import numpy as np
import pandas as pd
import seaborn
from matplotlib import pyplot as plt
from stable_baselines3.common.monitor import load_results

from scripts._plot_utils import build_shared_warp, sb3_abs_start, supermimic_abs_start

seaborn.set()

parser = argparse.ArgumentParser(
    "Compare PPO/SAC vs SuperMimic training speed (reward per real env step)"
)
parser.add_argument(
    "--supermimic", action="append", default=[], dest="supermimic_runs",
    help="SuperMimic run log dir or train_log.csv path, as PATH[:LABEL]. Repeatable.",
)
parser.add_argument(
    "--sb3", action="append", default=[], dest="sb3_runs",
    help="SB3 experiment dir(s) (each containing *.monitor.csv) as PATH[:LABEL]. Comma-separate "
    "multiple dirs to stitch a --trained-agent-continued run into one series. Repeatable.",
)
parser.add_argument("-x", "--x-axis", choices=["steps", "time"], default="steps")
parser.add_argument("-w", "--window", type=int, default=20, help="Rolling-mean window size")
parser.add_argument("--figsize", nargs=2, type=float, default=[7.5, 5.0])
parser.add_argument("--fontsize", type=int, default=14)
parser.add_argument("--out", type=str, default=None, help="Save to this path instead of opening a window")
parser.add_argument(
    "--collapse-stalls", dest="collapse_stalls", action=argparse.BooleanOptionalAction, default=True,
    help="On --x-axis time, compress abnormally large gaps (a system sleep/stall) down to a few "
    "times the typical sample spacing, so a multi-hour stall doesn't squash the rest of the "
    "curve into an unreadable sliver. ONE shared warp is computed from the union of every "
    "series' absolute timestamps and applied identically to all of them, so a stall that hit "
    "everyone at once still collapses by the same amount at the same point for every curve -- "
    "each series compressing its own gaps independently would desync them instead (confirmed "
    "wrong this way: two runs that had actually run for the same wall-clock duration ended up "
    "visibly offset). Default on; pass --no-collapse-stalls to see raw walltime.",
)
parser.add_argument(
    "--stall-factor", type=float, default=3.0,
    help="How many times the typical gap a stall gets capped at (smaller = more aggressive "
    "compression). Only used with --collapse-stalls.",
)
parser.add_argument(
    "--clip-to-shortest", action="store_true",
    help="Trim every curve's x-range to the shortest series' own final x value, so all curves "
    "end at the same point instead of some just stopping mid-chart while others run on past "
    "them -- avoids the visual impression that a shorter run is somehow worse at that same x "
    "position just because the other runs happen to have more (possibly less interesting) data.",
)
args = parser.parse_args()

if not args.supermimic_runs and not args.sb3_runs:
    parser.error("pass at least one --supermimic or --sb3 run")

x_label = "Real environment steps" if args.x_axis == "steps" else "Active training time (hours, each run's own clock)"

plt.figure("Training speed comparison", figsize=args.figsize)
plt.title("Mean reward per real environment step", fontsize=args.fontsize)
plt.xlabel(x_label, fontsize=args.fontsize)
plt.ylabel("Mean reward / step", fontsize=args.fontsize)

# Pass 1: load every series' raw data, as a list of (abs_seconds_or_steps, y) CHUNKS per label
# (normally just one chunk; multiple when --sb3 lists several comma-separated dirs to stitch).
# For the time axis we need ABSOLUTE epoch seconds (not each chunk's own process-relative clock)
# so every chunk/series shares one real-world timeline for stall detection -- otherwise runs
# started at different moments just look offset from each other for no reason, and (see
# --collapse-stalls help) independently-compressed stalls desync further still.
sb3_series, supermimic_series = [], []

for spec in args.sb3_runs:
    paths_part, _, label = spec.partition(":")
    paths = [p.strip() for p in paths_part.split(",") if p.strip()]
    label = label or os.path.basename(paths[0].rstrip("/"))
    chunks = []
    for path in paths:
        data_frame = load_results(path)
        r = data_frame["r"].to_numpy()
        l = data_frame["l"].to_numpy()
        per_step = r / np.maximum(l, 1)
        if args.x_axis == "steps":
            x = np.cumsum(l)
        else:
            x = sb3_abs_start(path) + data_frame["t"].to_numpy()
        chunks.append((x, per_step))
    sb3_series.append((label, chunks))

for spec in args.supermimic_runs:
    paths_part, _, label = spec.partition(":")
    paths = [p.strip() for p in paths_part.split(",") if p.strip()]
    label = label or os.path.basename(os.path.dirname(paths[0])) or paths[0]
    chunks = []
    for path in paths:
        if os.path.isdir(path):
            path = os.path.join(path, "train_log.csv")
        df = pd.read_csv(path)
        if args.x_axis == "steps":
            x = df["sim_steps"].to_numpy()
        else:
            x = supermimic_abs_start(path) + df["wall_h"].to_numpy() * 3600
        chunks.append((x, df["sim_rew"].to_numpy()))
    supermimic_series.append((label, chunks))

# Pass 2: build ONE shared stall-compressing warp from every chunk's absolute timestamps
# combined (so a stall that hit several runs at once is excised by the SAME real duration from
# all of them). Every series (and every chunk within it) is then re-anchored to start at x=0,
# and multiple chunks of one series are stitched with NO gap between them (the real-world pause
# between stopping one chunk and starting the next -- e.g. to apply a fix, or simply because
# `train.py --trained-agent` always starts a fresh experiment dir -- is exactly the kind of dead
# time this whole active-duration framing already excludes elsewhere).
if args.x_axis == "time" and args.collapse_stalls:
    all_abs = np.concatenate(
        [x for _, chunks in sb3_series for x, _ in chunks]
        + [x for _, chunks in supermimic_series for x, _ in chunks]
    )
    warp = build_shared_warp(all_abs, args.stall_factor)
else:
    warp = lambda x: x  # noqa: E731  (raw seconds; still re-anchored to each chunk's own start below)


def stitch(chunks):
    """Each chunk re-anchored to its own start (own-clock active duration/steps), then
    concatenated with zero gap between consecutive chunks."""
    xs, ys, offset = [], [], 0.0
    for x, y in chunks:
        if args.x_axis == "time":
            x = (warp(x) - warp(x)[0]) / 3600.0
        else:
            x = x - x[0]
        xs.append(x + offset)
        ys.append(y)
        offset += x[-1] if len(x) else 0.0
    return np.concatenate(xs), np.concatenate(ys)


plotted = []
for label, chunks in sb3_series:
    x, per_step = stitch(chunks)
    y = pd.Series(per_step).rolling(args.window, min_periods=1).mean().to_numpy()
    plotted.append((f"{label} (SB3)", x, y))

for label, chunks in supermimic_series:
    x, sim_rew = stitch(chunks)
    y = pd.Series(sim_rew).rolling(args.window, min_periods=1).mean().to_numpy()
    plotted.append((f"{label} (SuperMimic)", x, y))

if args.clip_to_shortest:
    x_max = min(x[-1] for _, x, _ in plotted if len(x))
    plotted = [(label, x[x <= x_max], y[x <= x_max]) for label, x, y in plotted]

for label, x, y in plotted:
    plt.plot(x, y, linewidth=2, label=label)

plt.legend()
plt.tight_layout()
if args.out:
    plt.savefig(args.out, dpi=150)
    print(f"Saved to {args.out}")
else:
    plt.show()
