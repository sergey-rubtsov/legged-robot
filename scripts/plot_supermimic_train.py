"""
Plot SuperMimic training curves from one or more `train_log.csv` run logs.

Counterpart to scripts/plot_train.py (PPO/SAC), but reads the SuperMimic port's own
plain-CSV log (quadruped_envs/supermimic/logs/<run>/train_log.csv) instead of an SB3
Monitor directory -- there's no episodic Monitor file on the SuperMimic side, since
`collect()` logs a running mean reward per real env-step across all parallel workers,
not per-episode.

Example:
    python3 -m scripts.plot_supermimic_train \
        --run quadruped_envs/supermimic/logs/pd_full_run1:pd_full_run1 \
        --run quadruped_envs/supermimic/logs/pace_run13:pace_run13 \
        --x-axis steps --y-axis reward
"""
import argparse
import os

import pandas as pd
import seaborn
from matplotlib import pyplot as plt

from scripts._plot_utils import compress_stalls

seaborn.set()

Y_COLUMNS = {
    "reward": ("sim_rew", "Real-env reward (mean per step)"),
    "wm_loss": ("wm_loss", "World-model loss"),
    "imag_reward": ("imag_rew", "Imagined-rollout reward (training objective)"),
    "survival": ("imag_survival", "Imagined survival probability"),
    "v_loss": ("v_loss", "Critic (value) loss"),
}
X_COLUMNS = {
    "steps": ("sim_steps", "Real environment steps"),
    "time": ("wall_h", "Walltime (hours)"),
    "iteration": ("iteration", "Iteration"),
}

parser = argparse.ArgumentParser("Plot SuperMimic training curves from train_log.csv")
parser.add_argument(
    "--run",
    action="append",
    required=True,
    dest="runs",
    help="A run log dir or train_log.csv path, optionally with a display label: "
    "PATH[:LABEL]. Repeat for multiple runs on one plot.",
)
parser.add_argument("-x", "--x-axis", choices=list(X_COLUMNS), default="steps")
parser.add_argument("-y", "--y-axis", choices=list(Y_COLUMNS), default="reward")
parser.add_argument("-w", "--window", type=int, default=20, help="Rolling-mean window (in log rows, not iterations)")
parser.add_argument("--figsize", nargs=2, type=float, default=[6.4, 4.8])
parser.add_argument("--fontsize", type=int, default=14)
parser.add_argument("--out", type=str, default=None, help="Save to this path instead of opening a window")
parser.add_argument(
    "--collapse-stalls", dest="collapse_stalls", action=argparse.BooleanOptionalAction, default=True,
    help="On --x-axis time, compress abnormally large gaps (a system sleep/stall) down to a few "
    "times the typical sample spacing. Default on; pass --no-collapse-stalls to see raw walltime.",
)
parser.add_argument("--stall-factor", type=float, default=3.0, help="See --collapse-stalls.")
args = parser.parse_args()

x_col, x_label = X_COLUMNS[args.x_axis]
y_col, y_label = Y_COLUMNS[args.y_axis]

plt.figure(y_label, figsize=args.figsize)
plt.title(y_label, fontsize=args.fontsize)
plt.xlabel(x_label, fontsize=args.fontsize)
plt.ylabel(y_label, fontsize=args.fontsize)

for spec in args.runs:
    path, _, label = spec.partition(":")
    if os.path.isdir(path):
        path = os.path.join(path, "train_log.csv")
    label = label or os.path.basename(os.path.dirname(path)) or path

    df = pd.read_csv(path)
    y = df[y_col].rolling(args.window, min_periods=1).mean()
    x = df[x_col].to_numpy()
    if args.x_axis == "time" and args.collapse_stalls:
        x = compress_stalls(x, args.stall_factor)
    plt.plot(x, y, linewidth=2, label=label)

plt.legend()
plt.tight_layout()
if args.out:
    plt.savefig(args.out, dpi=150)
    print(f"Saved to {args.out}")
else:
    plt.show()
