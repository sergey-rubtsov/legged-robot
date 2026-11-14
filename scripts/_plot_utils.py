"""Shared helpers for scripts/plot_supermimic_train.py and scripts/compare_training_speed.py."""
import json
import os

import numpy as np


def sb3_abs_start(path):
    """Absolute epoch time (seconds) an SB3 experiment actually started, read directly from its
    Monitor file headers -- works even if the process has since exited (unlike `ps`)."""
    from stable_baselines3.common.monitor import get_monitor_files

    starts = []
    for file_name in get_monitor_files(path):
        with open(file_name) as fh:
            starts.append(json.loads(fh.readline()[1:])["t_start"])
    if not starts:
        raise FileNotFoundError(f"no monitor files found under {path}")
    return min(starts)


def supermimic_abs_start(csv_path):
    """Absolute epoch time (seconds) a SuperMimic run actually started, derived from the log
    file's own mtime and its last logged wall_h -- works even if the process has since exited."""
    df_tail_wall_h = _last_wall_h(csv_path)
    return os.path.getmtime(csv_path) - df_tail_wall_h * 3600


def _last_wall_h(csv_path):
    import pandas as pd

    df = pd.read_csv(csv_path, usecols=["wall_h"])
    return float(df["wall_h"].iloc[-1])


def build_shared_warp(all_abs_seconds, factor=3.0):
    """Given the UNION of every series' absolute timestamps (seconds since the epoch), build
    ONE shared monotonic warp (raw absolute seconds -> compressed seconds-from-the-earliest-
    point) that collapses any gap in the COMBINED timeline above `factor` times the typical
    combined spacing.

    Applying the SAME returned function to every series (rather than each series compressing
    its own gaps independently) is what keeps a shared stall collapsing identically everywhere
    -- otherwise two series sampled at different rates cap their own gaps by different amounts,
    and a stall that hit everyone at once ends up looking like it happened at different times/
    for different durations on each curve, which is actively misleading, not just ugly.
    """
    t = np.unique(np.sort(np.asarray(all_abs_seconds, dtype=float)))
    if len(t) < 3:
        origin = t[0] if len(t) else 0.0
        return lambda x: np.asarray(x, dtype=float) - origin
    diffs = np.diff(t)
    positive = diffs[diffs > 1e-9]
    cap = np.median(positive) * factor if len(positive) else np.inf
    capped = np.minimum(diffs, cap)
    compressed = np.concatenate([[0.0], np.cumsum(capped)])

    def warp(x):
        return np.interp(np.asarray(x, dtype=float), t, compressed)

    return warp


def compress_stalls(x, factor=3.0):
    """Single-series version of the same idea, for when there's only one timeline to align (no
    cross-source absolute-time alignment needed) -- see build_shared_warp for the multi-series
    case, which this is a special case of."""
    return build_shared_warp(x, factor)(x)
