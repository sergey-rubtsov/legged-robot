"""
Plot training reward/success rate
"""
import argparse
import os
import numpy as np
import seaborn
from matplotlib import pyplot as plt
from stable_baselines3.common.monitor import LoadMonitorResultsError, load_results
from stable_baselines3.common.results_plotter import X_EPISODES, X_TIMESTEPS, X_WALLTIME, ts2xy, window_func

# Activate seaborn
seaborn.set()

parser = argparse.ArgumentParser("Gather results, plot training reward/success")
parser.add_argument("-a", "--algo", help="Algorithm to include", type=str, required=True)
parser.add_argument("-e", "--env", help="Environment to include", type=str, required=True)
parser.add_argument("-f", "--exp-folder", help="Folders to include", type=str, required=True)
parser.add_argument("--figsize", help="Figure size, width, height in inches.", nargs=2, type=int, default=[6.4, 4.8])
parser.add_argument("--fontsize", help="Font size", type=int, default=14)
parser.add_argument("-x", "--x-axis", help="X-axis", choices=["steps", "episodes", "time"], type=str, default="steps")
parser.add_argument("-y", "--y-axis", help="Y-axis", choices=["success", "reward"], type=str, default="reward")
parser.add_argument("-w", "--episode-window", help="Rolling window size", type=int, default=100)

args = parser.parse_args()


algo = args.algo
env = args.env
log_path = os.path.join(args.exp_folder, algo)

x_axis = {"steps": X_TIMESTEPS, "episodes": X_EPISODES, "time": X_WALLTIME}[args.x_axis]
x_label = {"steps": "Timesteps", "episodes": "Episodes", "time": "Walltime (in hours)"}[args.x_axis]

y_axis = {"success": "is_success", "reward": "r"}[args.y_axis]
y_label = {"success": "Training Success Rate", "reward": "Training Episodic Reward"}[args.y_axis]


def parse_number(e):
    return int(str(e).split('_')[-1])


dirs = [
    os.path.join(log_path, folder)
    for folder in os.listdir(log_path)
    if (env in folder and os.path.isdir(os.path.join(log_path, folder)))
]
dirs.sort(key=parse_number)

plt.figure(y_label, figsize=args.figsize)
plt.title(y_label, fontsize=args.fontsize)
plt.xlabel(f"{x_label}", fontsize=args.fontsize)
plt.ylabel(y_label, fontsize=args.fontsize)
x_plot = np.array([])
y_mean_plot = np.array([])
data_frame = load_results(dirs[0])
del dirs[0]
for folder in dirs:
    try:
        data_frame = data_frame.append(load_results(folder), ignore_index=True)

    except LoadMonitorResultsError:
        continue
x, y = ts2xy(data_frame, x_axis)
# Do not plot the smoothed curve at all if the timeseries is shorter than window size.
if x.shape[0] >= args.episode_window:
    # Compute and plot rolling mean with window of size args.episode_window
    x, y_mean = window_func(x, y, args.episode_window, np.mean)
    plt.plot(x, y_mean, linewidth=2, label=folder.split("/")[-1])

plt.legend()
plt.tight_layout()
plt.show()
