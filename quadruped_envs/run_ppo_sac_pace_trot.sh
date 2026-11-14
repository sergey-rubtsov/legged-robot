#!/bin/bash
# Sequential PPO then SAC training on pace+trot only, matched to SuperMimic's reward/control_mode
# defaults, for a genuine apples-to-apples training-speed comparison (see README.md "Comparing
# PPO/SAC vs SuperMimic"). Sequential (not parallel) to avoid oversubscribing CPU against the
# already-running pd_full_run1 SuperMimic process.
set -e
cd "$(dirname "$0")"

../.venv/bin/python3 train.py --algo ppo --env OpenDynamicImitationEnv-v0 \
  -n 40000000 --save-freq 2000000 --eval-freq 100000 \
  --env-kwargs render:False 'motion_names:"pace,trot"'

../.venv/bin/python3 train.py --algo sac --env OpenDynamicImitationEnv-v0 \
  -n 40000000 --save-freq 2000000 --eval-freq 100000 \
  --env-kwargs render:False 'motion_names:"pace,trot"'
