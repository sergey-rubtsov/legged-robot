For installing run

pip install -e .

####Due to the limitations of the current framework implementation, the training cannot be observed in real time. 
####Therefore, rendering must be switched off during training with argument --env-kwargs render:False 

Examples of commands:

To run the ppo and sac algorithm during 1000_000 steps and saving an intermediate result every 100_000 steps:
python train.py --algo sac --env OpenDynamicImitationEnv-v0 -n 1000000 --save-freq 100000 --env-kwargs render:False
python train.py --algo ppo --env OpenDynamicImitationEnv-v0 -n 1000000 --save-freq 100000 --env-kwargs render:False

To test the best trained agent after the 5th experiment of the ppo algorithm:
python enjoy.py --algo ppo --env OpenDynamicQuadrupedEnv-v0 -f logs/ --exp-id 5 --load-best --env-kwargs render:True

Plot the agent training process saved in the quadruped_envs/logs/ folder
python -m scripts.plot_train --algo ppo --env OpenDynamicImitationEnv-v0 -f quadruped_envs/logs/

Plot the process of the ppo and sac algorithms on one graph
python -m scripts.all_plots --algos ppo sac --env OpenDynamicImitationEnv -f ../quadruped_envs/logs/