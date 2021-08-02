pip install -e .

python train.py --algo sac --env OpenDynamicQuadrupedEnv-v0 -n 1000000 --save-freq 100000
python train.py --algo ppo --env OpenDynamicQuadrupedEnv-v0 -n 1000000 --save-freq 100000
python enjoy.py --algo ppo --env OpenDynamicQuadrupedEnv-v0 -f logs/ --exp-id 5 --load-best  

python -m scripts.plot_train --algo ppo --env OpenDynamicQuadrupedEnv-v0 -f quadruped_envs/logs/
python -m scripts.all_plots --algos ppo sac --env HumanoidDeepMimicWalkBulletEnv-v1 -f logs/

python -m utils.record_video --algo ppo --env OpenDynamicQuadrupedEnv-v0 -f logs/ -n 1000 --load-best

python -m motion_imitation.examples.test_env_gui --robot_type=A1 --motor_control_mode=Torque --on_rack=True

python enjoy.py --algo ppo --env A1GymEnv-v0 -f logs/ --exp-id 3 --load-best --env-kwargs render:True
python train.py --algo ppo --env A1GymEnv-v0 -n 100000 --save-freq 100000 --env-kwargs render:False