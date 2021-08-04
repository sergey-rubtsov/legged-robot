from quadruped_envs.quadruped import OpenDynamicQuadrupedEnv
from gym.envs.registration import register

register(
    id='OpenDynamicQuadrupedEnv-v0',
    entry_point='quadruped_envs:OpenDynamicQuadrupedEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
)

register(
    id='OpenDynamicImitationEnv-v0',
    entry_point='quadruped_envs.od_imitation_env:OpenDynamicImitationEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
)

register(
    id='A1GymEnv-v0',
    entry_point='motion_imitation.envs.gym_envs.a1_gym_env:A1GymEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
)

register(
    id='LocomotionGymEnv-v0',
    entry_point='motion_imitation.envs.locomotion_gym_env:LocomotionGymEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
)