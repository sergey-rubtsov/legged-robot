from quadruped_envs.simple.quadruped_env import OpenDynamicQuadrupedEnv
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
    id='A1GymEnv-v1',
    entry_point='motion_imitation.envs.gym_envs.a1_gym_env_v1:A1GymEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
)

register(
    id='A1GymEnv-v0',
    entry_point='motion_imitation.envs.gym_envs.a1_gym_env_v1:A1GymEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
)