from quadruped_envs.quadruped import OpenDynamicQuadrupedEnv
from gym.envs.registration import register

register(
    id='OpenDynamicQuadrupedEnv-v0',
    entry_point='quadruped_envs:OpenDynamicQuadrupedEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
)

# register(
#     id='OpenDynamicQuadrupedDuckEnv-v0',
#     entry_point='quadruped_envs:OpenDynamicQuadrupedEnv',
#     max_episode_steps=1000,
#     reward_threshold=5.0,
# )
