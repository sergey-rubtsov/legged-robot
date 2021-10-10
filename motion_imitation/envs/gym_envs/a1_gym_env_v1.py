"""Wrapper to make the a1 environment suitable for OpenAI gym."""
import gym
import os, inspect, sys

from motion_imitation.envs import env_builder
from motion_imitation.robots import a1
from motion_imitation.robots import robot_config


class A1GymEnv(gym.Env):
  """A1 environment that supports the gym interface."""
  metadata = {'render.modes': ['rgb_array']}

  def __init__(self,
               action_limit=(0.75, 0.75, 0.75),
               render=True,
               on_rack=False):
    motion_file = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/a1/pace.txt"
    num_procs = 8
    enable_env_rand = True
    self._env = env_builder.build_imitation_env(motion_files=[motion_file],
                                                robot_class=a1.A1,
                                                mode='train',
                                                num_parallel_envs=num_procs,
                                                enable_randomizer=enable_env_rand,
                                                enable_rendering=render)
    self.observation_space = self._env.observation_space
    self.action_space = self._env.action_space

  def step(self, action):
    return self._env.step(action)

  def reset(self):
    return self._env.reset()

  def close(self):
    self._env.close()

  def render(self, mode):
    return self._env.render(mode)

  def __getattr__(self, attr):
    return getattr(self._env, attr)

  def __len__(self):
      return 1
