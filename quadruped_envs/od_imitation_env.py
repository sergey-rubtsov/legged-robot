"""This file implements the gym environment of Open Dynamics robot.
"""
import gym
import os, inspect, sys

from quadruped_envs.imitation_wrapper_env import od_gym_config, od_gym_env, \
    simple_openloop, observation_dictionary_to_array_wrapper, \
    trajectory_generator_wrapper_env, imitation_wrapper_env, imitation_task
from motion_imitation.envs.sensors import sensor_wrappers, robot_sensors, environment_sensors
from motion_imitation.envs.utilities import controllable_env_randomizer_from_config
from quadruped_envs import od


def build_imitation_env(motion_files,
                        num_parallel_envs,
                        enable_rendering,
                        enable_randomizer=False,
                        robot_class=od.OD,
                        trajectory_generator=simple_openloop.SimpleNormalizer()):
    assert len(motion_files) > 0

    curriculum_steps = 30
    if enable_rendering:
        curriculum_episode_length_start = 300
    else:
        curriculum_episode_length_start = 20
    curriculum_episode_length_end = 600

    sim_params = od_gym_config.SimulationParameters()
    sim_params.enable_rendering = enable_rendering
    sim_params.allow_knee_contact = True

    gym_config = od_gym_config.LocomotionGymConfig(simulation_parameters=sim_params)

    sensors = [
        sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=robot_sensors.MotorAngleSensor(
            upper_bound=od.UPPER_BOUND,
            lower_bound=od.LOWER_BOUND,
            num_motors=od.NUM_MOTORS,
            noisy_reading=False),
            num_history=3),
        sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=robot_sensors.IMUSensor(
            noisy_reading=False), num_history=3),
        sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=environment_sensors.LastActionSensor(
            upper_bound=od.UPPER_BOUND,
            lower_bound=od.LOWER_BOUND,
            num_actions=od.NUM_MOTORS), num_history=3)
    ]

    task = imitation_task.ImitationTask(ref_motion_filenames=motion_files,
                                        enable_cycle_sync=True,
                                        tar_frame_steps=[1, 2, 10, 20],
                                        enable_rand_init_time=True,
                                        ref_state_init_prob=1,
                                        warmup_time=0.25)

    randomizers = []
    if enable_randomizer:
        randomizer = controllable_env_randomizer_from_config.ControllableEnvRandomizerFromConfig(verbose=False)
        randomizers.append(randomizer)

    env = od_gym_env.LocomotionGymEnv(gym_config=gym_config, robot_class=robot_class,
                                      env_randomizers=randomizers, robot_sensors=sensors, task=task)

    env = observation_dictionary_to_array_wrapper.ObservationDictionaryToArrayWrapper(env)
    env = trajectory_generator_wrapper_env.TrajectoryGeneratorWrapperEnv(env, trajectory_generator=trajectory_generator)

    env = imitation_wrapper_env.ImitationWrapperEnv(env,
                                                    episode_length_start=curriculum_episode_length_start,
                                                    episode_length_end=curriculum_episode_length_end,
                                                    curriculum_steps=curriculum_steps,
                                                    num_parallel_envs=num_parallel_envs)
    return env


class OpenDynamicImitationEnv(gym.Env):
    """The gym environment is made on the basis of the existing environment of
  the minitaur, Laikago and A1, since the Open Dynamics robot is very similar to them.

  It simulates the locomotion of a Open Dynamics robot, a quadruped robot with
  12 DoF. The state space include the angles, velocities and torques for all
  the motors and the action space is the desired motor angle for each motor.

  """

    metadata = {'render.modes': ['rgb_array']}

    def __init__(self,
                 render=True):
        # gallop = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/od/gallop.txt"
        # idle = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/od/idle.txt"
        # left_turn = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/od/left.txt"
        pace = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/od/pace.txt"
        # right_turn = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/od/right.txt"
        # trot = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/od/trot.txt"
        motion_files = [pace]
        num_procs = 8  # 1 by default
        enable_env_rand = False
        self._env = build_imitation_env(motion_files=motion_files,
                                        num_parallel_envs=num_procs,
                                        robot_class=od.OD,
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
