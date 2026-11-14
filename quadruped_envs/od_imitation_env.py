"""This file implements the gym environment of Open Dynamics robot.
"""
import gym
import numpy as np
import os, inspect, sys

from gym import spaces
from motion_imitation.robots import robot_config

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
                        trajectory_generator=simple_openloop.SimpleNormalizer(),
                        enable_rendering_gui=False,
                        recovery_motion_names=frozenset(),
                        recovery_init_prob=0.0,
                        recovery_sample_prob=0.0,
                        bin_duration=None,
                        motor_control_mode=None):
    assert len(motion_files) > 0

    curriculum_steps = 30
    if enable_rendering:
        curriculum_episode_length_start = 300
    else:
        curriculum_episode_length_start = 20
    curriculum_episode_length_end = 600

    sim_params = od_gym_config.SimulationParameters()
    sim_params.enable_rendering = enable_rendering
    # The on-screen slider panel (reference-opacity, render delay) defaults to hidden
    # (SimulationParameters' own default) regardless of enable_rendering -- pass True to see it.
    sim_params.enable_rendering_gui = enable_rendering_gui
    sim_params.allow_knee_contact = True
    # None (default) keeps SimulationParameters' own default (TORQUE -- this project's original,
    # still-live training convention, see project memory) unchanged; pass
    # robot_config.MotorControlMode.POSITION for the PD-position control path (plan.md's
    # position-control candidate) -- see supermimic/mdp.py's ACTION_POS_SCALE / env.py's worker
    # for where a normalized [-1,1] action actually gets converted into a position target under
    # this mode (SimpleNormalizer's own get_action is a pass-through, it does NOT do this itself).
    if motor_control_mode is not None:
        sim_params.motor_control_mode = motor_control_mode

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
                                        warmup_time=0.25,
                                        recovery_motion_names=recovery_motion_names,
                                        recovery_init_prob=recovery_init_prob,
                                        recovery_sample_prob=recovery_sample_prob,
                                        bin_duration=bin_duration)

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
                 render=True,
                 control_mode="position",
                 action_rate_weight=0.05,
                 motion_names="pace"):
        """control_mode/action_rate_weight default to match the SuperMimic port
        (quadruped_envs/supermimic/mdp.py's --control_mode position + ACTION_RATE_WEIGHT) so a
        PPO/SAC run here is a genuine apples-to-apples comparison, not torque-control-vs-position
        -control with different reward shaping on top. Pass control_mode="torque",
        action_rate_weight=0 via --env-kwargs to restore this project's original behavior.

        motion_names: comma-separated motion basenames (no path/extension), e.g. "pace,trot" --
        mirrors supermimic/train.py's --motion_file convention (jointly tracking several gaits
        via ImitationTask's own ref-motion-list handling), so PPO/SAC can be compared against a
        SuperMimic run trained on the same motion set instead of being stuck on pace.txt only.
        """
        motions_dir = os.path.dirname(sys.modules['__main__'].__file__) + "/data/motions/od/"
        if isinstance(motion_names, str):
            motion_names = [name.strip() for name in motion_names.split(",") if name.strip()]
        motion_files = [motions_dir + name + ".txt" for name in motion_names]
        num_procs = 8  # 1 by default
        enable_env_rand = False
        if control_mode not in ("position", "torque"):
            raise ValueError(f"control_mode must be 'position' or 'torque', got {control_mode!r}")
        self._is_position_mode = control_mode == "position"
        motor_control_mode = robot_config.MotorControlMode.POSITION if self._is_position_mode else None
        self._env = build_imitation_env(motion_files=motion_files,
                                        num_parallel_envs=num_procs,
                                        robot_class=od.OD,
                                        enable_randomizer=enable_env_rand,
                                        enable_rendering=render,
                                        motor_control_mode=motor_control_mode)
        self.observation_space = self._env.observation_space
        self.action_space = self._env.action_space
        if self._is_position_mode:
            # od_gym_env.py's own POSITION-mode action_space is Box(-1, 1) rad around the
            # robot's zero pose (OPTIMAL_ACTION_SPACE_LIMIT=1) -- wider than SuperMimic's
            # calibrated ordinary-gait range (ACTION_POS_SCALE, ~0.31 rad, same zero-centered
            # convention: see mdp.py's DEFAULT_Q=0). Narrow the POLICY-facing box to match --
            # it's a strict subset of the env's own wider internal bound, so this never triggers
            # an out-of-range action error, it just matches what the policy is allowed to learn
            # to command.
            from quadruped_envs.supermimic.mdp import ACTION_POS_SCALE
            self.action_space = spaces.Box(
                low=-ACTION_POS_SCALE, high=ACTION_POS_SCALE,
                shape=self.action_space.shape, dtype=self.action_space.dtype)
        self._action_rate_weight = action_rate_weight
        self._a_prev = None

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        obs, reward, done, info = self._env.step(action)
        if self._action_rate_weight:
            a_prev = self._a_prev if self._a_prev is not None else np.zeros_like(action)
            # Mirrors mdp.py's action_rate_penalty: mean squared per-joint action change,
            # subtracted from (not folded into) the tracking reward -- a penalty on the ACTION,
            # not a state tracking-error term.
            reward -= self._action_rate_weight * float(np.mean((action - a_prev) ** 2))
        self._a_prev = action
        return obs, reward, done, info

    def reset(self):
        self._a_prev = None  # fresh episode: no action-rate penalty carryover from the last one
        return self._env.reset()

    def close(self):
        self._env.close()

    def render(self, mode):
        return self._env.render(mode)

    def __getattr__(self, attr):
        return getattr(self._env, attr)

    def __len__(self):
        return 1
