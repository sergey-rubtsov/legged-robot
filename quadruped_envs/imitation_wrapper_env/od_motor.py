import collections
import collections.abc
import numpy as np

from motion_imitation.robots import robot_config

NUM_MOTORS = 12
DEFAULT_TORQUE_LIMIT = 25
# 25 N*m (was 50, and the URDF's own effort="2.7" before that -- both placeholders, see
# od.urdf's module history). Derived from this robot's own mass/geometry: static standing
# needs ~4.5-6.8 N*m per joint in a 4-leg stance, ~9-13.6 N*m worst-case in a 2-leg stance
# (od.py's analytical_leg_jacobian + summed URDF link masses, 14.01 kg total -- see project
# memory). ~2x that worst-case static figure gives headroom for dynamic loads (push-off,
# impact, recovery) without being unboundedly large. Cross-checked against real actuators in
# this robot's mass class (Unitree Go1, ~12 kg, ~23.7 N*m continuous / ~35 N*m peak per joint)
# -- 25 N*m sits in the same plausible range. TORQUE_MULTIPLIER (below) must stay equal to this
# value: action in [-1, 1] * TORQUE_MULTIPLIER is the actual applied torque in TORQUE control
# mode, and PyBullet does NOT separately clamp it to the URDF's declared `effort` (that only
# constrains the built-in PD controller in POSITION/HYBRID mode) -- so if these two numbers
# ever diverge again, the simulated robot is silently stronger or weaker than its own stated
# spec. Any policy trained with the old TORQUE_MULTIPLIER=50 is NOT valid under this value --
# retrain from scratch, don't resume an old checkpoint against this change.
TORQUE_MULTIPLIER = DEFAULT_TORQUE_LIMIT

MOTOR_COMMAND_DIMENSION = 5

# These values represent the indices of each field in the motor command tuple
POSITION_INDEX = 0
POSITION_GAIN_INDEX = 1
VELOCITY_INDEX = 2
VELOCITY_GAIN_INDEX = 3
TORQUE_INDEX = 4


class ODMotorModel(object):
    """A simple motor model (modified motor for Laikago).

    When in POSITION mode, the torque is calculated according to the difference
    between current and desired joint angle, as well as the joint velocity.
    For more information about PD control, please refer to:
    https://en.wikipedia.org/wiki/PID_controller.

    The model supports a HYBRID mode in which each motor command can be a tuple
    (desired_motor_angle, position_gain, desired_motor_velocity, velocity_gain,
    torque).

  """

    def __init__(self,
                 kp=5,
                 kd=0.5,
                 torque_limits=None,
                 motor_control_mode=robot_config.MotorControlMode.POSITION):
        self._kp = kp
        self._kd = kd
        self._torque_limits = torque_limits
        if torque_limits is not None:
            if isinstance(torque_limits, (collections.abc.Sequence, np.ndarray)):
                self._torque_limits = np.asarray(torque_limits)
            else:
                self._torque_limits = np.full(NUM_MOTORS, DEFAULT_TORQUE_LIMIT)
        self._motor_control_mode = motor_control_mode
        self._strength_ratios = np.full(NUM_MOTORS, TORQUE_MULTIPLIER)

    def set_strength_ratios(self, ratios):
        """Set the strength of each motors relative to the default value.

    Args:
      ratios: The relative strength of motor output. A numpy array ranging from
        0.0 to 1.0.
    """
        self._strength_ratios = ratios

    def set_motor_gains(self, kp, kd):
        """Set the gains of all motors.

    These gains are PD gains for motor positional control. kp is the
    proportional gain and kd is the derivative gain.

    Args:
      kp: proportional gain of the motors.
      kd: derivative gain of the motors.
    """
        self._kp = kp
        self._kd = kd

    def set_voltage(self, voltage):
        pass

    def get_voltage(self):
        return 0.0

    def set_viscous_damping(self, viscous_damping):
        pass

    def get_viscous_dampling(self):
        return 0.0

    def convert_to_torque(self,
                          motor_commands,
                          motor_angle,
                          motor_velocity,
                          true_motor_velocity,
                          motor_control_mode):
        """Convert the commands (position control or torque control) to torque.

    Args:
      motor_commands: The desired motor angle if the motor is in position
        control mode. The pwm signal if the motor is in torque control mode.
      motor_angle: The motor angle observed at the current time step. It is
        actually the true motor angle observed a few milliseconds ago (pd
        latency).
      motor_velocity: The motor velocity observed at the current time step, it
        is actually the true motor velocity a few milliseconds ago (pd latency).
      true_motor_velocity: The true motor velocity. The true velocity is used to
        compute back EMF voltage and viscous damping.
      motor_control_mode: A MotorControlMode enum.

    Returns:
      actual_torque: The torque that needs to be applied to the motor.
      observed_torque: The torque observed by the sensor.
    """
        del true_motor_velocity
        if not motor_control_mode:
            motor_control_mode = self._motor_control_mode

        if motor_control_mode is robot_config.MotorControlMode.PWM:
            raise ValueError(
                "{} is not a supported motor control mode".format(motor_control_mode))

        # No processing for motor torques
        if motor_control_mode is robot_config.MotorControlMode.TORQUE:
            assert len(motor_commands) == NUM_MOTORS
            motor_torques = self._strength_ratios * motor_commands
            return motor_torques, motor_torques

        desired_motor_angles = None
        desired_motor_velocities = None
        kp = None
        kd = None
        additional_torques = np.full(NUM_MOTORS, 0)
        if motor_control_mode is robot_config.MotorControlMode.POSITION:
            assert len(motor_commands) == NUM_MOTORS
            kp = self._kp
            kd = self._kd
            desired_motor_angles = motor_commands
            desired_motor_velocities = np.full(NUM_MOTORS, 0)
        elif motor_control_mode is robot_config.MotorControlMode.HYBRID:
            # The input should be a 60 dimension vector
            assert len(motor_commands) == MOTOR_COMMAND_DIMENSION * NUM_MOTORS
            kp = motor_commands[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION]
            kd = motor_commands[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION]
            desired_motor_angles = motor_commands[
                                   POSITION_INDEX::MOTOR_COMMAND_DIMENSION]
            desired_motor_velocities = motor_commands[
                                       VELOCITY_INDEX::MOTOR_COMMAND_DIMENSION]
            additional_torques = motor_commands[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION]
        else:
            print("Undefined motor_control_mode=", motor_control_mode)
            exit()
        motor_torques = -1 * (kp * (motor_angle - desired_motor_angles)) - kd * (
                motor_velocity - desired_motor_velocities) + additional_torques
        motor_torques = self._strength_ratios * motor_torques
        if self._torque_limits is not None:
            if len(self._torque_limits) != len(motor_torques):
                raise ValueError(
                    "Torque limits dimension does not match the number of motors.")
            motor_torques = np.clip(motor_torques, -1.0 * self._torque_limits,
                                    self._torque_limits)

        return motor_torques, motor_torques
