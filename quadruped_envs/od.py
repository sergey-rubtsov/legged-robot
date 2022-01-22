"""Pybullet simulation of an Open Dynamics robot

Joints configuration:

a - front left
b - front right
c - back left
d - back right

        a   b   c   d
hips    0   3   6   9
uleg    1   4   7   10
lleg    2   5   8   11

"""

import os
import math
import re
import numpy as np
import pybullet as pyb  # pytype: disable=import-error
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from motion_imitation.robots import laikago_constants
from .imitation_wrapper_env import od_motor
from motion_imitation.robots import minitaur
from motion_imitation.robots import robot_config
from motion_imitation.envs import locomotion_gym_config

NUM_MOTORS = 12
NUM_LEGS = 4

UPPER_BOUND = 6.3
LOWER_BOUND = -6.3

ACTION_CONFIG = [
    locomotion_gym_config.ScalarField(name="FL_hip_motor",
                                      upper_bound=2.443,
                                      lower_bound=-1.31),
    locomotion_gym_config.ScalarField(name="FL_upper_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND),
    locomotion_gym_config.ScalarField(name="FL_lower_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND),
    locomotion_gym_config.ScalarField(name="FR_hip_motor",
                                      upper_bound=1.31,
                                      lower_bound=-2.443),
    locomotion_gym_config.ScalarField(name="FR_upper_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND),
    locomotion_gym_config.ScalarField(name="FR_lower_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND),
    locomotion_gym_config.ScalarField(name="RL_hip_motor",
                                      upper_bound=2.443,
                                      lower_bound=-1.31),
    locomotion_gym_config.ScalarField(name="RL_upper_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND),
    locomotion_gym_config.ScalarField(name="RL_lower_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND),
    locomotion_gym_config.ScalarField(name="RR_hip_motor",
                                      upper_bound=1.31,
                                      lower_bound=-2.443),
    locomotion_gym_config.ScalarField(name="RR_upper_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND),
    locomotion_gym_config.ScalarField(name="RR_lower_motor",
                                      upper_bound=UPPER_BOUND,
                                      lower_bound=LOWER_BOUND)
]

MOTOR_NAMES = [
    "link_hip_a",
    "link_u_leg_a",
    "link_l_leg_a",
    "link_hip_b",
    "link_u_leg_b",
    "link_l_leg_b",
    "link_hip_c",
    "link_u_leg_c",
    "link_l_leg_c",
    "link_hip_d",
    "link_u_leg_d",
    "link_l_leg_d"
]
INIT_RACK_POSITION = [0, 0, 1]
INIT_POSITION = [0, 0, 0.28]
JOINT_DIRECTIONS = np.array([-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1])
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0
KNEE_JOINT_OFFSET = 0
DOFS_PER_LEG = 3
JOINT_OFFSETS = np.array(
    [
        HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET,
        HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET,
        HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET,
        HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET
    ]
)

PI = math.pi

MAX_MOTOR_ANGLE_CHANGE_PER_STEP = 0.5
_DEFAULT_HIP_POSITIONS = (
    (0, 0, 0),
    (0, 0, 0),
    (0, 0, 0),
    (0, 0, 0),
)

COM_OFFSET = -np.array([0, 0, 0])
HIP_OFFSETS = np.array([[0, 0, 0.], [0, 0, 0.],
                        [0, 0, 0.], [0, 0, 0.]
                        ]) + COM_OFFSET
K_P = 1.0
K_D = 0.01
ABDUCTION_P_GAIN = K_P
ABDUCTION_D_GAIN = K_D
HIP_P_GAIN = K_P
HIP_D_GAIN = K_D
KNEE_P_GAIN = K_P
KNEE_D_GAIN = K_D

MAX_TORQUE = 1  # MAX_TORQUE = 300 for position
TORQUE_LIMITS = np.full(NUM_MOTORS, MAX_TORQUE)

# Bases on the readings from robot's default pose.
INIT_MOTOR_ANGLES = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

HIP_NAME_PATTERN = re.compile(r"\w+_hip_\w+")
UPPER_NAME_PATTERN = re.compile(r"\w+_u_leg_\w+")
LOWER_NAME_PATTERN = re.compile(r"\w+_l_leg_\w+")
IMU_NAME_PATTERN = re.compile(r"imu\d*")

URDF_FILENAME = currentdir + "/urdf/od.urdf"

TIME_MULTIPLIER = 1
TIME_STEP = 0.01 * TIME_MULTIPLIER

_BODY_B_FIELD_NUMBER = 2
_LINK_A_FIELD_NUMBER = 3


def foot_position_in_hip_frame_to_joint_angle(foot_position, l_hip_sign=1):
    l_up = 0.2
    l_low = 0.2
    l_hip = 0.08505 * l_hip_sign
    x, y, z = foot_position[0], foot_position[1], foot_position[2]
    tmp = (x ** 2 + y ** 2 + z ** 2 - l_hip ** 2 - l_low ** 2 - l_up ** 2) / (2 * l_low * l_up)
    if tmp > 1:
        theta_knee = 0.000000001
    else:
        theta_knee = -np.arccos(tmp)
    l = np.sqrt(l_up ** 2 + l_low ** 2 + 2 * l_up * l_low * np.cos(theta_knee))
    tmp2 = -x / l
    if tmp2 < -1:
        theta_hip = -1.5707963267948966 - theta_knee / 2
    else:
        theta_hip = np.arcsin(tmp2) - theta_knee / 2
    c1 = l_hip * y - l * np.cos(theta_hip + theta_knee / 2) * z
    s1 = l * np.cos(theta_hip + theta_knee / 2) * y + l_hip * z
    theta_ab = np.arctan2(s1, c1)
    return np.array([theta_ab, theta_hip, theta_knee])


def foot_position_in_hip_frame(angles, l_hip_sign=1):
    theta_ab, theta_hip, theta_knee = angles[0], angles[1], angles[2]
    l_up = 0.2
    l_low = 0.2
    l_hip = 0.08505 * l_hip_sign
    leg_distance = np.sqrt(l_up ** 2 + l_low ** 2 +
                           2 * l_up * l_low * np.cos(theta_knee))
    eff_swing = theta_hip + theta_knee / 2

    off_x_hip = -leg_distance * np.sin(eff_swing)
    off_z_hip = -leg_distance * np.cos(eff_swing)
    off_y_hip = l_hip

    off_x = off_x_hip
    off_y = np.cos(theta_ab) * off_y_hip - np.sin(theta_ab) * off_z_hip
    off_z = np.sin(theta_ab) * off_y_hip + np.cos(theta_ab) * off_z_hip
    return np.array([off_x, off_y, off_z])


def analytical_leg_jacobian(leg_angles, leg_id):
    """
  Computes the analytical Jacobian.
  Args:
  ` leg_angles: a list of 3 numbers for current abduction, hip and knee angle.
    l_hip_sign: whether it's a left (1) or right(-1) leg.
  """
    l_up = 0.2
    l_low = 0.2
    l_hip = 0.08505 * (-1) ** (leg_id + 1)

    t1, t2, t3 = leg_angles[0], leg_angles[1], leg_angles[2]
    l_eff = np.sqrt(l_up ** 2 + l_low ** 2 + 2 * l_up * l_low * np.cos(t3))
    t_eff = t2 + t3 / 2
    J = np.zeros((3, 3))
    J[0, 0] = 0
    J[0, 1] = -l_eff * np.cos(t_eff)
    J[0, 2] = l_low * l_up * np.sin(t3) * np.sin(t_eff) / l_eff - l_eff * np.cos(
        t_eff) / 2
    J[1, 0] = -l_hip * np.sin(t1) + l_eff * np.cos(t1) * np.cos(t_eff)
    J[1, 1] = -l_eff * np.sin(t1) * np.sin(t_eff)
    J[1, 2] = -l_low * l_up * np.sin(t1) * np.sin(t3) * np.cos(
        t_eff) / l_eff - l_eff * np.sin(t1) * np.sin(t_eff) / 2
    J[2, 0] = l_hip * np.cos(t1) + l_eff * np.sin(t1) * np.cos(t_eff)
    J[2, 1] = l_eff * np.sin(t_eff) * np.cos(t1)
    J[2, 2] = l_low * l_up * np.sin(t3) * np.cos(t1) * np.cos(
        t_eff) / l_eff + l_eff * np.sin(t_eff) * np.cos(t1) / 2
    return J


# For JIT compilation
foot_position_in_hip_frame_to_joint_angle(np.random.uniform(size=3), 1)
foot_position_in_hip_frame_to_joint_angle(np.random.uniform(size=3), -1)


def foot_positions_in_base_frame(foot_angles):
    foot_angles = foot_angles.reshape((4, 3))
    foot_positions = np.zeros((4, 3))
    for i in range(4):
        foot_positions[i] = foot_position_in_hip_frame(foot_angles[i],
                                                       l_hip_sign=(-1) ** (i + 1))
    return foot_positions + HIP_OFFSETS


class OD(minitaur.Minitaur):
    """Pybullet simulation of an Open Dynamics robot"""

    # At high replanning frequency, inaccurate values of BODY_MASS/INERTIA
    # doesn't seem to matter much. However, these values should be better tuned
    # when the replan frequency is low (e.g. using a less beefy CPU).
    MPC_BODY_MASS = 20 / 9.8
    MPC_BODY_INERTIA = np.array((0.017, 0, 0, 0, 0.057, 0, 0, 0, 0.064)) * 4.
    MPC_BODY_HEIGHT = 0.26
    MPC_VELOCITY_MULTIPLIER = 0.5

    def __init__(
            self,
            pybullet_client,
            urdf_filename=URDF_FILENAME,
            enable_clip_motor_commands=False,
            time_step=TIME_STEP,  # for reference move, it was 0.001
            action_repeat=1,  # was 10
            sensors=None,
            control_latency=0.002,
            on_rack=False,
            enable_action_interpolation=False,
            enable_action_filter=False,
            motor_control_mode=None,
            reset_time=1,
            allow_knee_contact=False,
    ):

        self._urdf_filename = urdf_filename
        self._allow_knee_contact = allow_knee_contact
        self._enable_clip_motor_commands = enable_clip_motor_commands
        # pybullet_client.setPhysicsEngineParameter(numSolverIterations=200)
        # pybullet_client.setPhysicsEngineParameter(solverResidualThreshold=1e-30)
        # pybullet_client.setTimeStep(1. / 6000)
        # pybullet_client.configureDebugVisualizer(pybullet_client.COV_ENABLE_WIREFRAME, 1)

        motor_kp = [
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN,
            HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN,
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN
        ]
        motor_kd = [
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN,
            HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN,
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN
        ]

        super(OD, self).__init__(
            pybullet_client=pybullet_client,
            time_step=time_step,
            action_repeat=action_repeat,
            num_motors=NUM_MOTORS,
            dofs_per_leg=DOFS_PER_LEG,
            motor_direction=JOINT_DIRECTIONS,
            motor_offset=JOINT_OFFSETS,
            motor_overheat_protection=False,
            motor_control_mode=motor_control_mode,
            motor_model_class=od_motor.ODMotorModel,
            sensors=sensors,
            motor_kp=motor_kp,
            motor_kd=motor_kd,
            motor_torque_limits=TORQUE_LIMITS,
            control_latency=control_latency,
            on_rack=on_rack,
            enable_action_interpolation=False,
            enable_action_filter=enable_action_filter,
            reset_time=reset_time)

    def _LoadRobotURDF(self):
        urdf_path = self.GetURDFFile()
        if self._self_collision_enabled:
            self.quadruped = self._pybullet_client.loadURDF(
                urdf_path,
                self._GetDefaultInitPosition(),
                self._GetDefaultInitOrientation(),
                flags=self._pybullet_client.URDF_USE_SELF_COLLISION)
        else:
            self.quadruped = self._pybullet_client.loadURDF(
                urdf_path,
                self._GetDefaultInitPosition(),
                self._GetDefaultInitOrientation())

    def _SettleDownForReset(self, default_motor_angles, reset_time):
        self.ReceiveObservation()
        if reset_time <= 0:
            return

        for _ in range(500):
            self._StepInternal(
                INIT_MOTOR_ANGLES,
                motor_control_mode=robot_config.MotorControlMode.POSITION)

        if default_motor_angles is not None:
            num_steps_to_reset = int(reset_time / self.time_step)
            for _ in range(num_steps_to_reset):
                self._StepInternal(
                    default_motor_angles,
                    motor_control_mode=robot_config.MotorControlMode.POSITION)

    def GetHipPositionsInBaseFrame(self):
        return _DEFAULT_HIP_POSITIONS

    def GetFootContacts(self):
        all_contacts = self._pybullet_client.getContactPoints(bodyA=self.quadruped)

        contacts = [False, False, False, False]
        for contact in all_contacts:
            # Ignore self contacts
            if contact[_BODY_B_FIELD_NUMBER] == self.quadruped:
                continue
            try:
                toe_link_index = self._foot_link_ids.index(
                    contact[_LINK_A_FIELD_NUMBER])
                contacts[toe_link_index] = True
            except ValueError:
                continue

        return contacts

    def ResetPose(self, add_constraint):
        del add_constraint
        for name in self._joint_name_to_id:
            joint_id = self._joint_name_to_id[name]
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=(joint_id),
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0)
        for name, i in zip(MOTOR_NAMES, range(len(MOTOR_NAMES))):
            if "link_hip_a" in name:
                angle = 0
            elif "link_u_leg_a" in name:
                angle = 0
            elif "link_l_leg_a" in name:
                angle = 0
            elif "link_hip_b" in name:
                angle = 0
            elif "link_u_leg_b" in name:
                angle = 0
            elif "link_l_leg_b" in name:
                angle = 0
            elif "link_hip_c" in name:
                angle = 0
            elif "link_u_leg_c" in name:
                angle = 0
            elif "link_l_leg_c" in name:
                angle = 0
            elif "link_hip_d" in name:
                angle = 0
            elif "link_u_leg_d" in name:
                angle = 0
            elif "link_l_leg_d" in name:
                angle = 0
            else:
                raise ValueError("The name %s is not recognized as a motor joint." %
                                 name)
            self._pybullet_client.resetJointState(self.quadruped,
                                                  self._joint_name_to_id[name],
                                                  angle,
                                                  targetVelocity=0)

    def GetURDFFile(self):
        return self._urdf_filename

    def _BuildUrdfIds(self):
        """Build the link Ids from its name in the URDF file.

    Raises:
      ValueError: Unknown category of the joint name.
    """
        num_joints = self.pybullet_client.getNumJoints(self.quadruped)
        # self._hip_link_ids = [-1]
        self._hip_link_ids = []
        self._leg_link_ids = []
        self._motor_link_ids = []
        self._lower_link_ids = []
        self._foot_link_ids = []
        self._imu_link_ids = []

        for i in range(num_joints):
            joint_info = self.pybullet_client.getJointInfo(self.quadruped, i)
            joint_name = joint_info[1].decode("UTF-8")
            joint_id = self._joint_name_to_id[joint_name]
            if HIP_NAME_PATTERN.match(joint_name):
                self._hip_link_ids.append(joint_id)
            elif UPPER_NAME_PATTERN.match(joint_name):
                self._motor_link_ids.append(joint_id)
            elif LOWER_NAME_PATTERN.match(joint_name):
                self._lower_link_ids.append(joint_id)
            elif IMU_NAME_PATTERN.match(joint_name):
                self._imu_link_ids.append(joint_id)
            else:
                raise ValueError("Unknown category of joint %s" % joint_name)

        self._leg_link_ids.extend(self._lower_link_ids)
        self._leg_link_ids.extend(self._motor_link_ids)

        # assert len(self._foot_link_ids) == NUM_LEGS
        self._hip_link_ids.sort()
        self._motor_link_ids.sort()
        self._lower_link_ids.sort()
        self._foot_link_ids.sort()
        self._leg_link_ids.sort()

    def _GetMotorNames(self):
        return MOTOR_NAMES

    def _GetDefaultInitPosition(self):
        if self._on_rack:
            return INIT_RACK_POSITION
        else:
            return INIT_POSITION

    def _GetDefaultInitOrientation(self):
        init_orientation = pyb.getQuaternionFromEuler([0, 0, 0])
        return init_orientation

    def GetDefaultInitPosition(self):
        """Get default initial base position."""
        return self._GetDefaultInitPosition()

    def GetDefaultInitOrientation(self):
        """Get default initial base orientation."""
        return self._GetDefaultInitOrientation()

    def GetDefaultInitJointPose(self):
        """Get default initial joint pose."""
        joint_pose = (INIT_MOTOR_ANGLES + JOINT_OFFSETS) * JOINT_DIRECTIONS
        return joint_pose

    def ApplyAction(self, motor_commands, motor_control_mode=None):
        """Clips and then apply the motor commands using the motor model.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).N
      motor_control_mode: A MotorControlMode enum.
    """
        if self._enable_clip_motor_commands:
            motor_commands = self._ClipMotorCommands(motor_commands)
        super(OD, self).ApplyAction(motor_commands, motor_control_mode)

    def _ClipMotorCommands(self, motor_commands):
        """Clips motor commands.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).

    Returns:
      Clipped motor commands.
    """

        # clamp the motor command by the joint limit, in case weired things happens
        max_angle_change = MAX_MOTOR_ANGLE_CHANGE_PER_STEP
        current_motor_angles = self.GetMotorAngles()
        motor_commands = np.clip(motor_commands,
                                 current_motor_angles - max_angle_change,
                                 current_motor_angles + max_angle_change)
        return motor_commands

    @classmethod
    def GetConstants(cls):
        del cls
        return laikago_constants

    def ComputeMotorAnglesFromFootLocalPosition(self, leg_id,
                                                foot_local_position):
        """Use IK to compute the motor angles, given the foot link's local position.

    Args:
      leg_id: The leg index.
      foot_local_position: The foot link's position in the base frame.

    Returns:
      A tuple. The position indices and the angles for all joints along the
      leg. The position indices is consistent with the joint orders as returned
      by GetMotorAngles API.
    """
        assert len(self._foot_link_ids) == self.num_legs
        # toe_id = self._foot_link_ids[leg_id]

        motors_per_leg = self.num_motors // self.num_legs
        joint_position_idxs = list(
            range(leg_id * motors_per_leg,
                  leg_id * motors_per_leg + motors_per_leg))

        joint_angles = foot_position_in_hip_frame_to_joint_angle(
            foot_local_position - HIP_OFFSETS[leg_id],
            l_hip_sign=(-1) ** (leg_id + 1))

        # Joint offset is necessary for Laikago.
        joint_angles = np.multiply(
            np.asarray(joint_angles) -
            np.asarray(self._motor_offset)[joint_position_idxs],
            self._motor_direction[joint_position_idxs])

        # Return the joing index (the same as when calling GetMotorAngles) as well
        # as the angles.
        return joint_position_idxs, joint_angles.tolist()

    def GetFootPositionsInBaseFrame(self):
        """Get the robot's foot position in the base frame."""
        motor_angles = self.GetMotorAngles()
        return foot_positions_in_base_frame(motor_angles)

    def ComputeJacobian(self, leg_id):
        """Compute the Jacobian for a given leg."""
        # Does not work for Minitaur which has the four bar mechanism for now.
        motor_angles = self.GetMotorAngles()[leg_id * 3:(leg_id + 1) * 3]
        return analytical_leg_jacobian(motor_angles, leg_id)

    def GetFootLinkIDs(self):
        """Get list of IDs for all foot links."""
        return self._lower_link_ids

    def GetLegLinkIDs(self):
        return self._motor_link_ids

    def GetHipLinkIDs(self):
        return self._hip_link_ids
