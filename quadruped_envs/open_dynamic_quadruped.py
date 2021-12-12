"""This file implements the functionalities of a Open Dynamic robot using pybullet.
        a   b   c   d
hips    0   3   6   9
uleg    1   4   7   10
lleg    2   5   8   11

laikago:
hips    3   0   9   6
uleg    4   1  10   7
lleg    5   2  11   8

"""
import pathlib
import copy
import math
import numpy as np
from pybullet_envs.bullet import motor
import os

NUM_MOTORS = 12
NUM_LEGS = 4
INIT_POSITION = [0, 0, 0.26]
INIT_ORIENTATION = [0, 0, 0, 1]

OVERHEAT_SHUTDOWN_TIME = 1.0
LEG_POSITION = ["a", "c", "b", "d"]
MOTOR_NAMES = [
    "link_hip_a",
    "link_hip_b",
    "link_hip_c",
    "link_hip_d",
    "link_u_leg_a",
    "link_u_leg_b",
    "link_u_leg_c",
    "link_u_leg_d",
    "link_l_leg_a",
    "link_l_leg_b",
    "link_l_leg_c",
    "link_l_leg_d"
]
LEG_LINK_ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
MOTOR_LINK_ID = [0, 1, 3, 4, 6, 7, 9, 10]
MOTOR_DIRECTION = [1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1]
HIP_RANGE = [-1.31, 2.443]
FOOT_LINK_ID = [2, 5, 8, 11]
BASE_LINK_ID = -1


class OpenDynamicQuadruped(object):
    """The robot class that simulates a 12 DoF quadruped robot from Open Dynamic.

    """

    def __init__(self,
                 pybullet_client,
                 urdf_root=os.path.join(os.path.dirname(__file__), "../data"),
                 time_step=0.1,  # ?
                 self_collision_enabled=False,
                 motor_velocity_limit=0.05,
                 pd_control_enabled=False):
        """Constructs a robot and resets it to the initial states.

        Args:
          pybullet_client: The instance of BulletClient to manage different
            simulations.
          urdf_root: The path to the urdf folder.
          time_step: The time step of the simulation.
          self_collision_enabled: Whether to enable self collision.
          motor_velocity_limit: The upper limit of the motor velocity.
          pd_control_enabled: Whether to use PD control for the motors.
        """
        self._pybullet_client = pybullet_client
        self._pybullet_client.setAdditionalSearchPath(str(pathlib.Path(__file__).parent.absolute()))
        self._urdf_root = urdf_root
        self._self_collision_enabled = self_collision_enabled
        self._motor_velocity_limit = motor_velocity_limit
        self._pd_control_enabled = pd_control_enabled
        self._observed_motor_torques = np.zeros(NUM_MOTORS)
        self._applied_motor_torques = np.zeros(NUM_MOTORS)
        self._overheat_counter = np.zeros(NUM_MOTORS)
        self._motor_enabled_list = [True] * NUM_MOTORS
        self._max_force = 2.7
        self._kp = 1
        self._kd = 1
        self.time_step = time_step
        self._joint_name_to_id = {}
        self._motor_id_list = []
        self._leg_masses_urdf = []
        self.reset()

    def reset(self, reload_urdf=True):
        """Reset the robot to its initial states.

        Args:
          reload_urdf: Whether to reload the urdf file. If not, reset() just place
            the robot back to its starting position.
        """
        if reload_urdf:
            if self._self_collision_enabled:
                self.quadruped = self._pybullet_client.loadURDF("/urdf/od.urdf",
                                                                INIT_POSITION,
                                                                flags=self._pybullet_client.URDF_USE_SELF_COLLISION)
            else:
                self.quadruped = self._pybullet_client.loadURDF("/urdf/od.urdf")
            num_joints = self._pybullet_client.getNumJoints(self.quadruped)
            self._joint_name_to_id = {}
            for i in range(num_joints):
                joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
                self._joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
            self._motor_id_list = [self._joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]
            self._base_mass_urdf = self._pybullet_client.getDynamicsInfo(self.quadruped, BASE_LINK_ID)[0]
            self._leg_masses_urdf = []
            self._leg_masses_urdf.append(
                self._pybullet_client.getDynamicsInfo(self.quadruped, LEG_LINK_ID[0])[0])
            self._leg_masses_urdf.append(
                self._pybullet_client.getDynamicsInfo(self.quadruped, MOTOR_LINK_ID[0])[0])
            for i in range(NUM_LEGS):
                self._reset_pose_for_leg(i)
        else:
            self._pybullet_client.resetBaseVelocity(self.quadruped, [0, 0, 0], [0, 0, 0])
            self._pybullet_client.resetBasePositionAndOrientation(self.quadruped,
                                                                  INIT_POSITION,
                                                                  INIT_ORIENTATION)
            self._pybullet_client.resetBaseVelocity(self.quadruped, [0, 0, 0], [0, 0, 0])
            for i in range(NUM_LEGS):
                self._reset_pose_for_leg(i)

        self._overheat_counter = np.zeros(NUM_MOTORS)
        self._motor_enabled_list = [True] * NUM_MOTORS

    def _set_motor_torque_by_id(self, motor_id, torque):
        self._pybullet_client.setJointMotorControl2(bodyIndex=self.quadruped,
                                                    jointIndex=motor_id,
                                                    controlMode=self._pybullet_client.TORQUE_CONTROL,
                                                    force=torque)

    def _set_desired_motor_angle_by_id(self, motor_id, desired_angle):
        self._pybullet_client.setJointMotorControl2(bodyIndex=self.quadruped,
                                                    jointIndex=motor_id,
                                                    controlMode=self._pybullet_client.POSITION_CONTROL,
                                                    targetPosition=desired_angle,
                                                    positionGain=self._kp,
                                                    velocityGain=self._kd,
                                                    force=self._max_force)

    def _set_desired_motor_angle_by_name(self, motor_name, desired_angle):
        self._set_desired_motor_angle_by_id(self._joint_name_to_id[motor_name], desired_angle)

    def _reset_pose_for_leg(self, leg_id):
        """Reset the initial pose for the leg.
        """
        knee_friction_force = 0
        leg_position = LEG_POSITION[leg_id]
        self._pybullet_client.resetJointState(self.quadruped,
                                              self._joint_name_to_id["link_hip_" + leg_position],
                                              0,
                                              targetVelocity=0)
        self._pybullet_client.resetJointState(self.quadruped,
                                              self._joint_name_to_id["link_u_leg_" + leg_position],
                                              0,
                                              targetVelocity=0)
        self._pybullet_client.resetJointState(self.quadruped,
                                              self._joint_name_to_id["link_l_leg_" + leg_position],
                                              0,
                                              targetVelocity=0)
        self._pybullet_client.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=(self._joint_name_to_id["link_hip_" + leg_position]),
            controlMode=self._pybullet_client.VELOCITY_CONTROL,
            targetVelocity=0,
            force=knee_friction_force)
        self._pybullet_client.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=(self._joint_name_to_id["link_u_leg_" + leg_position]),
            controlMode=self._pybullet_client.VELOCITY_CONTROL,
            targetVelocity=0,
            force=knee_friction_force)
        self._pybullet_client.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=(self._joint_name_to_id["link_l_leg_" + leg_position]),
            controlMode=self._pybullet_client.VELOCITY_CONTROL,
            targetVelocity=0,
            force=knee_friction_force)

    def get_quadruped(self):
        return self.quadruped

    def get_base_position(self):
        """Get the position of robot's base.

        Returns:
          The position of robot's base.
        """
        position, _ = (self._pybullet_client.getBasePositionAndOrientation(self.quadruped))
        return position

    def get_base_orientation(self):
        """Get the orientation of robot's base, represented as quaternion.

        Returns:
          The orientation of robot's base.
        """
        _, orientation = (self._pybullet_client.getBasePositionAndOrientation(self.quadruped))
        return orientation

    def get_action_dimension(self):
        """Get the length of the action list.

        Returns:
          The length of the action list.
        """
        return NUM_MOTORS

    def get_observation_upper_bound(self):
        """Get the upper bound of the observation.

        Returns:
          The upper bound of an observation. See get_observation() for the details
            of each element of an observation.
        """
        upper_bound = np.array([0.0] * self.get_observation_dimension())
        upper_bound[0:NUM_MOTORS] = math.pi * 2.0  # Joint angle.
        upper_bound[NUM_MOTORS:2 * NUM_MOTORS] = motor.MOTOR_SPEED_LIMIT  # Joint velocity.
        upper_bound[2 * NUM_MOTORS:3 * NUM_MOTORS] = motor.OBSERVED_TORQUE_LIMIT  # Joint torque.
        upper_bound[3 * NUM_MOTORS:] = 1.0  # Quaternion of base orientation.
        return upper_bound

    def get_observation_lower_bound(self):
        """Get the lower bound of the observation."""
        return -self.get_observation_upper_bound()

    def get_observation_dimension(self):
        """Get the length of the observation list.

        Returns:
          The length of the observation list.
        """
        return len(self.get_observation())

    def get_observation(self):
        """Get the observations of robot.

        It includes the angles, velocities, torques and the orientation of the base.

        Returns:
          The observation list. observation[0:12] are motor angles. observation[12:24]
          are motor velocities, observation[24:36] are motor torques.
          observation[36:40] is the orientation of the base, in quaternion form.
        """
        observation = []
        observation.extend(self.get_motor_angles().tolist())
        observation.extend(self.get_motor_velocities().tolist())
        observation.extend(self.get_motor_torques().tolist())
        observation.extend(list(self.get_base_orientation()))
        return observation

    def scale_commands(self, motor_commands):
        motor_commands[0] = (motor_commands[0] - HIP_RANGE[0]) / (HIP_RANGE[1] - HIP_RANGE[0])
        motor_commands[3] = (motor_commands[3] - HIP_RANGE[0]) / (HIP_RANGE[1] - HIP_RANGE[0])
        motor_commands[6] = (motor_commands[6] - HIP_RANGE[0]) / (HIP_RANGE[1] - HIP_RANGE[0])
        motor_commands[9] = (motor_commands[9] - HIP_RANGE[0]) / (HIP_RANGE[1] - HIP_RANGE[0])
        return motor_commands

    def apply_action(self, motor_commands):
        """Set the desired motor angles to the motors of the robot.

        The desired motor angles are clipped based on the maximum allowed velocity.
        If the pd_control_enabled is True, a torque is calculated according to
        the difference between current and desired joint angle, as well as the joint
        velocity. This torque is exerted to the motor. For more information about
        PD control, please refer to: https://en.wikipedia.org/wiki/PID_controller.

        Args:
          motor_commands: The twelve desired motor angles.
        """
        motor_commands = self.scale_commands(motor_commands)
        if self._motor_velocity_limit < np.inf:
            current_motor_angle = self.get_motor_angles()
            motor_commands_max = (current_motor_angle + self.time_step * self._motor_velocity_limit)
            motor_commands_min = (current_motor_angle - self.time_step * self._motor_velocity_limit)
            motor_commands = np.clip(motor_commands, motor_commands_min, motor_commands_max)

        if self._pd_control_enabled:
            q = self.get_motor_angles()
            qdot = self.get_motor_velocities()
            torque_commands = -self._kp * (q - motor_commands) - self._kd * qdot

            # The torque is already in the observation space because we use
            # get_motor_angles and get_motor_velocities.
            self._observed_motor_torques = torque_commands

            # Transform into the motor space when applying the torque.
            self._applied_motor_torques = np.multiply(self._observed_motor_torques, MOTOR_DIRECTION)

            for motor_id, motor_torque in zip(self._motor_id_list, self._applied_motor_torques):
                self._set_motor_torque_by_id(motor_id, motor_torque)
        else:
            motor_commands_with_direction = np.multiply(motor_commands, MOTOR_DIRECTION)
            for motor_id, motor_command_with_direction in zip(self._motor_id_list,
                                                              motor_commands_with_direction):
                self._set_desired_motor_angle_by_id(motor_id, motor_command_with_direction)

    def get_motor_angles(self):
        """Get the twelve motor angles at the current moment.

        Returns:
          Motor angles.
        """
        motor_angles = [
            self._pybullet_client.getJointState(self.quadruped, motor_id)[0]
            for motor_id in self._motor_id_list
        ]
        motor_angles = np.multiply(motor_angles, MOTOR_DIRECTION)
        return motor_angles

    def get_motor_velocities(self):
        """Get the velocity of all twelve motors.

        Returns:
          Velocities of all twelve motors.
        """
        motor_velocities = [
            self._pybullet_client.getJointState(self.quadruped, motor_id)[1]
            for motor_id in self._motor_id_list
        ]
        motor_velocities = np.multiply(motor_velocities, MOTOR_DIRECTION)
        return motor_velocities

    def get_motor_torques(self):
        """Get the amount of torques the motors are exerting.

        Returns:
          Motor torques of all twelve motors.
        """
        if self._pd_control_enabled:
            return self._observed_motor_torques
        else:
            motor_torques = [
                self._pybullet_client.getJointState(self.quadruped, motor_id)[3]
                for motor_id in self._motor_id_list
            ]
            motor_torques = np.multiply(motor_torques, MOTOR_DIRECTION)
        return motor_torques

    def convert_from_leg_model(self, actions):
        """Convert the actions that use leg model to the real motor actions.

        Args:
          actions: The theta, phi of the leg model.
        Returns:
          The twelve desired motor angles that can be used in apply_actions().
        """

        motor_angle = copy.deepcopy(actions)
        scale_for_singularity = 1
        offset_for_singularity = 1.5
        half_num_motors = int(NUM_MOTORS / 2)
        quater_pi = math.pi / 4
        for i in range(NUM_MOTORS):
            action_idx = i // 2
            forward_backward_component = (
                    -scale_for_singularity * quater_pi *
                    (actions[action_idx + half_num_motors] + offset_for_singularity))
            extension_component = (-1) ** i * quater_pi * actions[action_idx]
            if i >= half_num_motors:
                extension_component = -extension_component
            motor_angle[i] = (math.pi + forward_backward_component + extension_component)
        return motor_angle

    def get_base_mass_from_URDF(self):
        """Get the mass of the base from the URDF file."""
        return self._base_mass_urdf

    def get_leg_masses_from_URDF(self):
        """Get the mass of the legs from the URDF file."""
        return self._leg_masses_urdf

    def set_base_mass(self, base_mass):
        self._pybullet_client.changeDynamics(self.quadruped, BASE_LINK_ID, mass=base_mass)

    def set_leg_masses(self, leg_masses):
        """Set the mass of the legs.

        A leg includes leg_link and motor. All four leg_links have the same mass,
        which is leg_masses[0]. All four motors have the same mass, which is
        leg_mass[1].

        Args:
          leg_masses: The leg masses. leg_masses[0] is the mass of the leg link.
            leg_masses[1] is the mass of the motor.
        """
        for link_id in LEG_LINK_ID:
            self._pybullet_client.changeDynamics(self.quadruped, link_id, mass=leg_masses[0])
        for link_id in MOTOR_LINK_ID:
            self._pybullet_client.changeDynamics(self.quadruped, link_id, mass=leg_masses[1])

    def set_foot_friction(self, foot_friction):
        """Set the lateral friction of the feet.

        Args:
          foot_friction: The lateral friction coefficient of the foot. This value is
            shared by all four feet.
        """
        for link_id in FOOT_LINK_ID:
            self._pybullet_client.changeDynamics(self.quadruped, link_id, lateralFriction=foot_friction)