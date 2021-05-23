"""
===========
URDF Joints
===========

This example shows how to load a URDF description of a robot, set some joint
angles and display relevant frames.
"""
import matplotlib.pyplot as plt
from pytransform3d.urdf import UrdfTransformManager


COMPI_URDF = """
<?xml version="1.0"?>
  <robot name="compi">
    <link name="link_Hips"/>
    <link name="link_LeftUpLeg"/>
    <link name="link_LeftLeg"/>
    <link name="link_LeftFoot"/>
    <link name="link_LeftToeBase"/>
    <link name="link_RightUpLeg"/>
    <link name="link_RightLeg"/>
    <link name="link_RightFoot"/>
    <link name="link_RightToeBase"/>
    
    <joint name="link_LeftUpLeg" type="revolute">
      <origin xyz="0.08208 0.01676 -0.06751" rpy="0 0 0"/>
      <parent link="link_Hips"/>
      <child link="link_LeftUpLeg"/>
      <limit effort="0" velocity="0"/>
    </joint> 

    <joint name="link_LeftLeg" type="revolute">
      <origin xyz="0.00023 -0.00426 -0.44522" rpy="0 0 0"/>
      <parent link="link_LeftUpLeg"/>
      <child link="link_LeftLeg"/>
      <limit effort="0" velocity="0"/>
    </joint>
    
    <joint name="link_LeftFoot" type="revolute">
      <origin xyz="0 0.0304 -0.4433" rpy="0 0 0"/>
      <parent link="link_LeftLeg"/>
      <child link="link_LeftFoot"/>
      <limit effort="0" velocity="0"/>
    </joint>

    <joint name="link_LeftToeBase" type="revolute">
      <origin xyz="-0.00023 -0.08629 -0.08665" rpy="0 0 0"/>
      <parent link="link_LeftFoot"/>
      <child link="link_LeftToeBase"/>
      <limit effort="0" velocity="0"/>
    </joint>

    <joint name="link_RightUpLeg" type="revolute">
      <origin xyz="-0.08224 0.01519 -0.06771" rpy="0 0 0"/>
      <parent link="link_Hips"/>
      <child link="link_RightUpLeg"/>
      <limit effort="0" velocity="0"/>
    </joint>
    
    <joint name="link_RightLeg" type="revolute">
      <origin xyz="0.00065 -0.00196 -0.44457" rpy="0 0 0"/>
      <parent link="link_RightUpLeg"/>
      <child link="link_RightLeg"/>
      <limit effort="0" velocity="0"/>
    </joint>
    
    <joint name="link_RightFoot" type="revolute">
      <origin xyz="-0.00065 0.03005 -0.44359" rpy="0 0 0"/>
      <parent link="link_RightLeg"/>
      <child link="link_RightFoot"/>
      <limit effort="0" velocity="0"/>
    </joint>
    
    <joint name="link_RightToeBase" type="revolute">
      <origin xyz="0.00016 -0.08667 -0.08682" rpy="0 0 0"/>
      <parent link="link_RightFoot"/>
      <child link="link_RightToeBase"/>
      <limit effort="0" velocity="0"/>
    </joint>
    
  </robot>
"""

tm = UrdfTransformManager()
tm.load_urdf(COMPI_URDF)
joint_names = ["link_LeftUpLeg", "link_LeftLeg", "link_LeftFoot", "link_LeftToeBase", "link_RightUpLeg", "link_RightLeg", "link_RightFoot", "link_RightToeBase"]
joint_angles = [0, 0, 0, 0, 0, 0, 0]
for name, angle in zip(joint_names, joint_angles):
    tm.set_joint(name, angle)
ax = tm.plot_frames_in("compi", whitelist=joint_names,
                       s=0.05, show_name=True)
ax = tm.plot_connections_in("compi", ax=ax)
ax.set_xlim((-0.2, 0.8))
ax.set_ylim((-0.5, 0.5))
ax.set_zlim((-0.2, 0.8))
plt.show()