from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
  # Get the path for the urdf path
  urdf_path = os.path.join(
    get_package_share_path('robot_arm_description'),
    'urdf/robot_arm.urdf'
  )

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description':
           ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}]
  )

  joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', os.path.join(
        get_package_share_path('robot_arm_description'),
        'rviz/config.rviz'
    )]
  )

  return LaunchDescription([
    robot_state_publisher_node,
    joint_state_publisher_gui_node,
    rviz_node
  ])

