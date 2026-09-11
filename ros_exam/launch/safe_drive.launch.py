"""Spawn the Gazebo robot and gate user velocity commands through safe_drive."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('safety_threshold', default_value='0.5',
                              description='Minimum laser range in metres'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bme_gazebo_sensors'),
            'launch', 'spawn_robot.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        Node(package='ros_exam', executable='robot_monitor', name='robot_monitor',
             output='screen', parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
        Node(package='ros_exam', executable='safe_drive.py', name='safe_drive',
             output='screen', parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'safety_threshold': ParameterValue(
                     LaunchConfiguration('safety_threshold'), value_type=float),
             }]),
    ])
