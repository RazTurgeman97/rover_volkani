#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
# from math import pi # Not used

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_imu = LaunchConfiguration('use_imu')
    pkg_share = get_package_share_directory('roverrobotics_driver')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # Typically true if simulating
        description='Use simulation/Gazebo clock')

    declare_use_imu_argument = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Whether to use IMU data for EKF localization.')

    ekf_config_with_imu = PathJoinSubstitution([pkg_share, 'config', 'localization_ekf.yaml'])
    ekf_config_no_imu = PathJoinSubstitution([pkg_share, 'config', 'localization_ekf_no_imu.yaml'])

    localization_node_with_imu = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_with_imu, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_imu)
    )

    localization_node_no_imu = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_no_imu, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_imu)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_use_imu_argument)
    ld.add_action(localization_node_with_imu)
    ld.add_action(localization_node_no_imu)
    
    return ld
