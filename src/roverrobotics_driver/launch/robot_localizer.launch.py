#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # Conditional EKF configuration path
    ekf_config_path = PythonExpression([
        "os.path.join('", pkg_share, "', 'config', 'localization_ekf.yaml') if '", use_imu, "' == 'true' else ",
        "os.path.join('", pkg_share, "', 'config', 'localization_ekf_no_imu.yaml')"
    ])
    
    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )
    
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_use_imu_argument)
    ld.add_action(localization_node)
    
    return ld
