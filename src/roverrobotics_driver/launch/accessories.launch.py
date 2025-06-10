#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    pkg_rover_driver = get_package_share_directory('roverrobotics_driver')
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Whether to activate the IMU from accessories.yaml or accessories_no_imu.yaml.'
    )

    from launch.actions import OpaqueFunction

    def launch_accessories(context):
        use_imu = context.launch_configurations.get('use_imu', 'true').lower() == 'true'
        if use_imu:
            config_path = os.path.join(pkg_rover_driver, 'config', 'accessories.yaml')
        else:
            config_path = os.path.join(pkg_rover_driver, 'config', 'accessories_no_imu.yaml')

        actions_to_add = []

        # Read the config file
        with open(config_path, 'r') as f:
            accessories_config = yaml.load(f, Loader=yaml.FullLoader)

        # RP Lidar Setup
        if accessories_config.get('rplidar', {}).get('ros__parameters', {}).get('active', False):
            actions_to_add.append(
                Node(
                    package='rplidar_ros',
                    executable='rplidar_composition',
                    name='rplidar',
                    parameters=[config_path],
                    output='screen'
                )
            )

        # BNO055 IMU Setup
        if accessories_config.get('bno055', {}).get('ros__parameters', {}).get('active', False):
            actions_to_add.append(
                Node(
                    package='bno055',
                    name='bno055',
                    executable='bno055',
                    parameters=[config_path],
                    remappings=[('/imu', '/imu/data')]
                )
            )

        # Realsense Node
        if accessories_config.get('realsense', {}).get('ros__parameters', {}).get('active', False):
            actions_to_add.append(
                Node(
                    package='realsense2_camera',
                    name="realsense",
                    executable='realsense2_camera_node',
                    parameters=[config_path],
                    output='screen'
                )
            )

        return actions_to_add

    ld = LaunchDescription()
    ld.add_action(use_imu_arg)
    ld.add_action(OpaqueFunction(function=launch_accessories))
    return ld