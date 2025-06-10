#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
# from math import pi # Not used
import yaml

def generate_launch_description():
    pkg_rover_driver = get_package_share_directory('roverrobotics_driver')
    use_imu_arg = LaunchConfiguration('use_imu')

    declare_use_imu_cmd = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Whether to activate the IMU from accessories.yaml or accessories_no_imu.yaml.'
    )

    # Conditionally select the accessories configuration file
    accessories_config_file_path = PythonExpression([
        "os.path.join('", pkg_rover_driver, "', 'config', 'accessories.yaml') if '", use_imu_arg, "' == 'true' else ",
        "os.path.join('", pkg_rover_driver, "', 'config', 'accessories_no_imu.yaml')"
    ])

    # It's better to pass the PythonExpression directly to parameters if the node supports it.
    # However, the original code reads the YAML to decide to launch nodes.
    # We need to make this part dynamic too, or always pass the file and let the node handle 'active'.
    # For now, let's assume we read the chosen file to maintain the structure.
    # This means the PythonExpression for path needs to be evaluated first.
    # This is typically done using OpaqueFunction.

    ld = LaunchDescription()
    ld.add_action(declare_use_imu_cmd)

    # Using OpaqueFunction to allow Python code execution at launch time to resolve the path
    def launch_accessories(context, *args, **kwargs):
        actual_accessories_config_path_str = LaunchConfiguration('accessories_config_file_path_resolved').perform(context)
        
        # Read the config file
        with open(actual_accessories_config_path_str, 'r') as f:
            accessories_config = yaml.load(f, Loader=yaml.FullLoader)
        
        actions_to_add = []
        # RP Lidar Setup
        if accessories_config.get('rplidar', {}).get('ros__parameters', {}).get('active', False):
            lidar_node = Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                name='rplidar',
                parameters=[actual_accessories_config_path_str],
                output='screen')
            actions_to_add.append(lidar_node)
        
        # BNO055 IMU Setup
        # The 'active' flag in the selected YAML file will control this
        if accessories_config.get('bno055', {}).get('ros__parameters', {}).get('active', False):
            bno055_node = Node(
                package = 'bno055',
                name = 'bno055',
                executable = 'bno055',
                parameters = [actual_accessories_config_path_str],
                remappings=[
                    ('/imu', '/imu/data')
                ])
            actions_to_add.append(bno055_node)

        # Realsense Node
        if accessories_config.get('realsense', {}).get('ros__parameters', {}).get('active', False):
            realsense_node = Node(
                package='realsense2_camera',
                name="realsense",
                executable='realsense2_camera_node',
                parameters=[actual_accessories_config_path_str],
                output='screen')
            actions_to_add.append(realsense_node)
        
        return actions_to_add
    
    # This OpaqueFunction will execute the PythonExpression for the path
    # and then call launch_accessories with the resolved path.
    # A bit convoluted due to the need to read YAML content within the launch file.
    # A cleaner way would be if all nodes just took the params file and handled 'active' internally.
    
    # Create a dummy LaunchConfiguration to hold the resolved path for OpaqueFunction
    # This is a workaround because OpaqueFunction doesn't directly accept PythonExpression results easily.
    # We can't directly use accessories_config_file_path in OpaqueFunction's context.
    # A better way is to use another OpaqueFunction to set a variable or use a custom LaunchSubstitution.

    # Simpler approach: Pass the PythonExpression for the path directly to the nodes.
    # The nodes will then load the correct file. The 'if active' check in this launch file
    # becomes problematic if we don't read the file here.
    # The original logic *requires* reading the file to decide if a node is launched.
    # So, we must resolve the path and read the file.

    # Let's use a simpler way to pass the conditionally resolved path to an OpaqueFunction
    # The PythonExpression itself can be evaluated by OpaqueFunction's context.
    
    from launch.actions import GroupAction # To group actions returned by OpaqueFunction
    from launch.actions import OpaqueFunction
    
    opaque_function_loader = OpaqueFunction(
        function=lambda context: [
            # LogInfo(msg=f"Accessories config path resolved to: {accessories_config_file_path.perform(context)}"),
            # Set a new LaunchConfiguration that can be used by the inner function. This is tricky.
            # A more direct way is to resolve path inside the opaque function.
            GroupAction(actions=launch_accessories(context, accessories_config_file_path_resolved=accessories_config_file_path.perform(context)))
        ]
    )
    ld.add_action(opaque_function_loader)

    return ld