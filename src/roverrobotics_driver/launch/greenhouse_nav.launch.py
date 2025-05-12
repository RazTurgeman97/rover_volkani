import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rover_dir = get_package_share_directory('roverrobotics_driver')
    
    # Declare map name argument
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='greenhouse_map',
        description='Name of the map (without extension)'
    )
    
    # Get the full path to map files using substitutions
    map_dir = os.path.join(rover_dir, 'maps')
    map_name = LaunchConfiguration('map_name')
    
    # Use PathJoinSubstitution for properly combining paths
    map_yaml_file = PathJoinSubstitution([
        get_package_share_directory('roverrobotics_driver'),
        'maps', 
        map_name
    ])
    
    # Launch the base rover driver with sensors
    zero_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rover_dir, '/launch/zero.launch.py'])
    )
    
    # Launch robot localizer
    robot_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rover_dir, '/launch/robot_localizer.launch.py']),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )
    
    # Use fixed string for parameters since they need to be evaluated immediately
    map_file_path = os.path.join(map_dir, 'greenhouse_map')
    
    # Launch SLAM Toolbox in localization mode with the serialized map
    localization_params = os.path.join(rover_dir, 'config/slam_configs', 'mapper_params_localization.yaml')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            localization_params,
            {'use_sim_time': False},
            {'map_file_name': map_file_path}
        ]
    )
    
    # Launch navigation backend
    nav_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rover_dir, '/launch/nav2_backend.py']),
        launch_arguments={
            'map': map_yaml_file + '.yaml',
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )
    
    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(declare_map_name_cmd)
    ld.add_action(zero_launch)
    ld.add_action(robot_localizer)
    ld.add_action(slam_toolbox_node)
    ld.add_action(nav_backend)
    
    return ld