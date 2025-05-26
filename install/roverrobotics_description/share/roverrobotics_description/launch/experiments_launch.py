# experiments_launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch args
    map_arg = DeclareLaunchArgument(
        'map_file', default_value='maps/indoor_abc_lab2',
        description='Relative path (inside package) to map YAML without extension'
    )
    exp_arg = DeclareLaunchArgument(
        'experiment_type', default_value='1',
        description='Experiment ID: 1=all, 2=every2nd, etc.'
    )

    pkg_share = get_package_share_directory('roverrobotics_driver')

    # 1) Robot driver (zero)
    zero_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'zero.launch.py')
        )
    )

    # 2) Nav2 stack (map + localization + planners)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'map_file_name': LaunchConfiguration('map_file')
        }.items()
    )

    # 3) Your experiment node
    experiment_node = Node(
        package='roverrobotics_driver',
        executable='greenhouse_experiment_node',
        name='greenhouse_experiment_node',
        output='screen',
        parameters=[
            {'plants_config': 'config/indoor_configs/plants.yaml'},
            {'experiment_type': LaunchConfiguration('experiment_type')}
        ],
    )

    return LaunchDescription([
        map_arg,
        exp_arg,
        zero_launch,
        nav_launch,
        experiment_node,
    ])
# This launch file is used to run the Rover Robotics driver and the Nav2 stack