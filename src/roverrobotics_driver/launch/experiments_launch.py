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
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='Whether to use IMU data for localization and accessories.'
    )
    exp_arg = DeclareLaunchArgument(
        'experiment_type', default_value='1',
        description='Experiment ID: 1=all, 2=every2nd, 3=Nth, 4=list, 5=realtime'
    )
    n_plants_arg = DeclareLaunchArgument(
        'n_plants', default_value='3',
        description='Value of N for experiment_type 3 (every Nth plant).'
    )
    infected_plants_list_str_arg = DeclareLaunchArgument(
        'infected_plants_list_str', default_value='',
        description='Comma-separated list of plant IDs for experiment_type 4.'
    )

    pkg_share = get_package_share_directory('roverrobotics_driver')
    gazebo_pkg_share = get_package_share_directory('roverrobotics_gazebo')

    # 1) Robot driver (zero)
    zero_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'zero.launch.py')
        ),
        launch_arguments={'use_imu': LaunchConfiguration('use_imu')}.items()
    )
    
    zero_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', '2wd_rover_indoor_lab.launch.py')
        ),
        launch_arguments={'use_imu': LaunchConfiguration('use_imu')}.items()
    )

    # 2) Nav2 stack (map + localization + planners)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_launch_indoor.py')
        ),
        launch_arguments={'use_imu': LaunchConfiguration('use_imu')}.items()
    )
    
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_configs', 'custom_slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
        

    # 3) Your experiment node
    experiment_node = Node(
        package='roverrobotics_driver',
        executable='greenhouse_experiment_node',
        name='greenhouse_experiment_node',
        output='screen',
        parameters=[
            {'plants_config': 'config/indoor_configs/plants.yaml'},
            {'experiment_type': LaunchConfiguration('experiment_type')},
            {'n': LaunchConfiguration('n_plants')},
            {'infected_plants_list': LaunchConfiguration('infected_plants_list_str')}
        ],
    )

    return LaunchDescription([
        use_imu_arg,
        exp_arg,
        n_plants_arg,
        infected_plants_list_str_arg,
        # zero_launch,
        zero_gazebo_launch,
        rviz,
        nav_launch,
        experiment_node,
    ])
# This launch file is used to run the Rover Robotics driver and the Nav2 stack