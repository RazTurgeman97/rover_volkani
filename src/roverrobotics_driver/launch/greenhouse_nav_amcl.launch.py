import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rover_dir = get_package_share_directory('roverrobotics_driver')
    
    # Declare map file argument
    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(rover_dir, 'maps', 'greenhouse_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    map_file = LaunchConfiguration('map')
    
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
    
    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_link',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 12.0,
            'laser_min_range': 0.15,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'differential',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': 'scan',
            'map_topic': 'map',
            'set_initial_pose': False
        }]
    )
    
    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False}, 
                    {'yaml_filename': map_file}]
    )
    
    # Lifecycle Manager for Navigation
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )
    
    # Launch navigation backend
    nav_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rover_dir, '/launch/nav2_backend.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )
    
    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(declare_map_file_cmd)
    ld.add_action(zero_launch)
    ld.add_action(robot_localizer)
    ld.add_action(map_server)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_mgr)
    ld.add_action(nav_backend)
    
    return ld