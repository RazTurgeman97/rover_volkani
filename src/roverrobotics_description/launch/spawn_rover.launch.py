import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package share directories
    rover_pkg = get_package_share_directory('roverrobotics_description')
    map_pkg = get_package_share_directory('map2gazebo')

    # Launch arguments for world and model
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            os.getcwd(), 'src', 'map2gazebo', 'worlds', 'indoor_lab_world.sdf'
        ),
        description='Absolute path to SDF world file'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(rover_pkg, 'urdf', 'rover_2wd.urdf'),
        description='Absolute path to URDF (xacro-enabled)'
    )

    # Combine resource paths: rover models and map models
    rover_models = Path(rover_pkg).parent / 'models'
    map_models = os.path.join(map_pkg, 'models')
    gz_resources = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(rover_models) + pathsep + map_models + pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # Convert XACRO to URDF and publish robot_description
    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('model'), ' is_ignition:=True'
        ]),
        value_type=str
    )
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Launch Ignition Gazebo with specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': PythonExpression([
                "'", LaunchConfiguration('world'), " -v 4 -r'"
            ])
        }.items()
    )

    # Spawn the robot into the simulation
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover_2wd'
        ]
    )

    # Bridge Gazebo topics to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V]'
        ]
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        gz_resources,
        rsp_node,
        gazebo,
        gz_spawn,
        gz_bridge
    ])
