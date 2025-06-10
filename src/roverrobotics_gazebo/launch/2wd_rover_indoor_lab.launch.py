import os

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Set the resource path for Gazebo to find the map model BEFORE anything else
    roverrobotics_gazebo = get_package_share_directory('roverrobotics_gazebo')
    
    models_path = str(Path(roverrobotics_gazebo).parent.resolve())
    models_path += pathsep + os.path.join(roverrobotics_gazebo, 'models')
        
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        models_path
    )
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'roverrobotics_description'), 'urdf', 'rover_2wd.urdf')
    world = LaunchConfiguration('world')

    robot_desc = ParameterValue(
        Command(['xacro ', urdf]),
        value_type=str
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_lab_world_walled.sdf',
        description='World file to use in Gazebo'
    )
    
    gz_world_arg = PathJoinSubstitution([
        get_package_share_directory('roverrobotics_gazebo'),
        'worlds',
        world]
    )

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments=[(
                "gz_args", [" -r ", gz_world_arg]
            )]
    )
    
    # Spawn Rover Robot
    # Adjust spawn position based on map origin [-0.651, -4.25, 0]
    # Place robot in a free area of the map
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rover_miti",
            "-allow_renaming", "true",
            "-z", "0.1",
            "-x", "0.0",    # Adjust based on your map's free space
            "-y", "0.0",    # Adjust based on your map's free space
        ]
    )
    
    # ROS-Gazebo bridge with updated message types for Gazebo Fortress
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odometry/wheels@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
    )

    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    
    ld.add_action(gazebo_resource_path)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)

    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    return ld