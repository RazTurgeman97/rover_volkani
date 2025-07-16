#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import subprocess

def convert_yolo_model(context, *args, **kwargs):
    """Convert YOLO .pt model to .onnx format"""
    try:
        from ultralytics import YOLO
        
        # Get the launch arguments
        yolo_model_pt_path = context.launch_configurations.get('yolo_model_pt_path', 
            '/home/rover/rover_workspace/src/roverrobotics_driver/models/yolo11n.pt')
        yolo_model_path = context.launch_configurations.get('yolo_model_path', 
            '/home/rover/rover_workspace/src/roverrobotics_driver/models/yolov11n.onnx')
        
        print(f'[launch] Converting YOLO model from {yolo_model_pt_path} to {yolo_model_path}')
        
        # Create models directory if it doesn't exist
        models_dir = os.path.dirname(yolo_model_path)
        os.makedirs(models_dir, exist_ok=True)
        
        # Check if .pt file exists
        if not os.path.exists(yolo_model_pt_path):
            print(f'[launch] Warning: .pt file not found at {yolo_model_pt_path}, downloading yolov11n.pt...')
            # Download the model if it doesn't exist
            model = YOLO('yolo11n.pt')  # This will download if not present
            model.export(format='onnx', imgsz=640)
            
            # Move to the correct location
            downloaded_onnx = 'yolo11n.onnx'
            if os.path.exists(downloaded_onnx):
                import shutil
                shutil.move(downloaded_onnx, yolo_model_path)
                print(f'[launch] Model converted and saved to {yolo_model_path}')
        else:
            # Load existing .pt file and convert
            print(f'[launch] Loading model from {yolo_model_pt_path}')
            model = YOLO(yolo_model_pt_path)
            
            # Export to ONNX format
            model.export(format='onnx', imgsz=640)
            
            # Move the generated ONNX file to the desired location
            pt_dir = os.path.dirname(yolo_model_pt_path)
            pt_name = os.path.splitext(os.path.basename(yolo_model_pt_path))[0]
            generated_onnx = os.path.join(pt_dir, f'{pt_name}.onnx')
            
            if os.path.exists(generated_onnx) and generated_onnx != yolo_model_path:
                import shutil
                shutil.move(generated_onnx, yolo_model_path)
                print(f'[launch] Model converted and saved to {yolo_model_path}')
            elif os.path.exists(yolo_model_path):
                print(f'[launch] ONNX model already exists at {yolo_model_path}')
                
    except ImportError:
        print('[launch] Warning: ultralytics not installed, cannot convert YOLO model')
    except Exception as e:
        print(f'[launch] Error converting YOLO model: {e}')
    
    return []

def maybe_launch_robot(context, *args, **kwargs):
    try:
        ps = subprocess.check_output(['ps', 'aux'], encoding='utf-8')
        if ('zero.launch.py' in ps) or ('zero_teleop.launch.py' in ps):
            print('[launch] Robot already running, skipping robot_launch')
            return []
    except Exception as e:
        print(f'[launch] Error checking running processes: {e}')
    
    pkg_rover_driver = get_package_share_directory('roverrobotics_driver')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_driver, "launch", "zero_teleop.launch.py")
        )
    )]

def generate_launch_description():
    # Package directories
    pkg_rover_driver = get_package_share_directory('roverrobotics_driver')
    
    # Launch arguments
    declare_use_perception = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description='Whether to enable AI-based plant perception'
    )
    
    declare_use_yolo = DeclareLaunchArgument(
        'use_yolo',
        default_value='true',
        description='Whether to use YOLO for plant detection'
    )
    
    declare_yolo_model_pt_path = DeclareLaunchArgument(
        'yolo_model_pt_path',
        default_value='/home/rover/rover_workspace/src/roverrobotics_driver/models/yolo11n.pt',
        description='Path to YOLO model file (.pt)'
    )
    
    declare_yolo_model_path = DeclareLaunchArgument(
        'yolo_model_path',
        default_value='/home/rover/rover_workspace/src/roverrobotics_driver/models/yolov11n.onnx',
        description='Path to YOLO model file (.onnx)'
    )
    
    declare_experiment_type = DeclareLaunchArgument(
        'experiment_type',
        default_value='6',  # AI-driven adaptive inspection
        description='Type of experiment to run'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )

    # Add user confirmation argument
    declare_require_confirmation = DeclareLaunchArgument(
        'require_confirmation',
        default_value='true',
        description='Whether to require user confirmation before starting experiment'
    )
        
    use_perception = LaunchConfiguration("use_perception")
    use_yolo = LaunchConfiguration("use_yolo")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    experiment_type = LaunchConfiguration("experiment_type")
    use_rviz = LaunchConfiguration("use_rviz")
    require_confirmation = LaunchConfiguration("require_confirmation")
    
    # Robot launch check
    robot_launch_check = OpaqueFunction(function=maybe_launch_robot)
    
    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_driver, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            'slam': 'false',
            'use_sim_time': 'false'
        }.items()
    )

    # YOLO model conversion (runs before plant detection node)
    yolo_converter = OpaqueFunction(function=convert_yolo_model)

    # Smart Plant Detection Node with Filtering
    plant_detection_node = Node(
        package='roverrobotics_driver',
        executable='plant_detection_node',
        name='plant_detection_node',
        parameters=[{
            'model_path': yolo_model_path,
            'config_file': os.path.join(pkg_rover_driver, 'config', 'plant_detection_config.yaml'),
            'camera_topic': '/camera/realsense/color/image_raw',
            'depth_topic': '/camera/realsense/depth/image_rect_raw',  # Add depth for 3D positioning
            'camera_frame': 'camera_color_optical_frame',
            'map_frame': 'map',
            
            # YOLO settings
            'enable_yolo': True,  # Enable YOLO but with smart filtering
            'confidence_threshold': 0.4,  # Lower threshold, we'll filter afterwards
            'nms_threshold': 0.4,
            
            # Smart filtering enabled
            'enable_smart_filtering': True,
            'use_depth_filtering': True,
            'enable_temporal_consistency': True,
            
            # Plant-specific settings
            'target_classes': ['potted plant', 'plant'],  # Only detect plant-related classes
            'min_plant_pixels': 500,    # Minimum size in pixels
            'max_plant_pixels': 10000,  # Maximum size in pixels
            
            # Clustering for multiple detections of same plant
            'enable_clustering': True,
            'cluster_distance_threshold': 0.2,  # 20cm
            'cluster_min_detections': 2,
            'temporal_window': 3.0,
        }],
        condition=IfCondition(use_perception),
    )
    
    # Adaptive Plant Detector Node
    adaptive_plant_detector_node = Node(
        package='roverrobotics_driver',
        executable='adaptive_plant_detector',
        name='adaptive_plant_detector',
        parameters=[{
            'detection_config': os.path.join(pkg_rover_driver, 'config', 'plant_detection_config.yaml'),
        }],
        condition=IfCondition(use_perception),
    )

    # Perception Pipeline Node with enhanced clustering
    perception_pipeline_node = Node(
        package='roverrobotics_driver',
        executable='greenhouse_perception_pipeline',
        name='greenhouse_perception_pipeline',
        parameters=[{
            'fusion_enabled': True,
            'lidar_verification': True,
            'health_classification': True,
            'confidence_threshold': 0.6,
            # Plant instance grouping
            'spatial_grouping_enabled': True,
            'max_plant_radius': 0.4,           # Maximum radius of a single plant
            'min_leaf_area': 0.01,             # Minimum area for valid leaf detection
            'leaf_density_threshold': 0.7,     # Density threshold for plant clustering
        }],
        condition=IfCondition(use_perception),
    )
    
    # # Adaptive Navigation Manager
    # adaptive_nav_node = Node(
    #     package='roverrobotics_driver',
    #     executable='adaptive_greenhouse_navigator',
    #     name='adaptive_greenhouse_navigator',
    #     parameters=[{
    #         'replan_frequency': 10.0,
    #         'priority_diseased_plants': True,
    #         'approach_distance': 0.5
    #     }],
    #     condition=IfCondition(use_perception),
    # )
    
    # Enhanced Experiment Node with user confirmation
    experiment_node = Node(
        package='roverrobotics_driver',
        executable='greenhouse_experiment_node',
        name='greenhouse_experiment_node',
        parameters=[{
            'plants_config': 'config/indoor_configs/plants.yaml',  # Use relative path since pkg is prepended
            'use_dynamic_detection': False,  # Set to False for now to use static config
            'experiment_type': experiment_type,
            'n': 5,
            'use_imu': True,
            'approach_dist': 0.4,
            'corridor_dist': 0.8,
            'use_ai_perception': use_perception,
            'adaptive_navigation': True,
            'require_confirmation': require_confirmation,  # This should work now
            'log_dir': '/home/rover/advanced_rover_experiments/experiment_logs'
        }]
    )
    
    # Waypoint Navigation Manager
    waypoint_manager_node = Node(
        package='roverrobotics_driver',
        executable='waypoint_navigation_manager',
        name='waypoint_navigation_manager',
        parameters=[{
            'waypoint_tolerance': 0.2,
            'dynamic_replanning': True,
            'obstacle_avoidance': True
        }]
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output="screen",
        arguments=['-d', os.path.join(pkg_rover_driver, 'config', 'rviz_configs', 'greenhouse_experiment.rviz')],
        condition=IfCondition(use_rviz),
    )
    
    # Group all perception-related nodes
    perception_group = GroupAction([
        yolo_converter,  # Convert model first
        adaptive_plant_detector_node,  # Detect and number plants
        plant_detection_node,
        perception_pipeline_node,
        # adaptive_nav_node,
    ])
    
    return LaunchDescription([
        # Launch arguments
        declare_use_perception,
        declare_use_yolo,
        declare_yolo_model_pt_path,
        declare_yolo_model_path,
        declare_experiment_type,
        declare_rviz_arg,
        declare_require_confirmation,  # Add the confirmation argument

        # Core system
        robot_launch_check,
        navigation_launch,

        # Enhanced functionality
        perception_group,
        # waypoint_manager_node,
        experiment_node,

        # Optional visualization
        rviz_node,
    ])

