# Create this file: /home/rover/rover_workspace/src/roverrobotics_driver/launch/test_yolo_detection.launch.py

import os
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
    pkg_rover_driver = get_package_share_directory('roverrobotics_driver')
    
    # Launch arguments
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_rover_driver, 'models', 'yolo11n.onnx'),
        description='Path to YOLO model file'
    )
    
    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/realsense/color/image_raw',
        description='Camera topic to subscribe to'
    )
    
    declare_yolo_model_path = DeclareLaunchArgument(
        'yolo_model_path',
        default_value='/home/rover/rover_workspace/src/roverrobotics_driver/models/yolov11n.onnx',
        description='Path to YOLO model file (.onnx)'
    )
    
    declare_use_perception = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description='Whether to enable AI-based plant perception'
    )
    
    use_perception = LaunchConfiguration("use_perception")
    use_yolo = LaunchConfiguration("use_yolo")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    
    # Get launch configurations
    model_path = LaunchConfiguration('model_path')
    camera_topic = LaunchConfiguration('camera_topic')
    
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
    
    # # Plant Detection Node (YOLO only)
    # plant_detection_node = Node(
    #     package='roverrobotics_driver',
    #     executable='plant_detection_node',
    #     name='plant_detection_node',
    #     parameters=[{
    #         'model_path': model_path,
    #         'camera_topic': camera_topic,
    #         'camera_frame': 'camera_color_optical_frame',
    #         'map_frame': 'map',
    #         'confidence_threshold': 0.5,
    #         'nms_threshold': 0.4,
    #         'enable_yolo': True,
    #         'enable_traditional_detection': False,
    #         'publish_debug_image': True,
    #     }],
    #     output='screen'
    # )
    
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
            'target_classes': ['potted plant'],  # Only detect plant-related classes
            # 'min_plant_pixels': 500,    # Minimum size in pixels
            # 'max_plant_pixels': 10000,  # Maximum size in pixels
            
            # # Clustering for multiple detections of same plant
            # 'enable_clustering': True,
            # 'cluster_distance_threshold': 0.2,  # 20cm
            # 'cluster_min_detections': 2,
            # 'temporal_window': 3.0,
        }],
        condition=IfCondition(use_perception),
    )
    
    return LaunchDescription([
        declare_use_perception,
        declare_yolo_model_path,
        declare_model_path,
        declare_camera_topic,
        # zero_teleop_node,
        robot_launch_check,
        navigation_launch,
        plant_detection_node,
    ])