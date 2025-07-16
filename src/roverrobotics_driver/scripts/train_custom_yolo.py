# Create a script to prepare training data: train_custom_yolo.py

import os
from ultralytics import YOLO
import yaml

def train_custom_plant_model():
    """Train a custom YOLO model for greenhouse plants"""
    
    # Create dataset configuration
    dataset_config = {
        'path': '/home/rover/plant_dataset',
        'train': 'images/train',
        'val': 'images/val',
        'names': {
            0: 'healthy_plant',
            1: 'diseased_plant',
            2: 'plastic_plant',
            3: 'background'
        }
    }
    
    # Save dataset config
    with open('/home/rover/plant_dataset/dataset.yaml', 'w') as f:
        yaml.dump(dataset_config, f)
    
    # Load base model
    model = YOLO('yolo11n.pt')
    
    # Train the model
    results = model.train(
        data='/home/rover/plant_dataset/dataset.yaml',
        epochs=100,
        imgsz=640,
        batch=16,
        name='greenhouse_plants',
        project='/home/rover/rover_workspace/src/roverrobotics_driver/models'
    )
    
    # Export to ONNX for deployment
    model.export(format='onnx')
    
    return results

if __name__ == "__main__":
    train_custom_plant_model()