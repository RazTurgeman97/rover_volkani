# Experiment Manager

This package contains a ROS 2 lifecycle node that orchestrates experiment runs in the greenhouse.

## Quick start

1. Build the workspace:

```bash
colcon build
```

2. Source the environment:

```bash
source install/setup.bash
```

3. Launch the node:

```bash
ros2 run experiment_manager experiment_manager
```

## Parameters

- **ros2_launch_cmd** (string, optional)
  - Command executed for each experiment. Defaults to `ros2 launch roverrobotics_driver experiments_launch.py`.
  - The referenced launch file accepts parameters such as `experiment_type`, `use_imu`, `n_plants` and `infected_plants_list_str`.

## Prerequisites

- ROS 2 installed (tested with Humble).
- Required packages, including `roverrobotics_driver`, must be built in this workspace.
