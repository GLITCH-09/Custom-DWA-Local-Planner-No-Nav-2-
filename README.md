# DWA Local Planner (No Nav2)

A custom implementation of the Dynamic Window Approach (DWA) local planner for ROS2, designed to work independently of the Nav2 stack.

## Description

This planner implements the Dynamic Window Approach algorithm for local path planning and obstacle avoidance. It generates and evaluates multiple possible trajectories in real-time, selecting the optimal path based on:
- Goal proximity
- Obstacle avoidance
- Forward velocity preference

## Features

- Real-time trajectory generation and evaluation
- Obstacle avoidance using laser scan data
- Visualization of planned paths in RViz
- Simple goal input interface
- Configurable parameters for speed and navigation

## Prerequisites

- Ubuntu - 24.04
- ROS2 (Tested on ROS2 Jazzy)
- Python 3.12
- Required ROS2 packages:
  - geometry_msgs
  - nav_msgs
  - sensor_msgs
  - visualization_msgs
  - tf_transformations

## Demo Video  

<p align="center">
  <a href="https://drive.google.com/file/d/1kn681qXGh3zXfTCJqKTDBMm-vYjhEUPi/view?usp=drive_link">
    <img src="https://drive.google.com/file/d/1EhX9MgFx0HWDgWcLg13WKlg5d79RkDwq/view?usp=drive_link" alt="Watch the video" width="600"/>
  </a>
</p>


## Installation

```bash
# Create a ROS2 workspace (if it does'nt exist already)
mkdir -p ~/ros2_dwa_ws/src
cd ~/ros2_dwa_ws/src

# Clone the repository
git clone https://github.com/GLITCH-09/Custom-DWA-Local-Planner-No-Nav-2-.git dwa_local_planner

# Build the package
cd ~/ros2_dwa_ws
colcon build --packages-select dwa_local_planner

# Source the workspace
source install/setup.bash
```

## Usage

1. Start your Gazebo simulator (ensure it publishes `/odom` and `/scan` topics):
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

3. Launch the DWA planner:
```bash
ros2 run dwa_local_planner dwa_planner
```

3. Enter the goal coordinates when prompted:
```
Enter your desired co-ordinates below:
Goal X: <enter_x_coordinate>
Goal Y: <enter_y_coordinate>
```
4. Launch RViz by running:
```
rviz2
```
  - In **RViz**, click **Add → By Topic** and select **visual_paths** to view the planned trajectories.
  - In **RViz**, click **Add → By Topic** and select **laser_scan** for better visualization of the percevied environment.


## Configuration

Key parameters in `dwa_planner.py`:
- `max_speed`: Maximum linear velocity (default: 0.35 m/s)
- `max_turn`: Maximum angular velocity (default: 2.0 rad/s)
- `step_time`: Control loop frequency (default: 0.05s - 20Hz)
- `footprint`: Robot footprint radius (default: 0.55m)

## Topics

### Subscribed Topics
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/scan` (sensor_msgs/LaserScan): Laser scan data

### Published Topics
- `/cmd_vel` (geometry_msgs/TwistStamped): Velocity commands
- `/visual_paths` (visualization_msgs/Marker): Visualization markers for RViz


## Authors

- Gaarv Khanna (Khanna.gaarv@gmail.com)
