# ros2_rtos_assign

# ROS2 Turtlesim Path Planner with avoiding the obstacles

With the help of the "turtlesim" package using ROS2, this project implements a path planning algorithm. The turtle robot navigates towards a target position with the help of proportional control and dynamically avoids the obstacles. This Readme file provides an step by step approach to set up, build, and run the project.

## Features
- Proportional control scheme for smooth motion control.
- Obstacle avoidance at a designated zone in the grid.
- User-defined start position and heading orientation.

## Requirements
- ROS2 Humble
- `turtlesim` package (pre-installed in ROS2 Humble)
- Python 3

## Project Structure
- **Workspace**: `ros2_rtos`
- **Package**: `ros2_rtos_pkg`
- **Main Script**: `turtle_path_planner.py`

## Setup Instructions

### 1. Clone the Repository
First, clone the GitHub repository for this project. If you haven't cloned it yet, create a GitHub repository named `ros2_rtos_assign` and generate a PAT (Personal Access Token) for secure access.

### 2. Create,Initialize the ros2 workspace
mkdir -p ~/ros2_rtos/src
cd ~/ros2_rtos/src

### 3. Build workspace
cd ~/ros2_rtos
colcon build

### 4. Source the setup file
source install/setup.bash

### 5. Start the turtlesim node
ros2 run turtlesim turtlesim_node

### 6. Run the Path Planner
source ~/ros2_rtos/install/setup.bash
ros2 run ros2_rtos_pkg turtle_path_planner



