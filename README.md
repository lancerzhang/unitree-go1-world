
# ros-go1-World

Welcome to the `ros-go1-world` repository, a collection of simulation worlds for the ROS (Robot Operating System) Gazebo simulator. This package includes a set of worlds that have been tailored for use with the `unitree_gazebo` package, enabling realistic and versatile simulation environments for robot simulation.

## Contents

- **Overview**: Introduction to the `ros-go1-world` package.
- **Setup**: Step-by-step instructions for setting up the simulation environment.
- **Usage**: How to run the simulation with the provided worlds.

## Overview

The `ros-go1-world` package is designed to work seamlessly with the `unitree_gazebo` package, providing a rich set of simulation environments for testing and developing robotic applications. It includes a variety of worlds, each offering unique challenges and scenarios for robot simulation.

## Setup

To set up your simulation environment, follow these steps:

1. **Clone the `gazebo_worlds` Repository**:
   - Download the `gazebo_worlds` package from [macc-n/gazebo_worlds](https://github.com/macc-n/gazebo_worlds) and unzip it into your `catkin_ws/src/` folder.

2. **Clone the `ros-go1-world` Repository**:
   - Download this `ros-go1-world` repository and unzip it into your `catkin_ws/src/` folder.

3. **Build the Workspace**:
   - Navigate to your catkin workspace directory:
     ```bash
     cd ~/catkin_ws
     ```
   - Build the workspace using `catkin_make`:
     ```bash
     catkin_make
     ```

## Usage

Once the setup is complete, you can run the simulation as follows:

1. **Launch the Simulation**:
   - Use the `roslaunch` command to start the simulation with the `robot_simulation.launch` file:
     ```bash
     roslaunch unitree_gazebo robot_simulation.launch
     ```

This will load the default world ("office_small" from `gazebo_worlds`) and start the simulation environment, allowing you to test and develop your robotic applications.

## Reference
### Origin Repo [ros_unitree](https://github.com/macc-n/ros_unitree)