
# ros-go1-World

Welcome to the `ros-go1-world` repository, a collection of simulation worlds for the ROS (Robot Operating System) Gazebo simulator. This package includes a set of worlds that have been tailored for use with the `unitree_gazebo` package, enabling realistic and versatile simulation environments for robot simulation.

## Overview

The `ros-go1-world` package is designed to work seamlessly with the `unitree_gazebo` package, providing a rich set of simulation environments for testing and developing robotic applications. It includes a variety of worlds, each offering unique challenges and scenarios for robot simulation.

## Settings

### PyCharm
In PyCharm start window, select "Create Desktop Entry".
```shell
sudo nano /usr/share/applications/jetbrains-pycharm.desktop
```
Update the "Exec" line to below. (Otherwise it can't find the rospy package in venv)
```text
[Desktop Entry]
...
Exec=bash -i -c "~/Documents/pycharm-community-2024.1.1/bin/pycharm.sh"
...
```

### Virtual environment
Create a virtual environment in PyCharm and check "Inherit global site-packages" (Which will includes rospy)

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

1. **Launch the default Simulation**:
   - Use the `roslaunch` command to start the simulation with the `robot_simulation.launch` file:
     ```bash
     roslaunch unitree_gazebo robot_simulation.launch
     ```

This will load the default world "earth" and start the simulation environment.

2. **Launch the office_small Simulation**:
   - Use the `roslaunch` command to start the simulation with the `robot_simulation.launch` file:
     ```bash
     roslaunch unitree_gazebo robot_simulation.launch wname:=office_small
     ```

This will load the world "office_small" from `gazebo_worlds` and start the simulation environment.

## Reference
### Origin Repo [ros_unitree](https://github.com/macc-n/ros_unitree)