# Move Bot Visualisation

This is a ROS package for visualising the motion-planning and movement of a hybrid robot consisting of a PAL Omni Base, a Kinova Gen3 robotic arm, and a Robotiq 85 gripper.

In this repository, we look at two possible ways to control the robot. The first strategy goes the MoveIt! path. The second strategy goes the path of the ROS Navigation. Both have their advantages and disadvantages which will be discussed in the respective section.

## Prerequisite

- Ubuntu 20.04
- ROS Noetic Ninjemys
- MoveIt! Motion Planning Framework
- Gazebo 11

## Installation

It is assumed that ROS Noetic Ninjemys is installed in the system. If not, please refer to [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) for step-by-step installation instructions.

It is also assumed that MoveIt! is installed. If not, run the following bash command:

```bash
sudo apt install ros-noetic-moveit
```

To control the robots in simulation, we need to install the ```ros_control``` and ```ros_controllers``` packages.

```bash
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-diff-drive-controller
```

Initialise catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

This package is all-encompassing, in that it contains all the robot description files you need. So clone this repository into the catkin workspace.

```bash
git clone https://github.com/juniorsundar/move_bot_sim.git
```

Build catkin workspace.

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Method 1: MoveIt! with Visualisation



![moveit_visualisation](./media/move_bot_demo-moveit.gif)

## Method 2: ROS Navigation Stack with Simulation

![move_base_visualisation](./media/move_bot_demo-move_base.gif)