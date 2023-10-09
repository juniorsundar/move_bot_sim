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

For the visualisation in MoveIt!, a URDF file is required that defines the hybrid robot. The ```.xacro``` file can be accessed [here](./move_bot_description/urdf/move_bot.xacro).

Note that an additional part was modelled to act as the base joining the PAL Omni Base and the Kinova Arm. It was designed using **Blender**, and can be accessed [here](./kinova_gen3_description/meshes/base_bot_platform.stl).

MoveIt! is not optimal to control moving base, since the motion plan is taken with respect to a fixed 'world' TF. However, in order to motion plan for this model through MoveIt! certain steps were taken.

Since the moving base can be defined as a 3DOF system, in the URDF file, three additional dummy links ```x_base```, ```y_base```, and ```z_base```. The dummy 'world' frame is joined to ```x_base``` with a prismatic joint for translation in the x-axis. ```x_base``` is joined to ```y_base``` with a prismatic joint for translation in the y-axis. ```y_base``` is joined to the ```z_base``` with a revolute joint for rotation in the z-axis. This effective defines all the possible movements the base can make. Note, however, that this means the control is being performed on the base as a whole and not through the wheels, i.e. the base is moving as a static object while the wheels aren't rotating.

### Planning Groups

- **base** - Contains the three joints that define the 3DOF moving base.
- **arm** - Contains the seven joints that define the arm's configuration.
- **end_effector** - Contains the finger joint that controls the gripper.

*refer [here](./move_bot_moveit_config/config/move_bot.srdf).*

### Motion Planner Selection

- **base** - We are using *RRTConnect* since the dimensionality of this system is low, and speed and efficiency of plan is important.
- **arm** - We are using *PRMstar* since this has a higher dimensionality. Furthermore, it also considers the optimality of the path, pruning out poor paths if there is a lower cost alternative.
- **end_effector** - No planner required since.

*refer [here](./move_bot_moveit_config/config/ompl_planning.yaml)*

### Execute Visualisation

To replicate the above visualisation, run the following bash command:

```bash
roslaunch move_bot_visualisation move_bot_visualisation.launch
```

The robot's execution is defined in [```move_bot.py```](./move_bot_visualisation/scripts/move_bot.py).

## Method 2: ROS Navigation Stack with Simulation

![move_base_visualisation](./media/move_bot_demo-move_base.gif)