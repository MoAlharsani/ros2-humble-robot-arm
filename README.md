# ROS2 Humble Robot Arm

This repository provides guidance on launching a URDF model with STL meshes using ROS2 Humble. The URDF model is in this package [robot-arm-pkg](https://github.com/smart-methods/arduino_robot_arm), created by [SmartMethods](https://github.com/smart-methods/).

## Introduction

This package allows users to visualize and manipulate a robot arm model in ROS2 Humble evniroment with RViz and MoveIt2. It includes the necessary URDF files and STL meshes to represent the robot arm. This guide will walk you through the steps to set up the workspace, download the package, build it, and launch the visualization. 

### Overview of Packages:
1. robot_arm_description:
    This package contains launch files to visualize the robot arm's URDF model in RViz. You'll be able to observe the robot arm in a 3D environment, inspecting its structure and components.
2. moveit_pkg:
In addition to RViz visualization, this package includes launch files for MoveIt!, a powerful motion planning framework. With MoveIt!, you can control the robot arm using Inverse Kinematics (IK). This means you can manipulate the end effector coordinates, and MoveIt! will automatically adjust the joint positions accordingly, enabling intuitive control and motion planning.

## Prerequisites

Before starting, ensure you have the following installed on your system:

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
- [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
- [MoveIt2](https://moveit.ros.org/install-moveit2/binary/)



## Installation Steps

### 1. Create the Workspace

```bash
mkdir -p ~/robot_arm_ros2_ws/src
cd ~/robot_arm_ros2_ws/src
```
### 2. Download the package, and remove extra files
```bash
git clone https://github.com/MoAlharsani/ros2-humble-robot-arm.git
```
```bash
mv ros2-humble-robot-arm/robot_arm_description ros2-humble-robot-arm/moveit_pkg .

```
```bash
rm -rf ros2-humble-robot-arm
```
### 3. Build, source, and launch the package
```bash
cd ~/robot_arm_ros2_ws/
```
```bash
colcon build
```
```bash
source ~/robot_arm_ros2_ws/install/setup.bash
```
#### 3.1 To run the URDF Model In RViz
```bash
ros2 launch robot_arm_description display.launch.py
```

#### 3.2 To run the URDF Model In RViz with MoveIt Plugin
```bash
ros2 launch moveit_pkg demo.launch.py 
```



By following these steps, you should be able to visualize the robot arm model using ROS2 Humble.


