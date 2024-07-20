# ROS2 Humble Robot Arm

This repository provides guidance on launching a URDF model with STL meshes using ROS2 Humble. The URDF model is in this package [robot-arm-pkg](https://github.com/smart-methods/arduino_robot_arm), created by [SmartMethods](https://github.com/smart-methods/).

## Introduction

This package allows users to visualize and manipulate a robot arm model using ROS2 Humble. It includes the necessary URDF files and STL meshes to represent the robot arm. This guide will walk you through the steps to set up the workspace, download the package, build it, and launch the visualization.

## Prerequisites

Before starting, ensure you have the following installed on your system:

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
- [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)



## Installation Steps

### 1. Create the Workspace

```bash
mkdir -p ~/robot_arm_ros2_ws/src
cd ~/robot_arm_ros2_ws/src
```
#### 1.1 Download the package, and remove extra files
```bash
git clone https://github.com/MoAlharsani/ros2-humble-robot-arm.git
```
```bash
mv ros2-humble-robot-arm/robot_arm_description ros2-humble-robot-arm/moveit_pkg .

```
```bash
rm -rf ros2-humble-robot-arm
```
#### 1.2 Build, source, and launch the package
```bash
cd ~/robot_arm_ros2_ws/
```
```bash
colcon build
```
```bash
source ~/robot_arm_ros2_ws/install/setup.bash
```
##### 1.2.1 To run the URDF Model In RViz
```bash
ros2 launch robot_arm_description display.launch.py
```

##### 1.2.2 To run the URDF Model In RViz with MoveIt Plugin
```bash
ros2 launch moveit_pkg demo.launch.py 
```



By following these steps, you should be able to visualize the robot arm model using ROS2 Humble.


