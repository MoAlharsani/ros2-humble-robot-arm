# ROS2 Humble Robot Arm

This repository provides guidance on launching a URDF model with STL meshes using ROS2 Humble. The URDF model is in this package [robot-arm-pkg](https://github.com/smart-methods/arduino_robot_arm),and created by [SmartMethods](https://github.com/smart-methods/).


## Installation Steps

### 1. Create the Workspace

```bash
mkdir -p ~/robot_arm_ros2_ws/src
cd ~/robot_arm_ros2_ws/src
```
#### 1.1 Download the package, and remove extra files
```bash
git clone https://github.com/MoAlharsani/ros2-humble-robot-arm.git
mv ros2-humble-robot-arm/robot_arm_description .
rm -rf ros2-humble-robot-arm
```
```bash
mv ros2-humble-robot-arm/robot_arm_description .
rm -rf ros2-humble-robot-arm
```
#### 1.2 Build, source, and launch the package
```bash
cd ~/robot_arm_ros2_ws/
colcon build
source ~/robot_arm_ros2_ws/install/setup.bash
ros2 launch robot_arm_description display.launch.py
```
```bash
source ~/robot_arm_ros2_ws/install/setup.bash
ros2 launch robot_arm_description display.launch.py
```




