# ROS2 Humble Robot Arm

This repository provides guidance on launching a URDF model with STL meshes using ROS2 Humble. The URDF model is in this package [robot-arm-pkg](https://github.com/smart-methods/arduino_robot_arm),and created by [SmartMethods](https://github.com/smart-methods/).

This tutorial emphasizes understanding the process of setting up a ROS workspace and building the package step by step, rather than a one-click installation.

## Installation Steps

### 1. Create the Workspace

```bash
mkdir -p ~/robot_arm_ros2_ws/src
cd ~/robot_arm_ros2_ws/src
```
#### 1.1 To install the package quickly, setup the package using these commands
```bash
git clone https://github.com/MoAlharsani/ros2-humble-robot-arm.git
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

### 2. Create the Package

```bash
ros2 pkg create robot_arm_description
```

#### 2.1 Remove Unnecessary Directories

```bash
rm -rf include/ src/
```

#### 2.2 Build the Workspace

```bash
cd ~/robot_arm_ros2_ws/
colcon build
```

### 3. Create these directories to organize files

```bash
cd ~/robot_arm_ros2_ws/src/robot_arm_description/
mkdir urdf rviz launch meshes
```

### 4. Add URDF and Mesh Files

#### 4.1 Download the URDF File

```bash
cd ~/robot_arm_ros2_ws/src/robot_arm_description/urdf
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/urdf/robot_arm.urdf
```

#### 4.2 Download the STL Meshes

```bash
cd ~/robot_arm_ros2_ws/src/robot_arm_description/meshes
mkdir stl
cd stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/arm_1.stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/arm_2.stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/base.stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/cube.stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/gripper.stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/gripper_1.stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/waist.stl
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/meshes/stl/waist_1.stl
```

#### 4.3 Upload the Launch File

```bash
cd ~/robot_arm_ros2_ws/src/robot_arm_description/launch
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/launch/display.launch.py
```

#### 4.4 Download the RViz Configuration

```bash
cd ~/robot_arm_ros2_ws/src/robot_arm_description/rviz
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/rviz/config.rviz
```

### 5. Update CMakeLists.txt

```bash
cd ~/robot_arm_ros2_ws/src/robot_arm_description/
rm -rf CMakeLists.txt
curl -O https://raw.githubusercontent.com/MoAlharsani/ros2-humble-robot-arm/main/robot_arm_description/CMakeLists.txt
```

### 6 Build, source, and launch the package
```bash
cd ~/robot_arm_ros2_ws/
colcon build
source ~/robot_arm_ros2_ws/install/setup.bash
ros2 launch robot_arm_description display.launch.py
```
