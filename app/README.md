# xArm32 URDF Description

## Description

This folder contains the URDF description of the xArm32 robot for ROS 2 Humble, including Gazebo models.

## Prerequisites

- Docker (for environment isolation)
- ROS 2 Humble
- Colcon (ROS build tool)

## Installation and Setup

### Option 1: Using Docker (Recommended)

#### 1. Pull the Docker image

```bash
docker pull osrf/ros:humble-desktop
```

#### 2. Launch the container

```bash
docker run -it --rm \
  -v /mnt/c/%USER_PATH%/DeepSight-Nebula/:/ros2_ws \
  -w /ros2_ws \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --net=host \
  osrf/ros:humble-desktop
```

### Option 2: Direct Installation on Ubuntu

```bash
sudo apt update
sudo apt install ros-humble-urdf-tutorial
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Build and Launch

### 1. Build the project

```bash
cd app
colcon build
source install/setup.bash
```

### 2. Launch visualization

**Option A: Using the full path**

```bash
ros2 launch urdf_tutorial display.launch.py model:=/ros2_ws/app/src/xArm32_urdf_description/urdf/xArm32_urdf.xacro
```

**Option B: After building (simpler)**

```bash
ros2 launch xArm32_urdf_description display.launch.py
```
