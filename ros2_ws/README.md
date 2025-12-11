# Usage of VACOP_simulation

##This project is still in development.

## Requirements

- Ubuntu 22.02 LTS
- ROS 2 Humble
- Webots R2025a or higher

## Installation

```bash
cd ros2_ws
sudo apt update
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros2-humble-webots-ros2-control
sudo apt install -y ros2-humble-webots-ros2-driver
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y ros-humble-rqt-image-view
pip install -r requirements.txt
```

## Compiling the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

## Launching the simulation

```bash
ros2 launch vacop robot_launch.py
```

# Manual control of the VACOP

```bash
source install/setup.bash
ros2 run vacop teleop
```

Control via ZQSD, space for emergency brake.

# View camera POV

```bash
ros2 run vacop camera_view
```

# View LiDAR scan

```bash
rviz2
```

1. Display -> Fixed Frame = lidar
2. Click on Add
3. Select PointCloud2 and topic = /scan/point_cloud

# Quick fixes

If you have an error `ModuleNotFoundError` verify that you installed all the packages

If you have the `AtributeError: _ARRAY_API not found` you have a too recent version of NumPy
