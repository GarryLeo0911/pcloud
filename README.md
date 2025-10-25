# OAK-D Pcloud

A ROS2 package for publishing pointcloud data from the OAK-D camera.

## Overview

This package provides ROS2 nodes for:
- Publishing rectified stereo images and depth from OAK-D
- Applying median and WLS filtering to depth data
- Converting depth to pointcloud (planned)

## Dependencies

Required:
- ROS2 Jazzy (or another ROS2 distro if you adapt the dependencies)
- OpenCV
- depthai (optional - package can build without it)
- cv_bridge (optional - package can build without it)

## Installation

### Building from Source

```bash
# Clone the repository into your ROS2 workspace
cd ~/ros_ws/src
git clone https://github.com/GarryLeo0911/pcloud.git oakd_pcloud

# Install dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install ros-jazzy-cv-bridge ros-jazzy-depthai-ros

# Build the package
cd ~/ros_ws
colcon build --packages-select oakd_pcloud
```

### Building without DepthAI

If you don't have the DepthAI SDK or ROS2 packages installed, you can build the package in a limited mode:

```bash
cd ~/ros_ws
colcon build --packages-select oakd_pcloud --cmake-args -DBUILD_WITHOUT_DEPTHAI=ON
```

In this mode, the nodes will start but won't connect to actual camera hardware.

## Usage

Source your workspace:
```bash
source ~/ros_ws/install/setup.bash
```

### Running the Nodes

Basic stereo publisher:
```bash
ros2 run oakd_pcloud stereo_rectified_rgb_node
```

With median filter:
```bash
# Coming soon - launch file will be provided
```

### Published Topics

When running with DepthAI enabled:
- `/stereo/depth` (sensor_msgs/Image) - Raw depth data
- `/rectified_left/image` (sensor_msgs/Image) - Rectified left camera image
- `/rectified_right/image` (sensor_msgs/Image) - Rectified right camera image
- `/color/image` (sensor_msgs/Image) - Color camera preview

### Parameters

- `camera_name` - Name/identifier for the camera
- `camera_param_uri` - URI to camera calibration parameters

## Development Status

This is a ROS2 port of the original ROS1 package. Current status:
- [x] Basic node structure converted to ROS2
- [x] CMake/package.xml updated for ROS2
- [x] Stereo publisher node working
- [ ] Launch files need conversion to Python
- [ ] Full message_filters integration pending
- [ ] More testing needed

## License

This package is released under the MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgements

This is a ROS2 port of the original [oakd_pcloud](https://github.com/h3ct0r/oakd_pcloud) package.
