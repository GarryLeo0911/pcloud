# OAK-D Pcloud
## _Publish pointcloud data from the OAK-D camera with ROS_

## Installation

This package uses the `depthai_bridge` from `https://github.com/luxonis/depthai-ros`

```sh
cd catkin_ws/src
git clone git@github.com:h3ct0r/oakd_pcloud.git
cd catkin_ws
catkin build
```

## Execution

`roslaunch oakd_pcloud stereo_nodelet.launch`

## ROS 2 port (experimental)

This repository has been ported to ROS 2 (target: ROS 2 Humble by default). The port
replaces nodelets with rclcpp components and converts the publisher node to a
native rclcpp node. This is an initial port and may require additional
adjustments depending on your ROS 2 environment and the availability of
ROS2-compatible depthai/depthai_bridge packages.

Build (Linux / general instructions):

```bash
# from your ROS2 workspace src/
git clone git@github.com:h3ct0r/oakd_pcloud.git
cd ..
colcon build --symlink-install
```

Run (example):

```bash
source install/setup.bash
ros2 run oakd_pcloud stereo_rectified_rgb_node
```

Notes and assumptions:
- Target ROS 2 distro: Humble (change CMakeLists/package.xml if you prefer another distro).
- This port assumes a ROS2-compatible `depthai` / `depthai_bridge` is available. If not,
  the node currently publishes a minimal color preview as an example; you may want to
  integrate the ROS2 depthai bridge API or adapt the publisher logic to your environment.

If you want me to target a different ROS2 distro (e.g., Foxy, Galactic, Rolling), tell me and
I'll adapt the package.xml and CMake settings accordingly.
