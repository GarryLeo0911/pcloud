# OAK-D Pcloud (ROS 2 Jazzy)
Publish OAK-D streams and simple filtered images on ROS 2 Jazzy.

## Build (ROS 2 Jazzy)

Prereqs:
- ROS 2 Jazzy setup sourced
- DepthAI C++ SDK installed and discoverable by CMake (`find_package(depthai REQUIRED)`).
- OpenCV available.

```bash
cd ~/ros2_ws/src
# place this package here
cd ..
colcon build --packages-select oakd_pcloud
source install/setup.bash
```

## Run

- Preview publisher (publishes `preview/image`):
```bash
ros2 launch oakd_pcloud preview.launch.py
```

- Median filter (subscribes to `image`, publishes `image_filtered`):
```bash
ros2 launch oakd_pcloud median_filter.launch.py
```

Parameters (median_filter): `kernel_size` (odd int), `iterations` (int).

Notes:
- This is a focused ROS 2 port. ROS 1 nodelets and dynamic_reconfigure cfg files are retained for reference but not built.
- depthai_bridge is not required; images are converted via cv_bridge.
