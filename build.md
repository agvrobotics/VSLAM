### Building `rtabmap_ros` Packages on Low-Spec Machines

If your laptop has limited resources, it's recommended to build one package at a time.
After each successful build, be sure to source the environment:

```bash
source install/setup.bash
```

Use the following command structure for each package:

```bash
colcon build --packages-select <package_name> --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DRTABMap_DIR=~/Documents/AGV/rtabmap/rtabmap_core/build
```

Build the packages in the following order:

1. `rtabmap_msgs` – builds the message types
2. `rtabmap_conversions` – depends on messages
3. `rtabmap_util` – utility nodes, uses conversions
4. `rtabmap_sync` – synchronization tools
5. `rtabmap_odom` – odometry
6. `rtabmap_slam` – core SLAM logic wrapper
7. `rtabmap_viz` – additional visualization tools
8. `rtabmap_launch` – launch files
9. `rtabmap_examples` – example scripts and demos
10. `rtabmap_demos` – ready-made demo launchers
11. `rtabmap_ros` – main launch and integration node
12. `rtabmap_rviz_plugins` – RViz visualization tools
13. `rtabmap_python` – Python utilities

After building individually, try building the whole thing all together

```bash
sudo updatedb
locate RTABMapConfig.cmake #checks path of your *.cmake
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DRTABMap_DIR=~/Documents/AGV/rtabmap/rtabmap_core/build
```

## Getting started with RTAB-MAP
- Refer to [RTAB](./rtab.md)