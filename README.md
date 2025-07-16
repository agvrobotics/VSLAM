````markdown
# AGV RTAB-Map + Astra Camera Setup (ROS 2 Humble)

This guide explains how to set up the **Orbbec Astra Pro camera** and build **RTAB-Map** with ROS 2 Humble support from source.
````

## Setting up Astra Camera

1. Clone the necessary repositories:

```bash
git clone https://github.com/orbbec/ros2_astra_camera.git
git clone https://github.com/libuvc/libuvc.git
````

2. Follow the installation instructions on their GitHub repository to build and install the camera drivers.

3. Set USB permissions (update device ID as needed). This is temporary incase you don't setup libusb rules

```bash
sudo chmod 666 /dev/bus/usb/001/009
```

4. Launch the Astra camera:

```bash
ros2 launch astra_camera astra_pro.launch.xml
```

---

## Setting up RTAB-Map (SLAM)

### Step 1: Install Dependencies

```bash
# ROS 2 message types and useful packages
sudo apt install ros-humble-nav2-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-tf2-sensor-msgs
sudo apt install ros-humble-pcl-conversions ros-humble-pcl-msgs ros-humble-tf2-geometry-msgs ros-humble-tf2-eigen

# Visualization, conversions, and other core tools
sudo apt install ros-humble-image-transport ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-tools
sudo apt install ros-humble-rviz2 ros-humble-visualization-msgs ros-humble-message-filters
sudo apt install ros-humble-octomap-msgs ros-humble-grid-map-ros

# PCL and sensor bridge
sudo apt install ros-humble-pcl-ros

# Required for building from source
sudo apt install ros-humble-rtabmap-msgs

# Optional but useful
sudo apt install ros-humble-depth-image-proc ros-humble-camera-info-manager ros-humble-compressed-image-transport
```

---

### Step 2: Build `rtabmap` Core (from source)

```bash
cd ~/Documents/AGV/
git clone https://github.com/introlab/rtabmap.git rtabmap_core
cd rtabmap_core
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

---

### Step 3: Build `rtabmap_ros` (from source)

1. Clone the ROS interface:

```bash
mkdir ~/Documents/AGV/ros_ws && cd ~/Documents/AGV/ros_ws
git clone https://github.com/introlab/rtabmap_ros.git
mv rtabmap_ros src
```

2. Build with `colcon`, pointing to the core library you installed above:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DRTABMap_DIR=~/Documents/AGV/rtabmap_core/build
```

##  Note

- If your laptop has limited resources ( < 16gb ram), consider building one package at a time.  
See [Build](./build.md) for detailed instructions.
---

After building successfully, source your workspace:

```bash
source install/setup.bash
```

