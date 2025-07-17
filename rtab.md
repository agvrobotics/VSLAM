# Astra Pro + RTAB-Map + TF Setup (ROS2)

### Visualize the Camera TF Tree in RViz

```bash
rviz2
```

Expected TF structure:

```
base_link
  └── camera_link
      ├── camera_color_frame
      │    └── camera_color_optical_frame
      └── camera_depth_frame
           └── camera_depth_optical_frame
```

---

### Launch the Astra Camera & Static Transform

```bash
ros2 launch astra_camera astra_pro.launch.xml

# Publish static transform from robot base to camera mount
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
```

---

### Check Your TF Tree

```bash
ros2 run tf2_tools view_frames
```

This will generate a `frames.pdf` file with a visual map of your current TF structure.

---

### Get the Camera Frame ID

```bash
ros2 topic echo /camera/color/image_raw --once
```

Expected output snippet:

```yaml
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: "camera_color_optical_frame"
```

---

### Launch RTAB-Map

Use the `frame_id` from the previous step (`camera_color_optical_frame`):

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  depth_topic:=/camera/depth/image_raw \
  rgb_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_color_optical_frame \
  approx_sync:=true \
  rtabmap_args:="--delete_db_on_start"
```

---

## Known Problems 


### 1. Split TF Trees:

> RTAB-Map throws a warning:
>
> `Could not find a connection between 'odom' and 'camera_color_optical_frame'... Tf has two or more unconnected trees.`

This occurs because `odom` is not connected to `base_link`.

---

### TF Tree:

```
odom
 └─ parent: map

base_link
 └─ parent: ??? (no parent shown)

camera_link
 └─ parent: base_link

camera_color_frame
 └─ parent: camera_link

camera_color_optical_frame
 └─ parent: camera_color_frame
```

---

Once `odom → base_link` is added, RTAB-Map will be able to compute camera motion relative to odometry and begin mapping.

Here's a clean and well-structured version of that section for your README, with consistent formatting and clear explanation:

---

### 2. Axis Alignment vs TF Timing — Choosing the Right `frame_id`

When launching RTAB-Map, the `frame_id` you specify determines **how the 3D map is oriented**, and whether you might encounter **TF timing errors**.

---

#### Default Behavior (Optical Frame Convention)

If you use the camera's default frame (e.g., `camera_color_optical_frame`), the map will be built using the **optical frame axis convention**:

* **Z** → Forward
* **X** → Right
* **Y** → Down

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  depth_topic:=/camera/depth/image_raw \
  rgb_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_color_optical_frame \
  approx_sync:=true \
  rtabmap_args:="--delete_db_on_start"
```

* Avoids TF errors — image data is already in the correct frame.
* Works well for quick testing or simple visualization.
* The resulting 3D map will appear tilted or floating (optical frames don’t align with world axes).

---

#### Aligned with Robot Axes (`base_link`)

To orient the map correctly with the robot’s natural axes (X = forward, Z = up), use:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  depth_topic:=/camera/depth/image_raw \
  rgb_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=base_link \
  approx_sync:=true \
  rtabmap_args:="--delete_db_on_start"
```
* The map is aligned to your robot frame (`base_link`) — perfect for SLAM, navigation, or multi-sensor fusion.
* You may get the following **TF timing error**:

```bash
[ERROR] MsgConversion.cpp:2182::convertRGBDMsgs()
TF of received image for camera 0 at time ... is not set!
```

This happens because RTAB-Map attempts to **lookup the transform between `camera_color_optical_frame` and `base_link` at the exact timestamp of the image**, and that transform may not be ready in time.


