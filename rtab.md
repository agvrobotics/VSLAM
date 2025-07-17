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

This will generate a `frames.pdf` file with a visual map of your current TF structure. This will only work if the problem stated at the end is fixed.

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

## Known Problem: Split TF Trees

### Problem:

> RTAB-Map throws a warning:
>
> `Could not find a connection between 'odom' and 'camera_color_optical_frame'... Tf has two or more unconnected trees.`

This occurs because `odom` is not connected to `base_link`.

---

### Not the Cause:

Your static transform:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
```

...is **correct** and **not the issue**.

---

### Untested fix:

You must provide a transform (static or dynamic) from `odom` → `base_link`.

For testing purposes:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

---

### 🔄 TF Tree Before Fix:

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
