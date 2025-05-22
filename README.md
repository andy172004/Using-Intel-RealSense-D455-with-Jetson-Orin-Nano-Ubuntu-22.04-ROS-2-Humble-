# Using Intel RealSense D455 with Jetson Orin Nano (Ubuntu 22.04 + ROS 2 Humble)

This guide walks you through setting up the **Intel RealSense D455** camera with a **Jetson Orin Nano** running **Ubuntu 22.04** and **ROS 2 Humble**. You'll also learn how to perform SLAM using **RTAB-Map** and visualize data using **RViz2**.

---

## 🛠️ Prerequisites

* Jetson Orin Nano with **Ubuntu 22.04**
* **ROS 2 Humble** installed and sourced ([install guide below](#references))
* Intel **RealSense D455** camera
* Basic knowledge of ROS 2 (nodes, topics, launch files)

---

## ✅ Step 1: Install Required Packages

Open a terminal and run:

```bash
# RealSense ROS 2 packages
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# RTAB-Map for SLAM
sudo apt install ros-humble-rtabmap-ros ros-humble-rtabmap-util ros-humble-rtabmap-slam

# Visualization tools
sudo apt install ros-humble-rviz2 ros-humble-rqt-*
```

---

## 📷 Step 2: Test the RealSense Camera

### Launch the RealSense node:

```bash
ros2 launch realsense2_camera rs_launch.py
```

### In another terminal, view the image output:

```bash
ros2 run rqt_image_view rqt_image_view
```

Choose one of the topics such as:

* `/camera/infra1/image_rect_raw`
* `/camera/infra2/image_rect_raw`

If infra topics are unavailable, try enabling them explicitly:

```bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true enable_color:=false
```

### Optional: Use the Intel RealSense Viewer

```bash
realsense-viewer
```

---

## 🗜️ Step 3: Mapping with RTAB-Map

Once your camera is working, you're ready to map the environment.

### Option 1: Example Launch Files

Example RTAB-Map launch files are located at:

```
/opt/ros/humble/share/rtabmap_examples/
```

### Option 2: Launch Custom RTAB-Map SLAM

To start mapping with your D455:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
rgb_topic:=/camera/camera/color/image_raw \
depth_topic:=/camera/camera/depth/image_rect_raw \
camera_info_topic:=/camera/camera/color/camera_info \
frame_id:=camera_link \
odom_frame_id:=odom \
map_frame_id:=map \
approx_sync:=true \
qos:=1
```

🔀 If using **stereo mapping**, change the topics to use `/infra1` and `/infra2`.

### Tips:

* **Move the camera slowly** to generate a high-quality map.
* You can **pause mapping** using the spacebar or GUI button.
* Export the point cloud from the "File" menu, or view it from the "View" tab.

---

## 🛈 Visualize Map in RViz2

Open another terminal and run:

```bash
ros2 run rviz2 rviz2
```

### In RViz2:

* Click `Add` → choose `PointCloud2`
* Set the topic to: `/rtabmap/cloud_map`
* Set `Fixed Frame` to: `map`

---

## 🔗 References

* 📘 [ROS 2 Humble Installation (Official Guide)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* 📷 [Intel RealSense D455 Specs](https://www.intelrealsense.com/depth-camera-d455/)
* 🤖 [RTAB-Map ROS 2 Documentation](http://wiki.ros.org/rtabmap_ros)

---

## 📜 License

You are free to use, share, and modify this content with attribution.
