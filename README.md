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
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info frame_id:=camera_link odom_frame_id:=odom map_frame_id:=map approx_sync:=true qos:=1
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
📊 Working with Point Cloud in Python using Open3D



Once you export your point cloud (e.g., as a .pcd or .ply file), you can use the following script to visualize it with Open3D:

Install Open3D (if not already installed):

pip install open3d

Basic Visualization:

import open3d as o3d

# Load your PCD or PLY file
pcd = o3d.io.read_point_cloud("map.pcd")  # or "map.ply"

# Visualize
o3d.visualization.draw_geometries([pcd])

🔍 Example: Floor Plane Detection and Waypoint Extraction

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA

# Step 1: Load Point Cloud
pcd = o3d.io.read_point_cloud("cloud4.ply")

# Step 2: Downsample and estimate normals
pcd_down = pcd.voxel_down_sample(voxel_size=0.02)
pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Step 3: Plane Segmentation (find floor)
plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
[a, b, c, d] = plane_model
print(f"[INFO] Floor plane: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# Step 4: Extract floor and non-floor
floor_cloud = pcd_down.select_by_index(inliers)
non_floor = pcd_down.select_by_index(inliers, invert=True)

# Step 5: Project floor points to 2D using PCA
points = np.asarray(floor_cloud.points)
pca = PCA(n_components=3)
pca.fit(points)
points_pca = pca.transform(points)
xy = points_pca[:, :2]

# Step 6: Bin along x-axis and extract waypoints
x_vals = xy[:, 0]
x_bins = np.linspace(np.min(x_vals), np.max(x_vals), num=30)
waypoints = []
for i in range(len(x_bins) - 1):
    bin_mask = (x_vals >= x_bins[i]) & (x_vals < x_bins[i + 1])
    bin_pts = xy[bin_mask]
    if len(bin_pts) > 5:
        mean_point = np.mean(bin_pts, axis=0)
        waypoints.append(mean_point)
waypoints = np.array(waypoints)

# Step 7: Inverse transform to 3D
waypoints_3D = pca.inverse_transform(np.hstack([waypoints, np.zeros((len(waypoints), 1))]))

# Step 8: Visualize
waypoint_pcd = o3d.geometry.PointCloud()
waypoint_pcd.points = o3d.utility.Vector3dVector(waypoints_3D)
waypoint_pcd.paint_uniform_color([1, 0, 0])  # Red
floor_cloud.paint_uniform_color([0.1, 0.8, 0.1])  # Green
non_floor.paint_uniform_color([0.3, 0.3, 0.3])  # Grey

o3d.visualization.draw_geometries([floor_cloud, non_floor, waypoint_pcd],
                                  window_name="Floor + Waypoints")

# Visualize only floor
o3d.visualization.draw_geometries([floor_cloud],
                                  window_name="Only Floor")

# Save waypoints
np.savetxt("waypoints.csv", waypoints_3D, delimiter=",", header="x,y,z", comments='')
print("[DONE] Waypoints saved to waypoints.csv")



## 🔗 References

* 📘 [ROS 2 Humble Installation (Official Guide)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* 📷 [Intel RealSense D455 Specs](https://www.intelrealsense.com/depth-camera-d455/)
* 🤖 [RTAB-Map ROS 2 Documentation](http://wiki.ros.org/rtabmap_ros)

---

## 📜 License

You are free to use, share, and modify this content with attribution.
