# Scan and Map Fusion for ROS 2
This repository contains two ROS 2 packages designed to improve 2D perception quality by fusing multi-source laser scans and occupancy grids:

- scan_fusion: Fuse two LaserScan streams (e.g., 2D LiDAR and depth-to-scan) into a single, TF-consistent LaserScan.
- map_fusion: Fuse two OccupancyGrid maps (e.g., LiDAR-built and depth-built maps) into a unified occupancy map.
Typical workflow:

1. 1.
   Fuse LiDAR and depth-based scans into a single scan topic for SLAM or localization; or
2. 2.
   Build two maps independently and then fuse them into a single map for planning.
Repository note: The robot description files are proprietary and not included.

Repository structure

- scan_fusion: C++ ROS 2 node for LaserScan fusion; launch and config included.
- map_fusion: Python ROS 2 node for OccupancyGrid fusion; launch and config included.
Requirements

- ROS 2 (use your distribution; both packages are standard rclcpp/rclpy-based)
- TF tree with valid transforms between each sensor frame and the target robot frame (for scan_fusion)
- Colcon for building, and typical ROS 2 dev environment setup
Build

- From your workspace root:
```
colcon build --packages-select 
scan_fusion map_fusion
```
- Source the workspace:
```
source install/setup.bash
```
Package: scan_fusion
Overview

- Fuses two LaserScan topics into one, with optional weighted blending and TF-based point projection into a target frame.
- Handles angle/time grid alignment, range limits, and intensity filling.
Key files

- `scan_fusion.launch.py`
- `fusion_params.yaml`
- `scan_fusion_node.cpp`
- `scan_fusion_ros.cpp`
Node

- Name: LaserScanFusionNode
- Executable: scan_fusion_node
- Input topics:
  - scan1_topic (default: /scan_lidar)
  - scan2_topic (default: /scan_depth)
- Output topic:
  - output_topic (default: /scan)
- TF:
  - All scan points are transformed into target_frame (default: base_footprint) if TF is available.
Essential parameters (see fusion_params.yaml)

- target_frame: Frame to project the fused scan into (default: base_footprint)
- grid_angle_min, grid_angle_max, grid_angle_increment: Output scan angular grid
- range_min, range_max: Range limits for output scan
- fusion_mode: "min" (default) or "weighted"
- weight_scan1, weight_scan2: Used if fusion_mode == "weighted"
- use_inf_for_no_data: If true, sets +inf for no-data beams instead of range_max
- fill_intensities: If true, fills intensities with a default value
- Synchronization:
  - queue_size, slop: Controls approximate synchronization tolerance
- tf_timeout: Timeout waiting for TF transforms
Launch examples

- Start with default config:
```
ros2 launch scan_fusion scan_fusion.
launch.py
```
- Use a custom config:
```
ros2 launch scan_fusion scan_fusion.
launch.py config_file:=/absolute/
path/to/fusion_params.yaml
```
Tips

- Ensure TF frames exist for both scan sources to target_frame before starting.
- If you see TF timeout or empty fused scan, check frame_ids on the incoming LaserScans and verify transforms.
Package: map_fusion
