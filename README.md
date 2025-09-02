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
colcon build --packages-select 
scan_fusion map_fusion
```
- Source the workspace:
```
source install/setup.bash
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
ros2 launch scan_fusion scan_fusion.
launch.py
```
- Use a custom config:
```
ros2 launch scan_fusion scan_fusion.
launch.py config_file:=/absolute/
path/to/fusion_params.yaml
```
Tips

- Ensure TF frames exist for both scan sources to target_frame before starting.
- If you see TF timeout or empty fused scan, check frame_ids on the incoming LaserScans and verify transforms.
Package: map_fusion
Overview

- Fuses two occupancy grids (e.g., LiDAR-based and pseudo-laser-based) into a single map using a geometry-based compositing method.
- Handles different map origins and resolutions by resampling and building a fused canvas that covers the union of both maps’ world extents.
Key files

- `map_fusion_node.py`
- `map_fusion.launch.py`
- `map_fusion_params.yaml`
Node

- Name: geometry_map_fusion_node (class ORBMapFusionNode)
- Executable: map_fusion_node
- Subscribed topics:
  - /base_map (nav_msgs/OccupancyGrid, e.g., LiDAR map)
  - /affixion_map (nav_msgs/OccupancyGrid, e.g., depth map)
- Published topic:
  - /fused_map (nav_msgs/OccupancyGrid)
Core parameters

- occ_threshold: Obstacle threshold (>= 65 is obstacle by default)
- binarize_write: If true, writes obstacle cells as 100
- enable_debug_logging: Verbose logs to help with debugging
- boundary_width: Reserved for boundary handling (compatibility)
- Remappable topics via config/launch:
  - base_map_topic (default in config: /lidar_map)
  - affixion_map_topic (default in config: /depth_map)
  - fused_map_topic (default in config: /fused_map)
Fusion rules (high-level)

- Build a fused canvas based on the union of both maps’ world coverage
- Resample the additional map to the base map’s resolution if needed
- Conflict resolution: Occupied (100) > Free (0) > Unknown (-1); base map preferred on conflicts
- Optional boundary protection and noise filtering for isolated points
Launch examples

- Default launch:
```
ros2 launch map_fusion map_fusion.
launch.py
```
- With custom remappings (example):
```
ros2 launch map_fusion map_fusion.
launch.py \
  base_map:=/lidar_map 
  affixion_map:=/depth_map 
  fused_map:=/map
```
- In a larger pipeline (example), the fusion output can be remapped to /map as the global map:
```
ros2 launch map_fusion map_fusion.
launch.py fused_map:=/map
```
End-to-end usage examples

- Scan fusion only (feed SLAM/localization with a single scan):
  
  1. 1.
     Ensure /scan_lidar and /scan_depth are available and TF is correct.
  2. 2.
     Launch scan fusion.
  3. 3.
     Point your SLAM/localization stack to use the fused /scan.
- Map fusion (build both maps separately and fuse):
  
  1. 1.
     Run your LiDAR mapping node publishing /lidar_map.
  2. 2.
     Run your depth-based mapping node publishing /depth_map.
  3. 3.
     Launch map_fusion to publish /fused_map (or remap to /map for downstream Nav2).
Troubleshooting

- Empty fused scan (scan_fusion):
  - Check TF transforms from both scans’ frame_ids to target_frame
  - Increase tf_timeout or slop if data is slightly misaligned
- Empty fused map (map_fusion):
  - Verify both input maps are publishing (correct topic names)
  - Ensure their resolutions/origins are reasonable and not degenerate
  - Keep enable_debug_logging: true to print map sizes and origins
- Topic mismatches:
  - Prefer using the provided launch files and remappings; align to your actual topic names if different
License and citation

- Please add your preferred license to both packages.
- If you use this repository in academic work, please cite your paper accordingly.
Contact

- Maintainer: alex
- Email: 717863696@qq.com
If you want, I can also create and commit this English README.md to your repo for you.
