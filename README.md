# Scan and Map Fusion for ROS 2

This repository contains two ROS 2 packages designed to improve 2D perception quality by fusing multi-source laser scans and occupancy grids:
- scan_fusion: Fuse two LaserScan streams (e.g., 2D LiDAR and depth-to-scan) into a single TF-consistent LaserScan.
- map_fusion: Fuse two OccupancyGrid maps (e.g., LiDAR-built and depth-built maps) into a unified occupancy map.

Typical workflow:
1) Fuse LiDAR and depth-based scans into a single scan topic for SLAM or localization; or
2) Build two maps independently and then fuse them into a single map for planning.

Note: Robot description files are proprietary and are not included in this repository.

## Repository structure

- scan_fusion: C++ ROS 2 node for LaserScan fusion; includes launch and config files.
- map_fusion: Python ROS 2 node for OccupancyGrid fusion; includes launch and config files.

## Requirements

- ROS 2 (rclcpp/rclpy-based)
- A valid TF tree with transforms from each scan’s frame to the target robot frame (for scan_fusion)
- Colcon and a standard ROS 2 development environment

## Build

From your workspace root:
```bash
colcon build --packages-select scan_fusion map_fusion
```

Source the workspace:
```bash
source install/setup.bash
```

---

## Package: scan_fusion

### Overview
Fuses two LaserScan topics into one, with optional weighted blending and TF-based point projection into a target frame. Handles angular grid alignment, range limits, and intensity filling.

### Key files
- launch/scan_fusion.launch.py
- config/fusion_params.yaml
- src/scan_fusion_node.cpp
- src/scan_fusion_ros.cpp

### Node
- Name: LaserScanFusionNode
- Executable: scan_fusion_node

### Topics
- Subscribed:
  - scan1_topic (default in provided config: /scan_lidar)
  - scan2_topic (default in provided config: /scan_depth)
- Published:
  - output_topic (default in provided config: /scan)

All scan points are transformed into target_frame (default: base_footprint) if TF is available.

### Essential parameters (see config/fusion_params.yaml)
- target_frame: Frame to project the fused scan into (default: base_footprint)
- grid_angle_min, grid_angle_max, grid_angle_increment: Output scan angular grid
- range_min, range_max: Range limits for output scan
- fusion_mode: "min" (default) or "weighted"
- weight_scan1, weight_scan2: Used if fusion_mode == "weighted"
- use_inf_for_no_data: If true, sets +inf for no-data beams
- fill_intensities: If true, fills intensities with a default or blended value
- Synchronization:
  - queue_size, slop: Approximate sync buffer and tolerance
- tf_timeout: Timeout while waiting for TF transforms

### Launch examples
Start with default config:
```bash
ros2 launch scan_fusion scan_fusion.launch.py
```

Use a custom config:
```bash
ros2 launch scan_fusion scan_fusion.launch.py config_file:=/absolute/path/to/fusion_params.yaml
```

### Tips
- Ensure TF frames exist for both input scans to target_frame before starting.
- If you see TF timeouts or empty fused scan, check incoming LaserScan frame_ids and verify transforms.
- Align the angular grid and range limits to your downstream SLAM/localization stack.

---

## Package: map_fusion

### Overview
Fuses two occupancy grids (e.g., LiDAR map and depth-based map) into a single map using a geometry-based compositing method. Handles different origins/resolutions by building a fused canvas that covers the union of both maps’ world extents and resampling the additional map if necessary.

### Key files
- map_fusion/map_fusion_node.py
- launch/map_fusion.launch.py
- config/map_fusion_params.yaml

### Node
- Name: geometry_map_fusion_node (class ORBMapFusionNode)
- Executable: map_fusion_node

### Topics
- Subscribed:
  - /base_map (nav_msgs/OccupancyGrid, e.g., LiDAR map)
  - /affixion_map (nav_msgs/OccupancyGrid, e.g., depth map)
- Published:
  - /fused_map (nav_msgs/OccupancyGrid)

Note: Topic names are remapped in launch (e.g., /base_map -> /lidar_map, /affixion_map -> /depth_map). The node itself subscribes/publishes on /base_map, /affixion_map, and /fused_map by default.

### Core parameters
- occ_threshold: Obstacle threshold (>= 65 considered occupied by default)
- binarize_write: If true, writes obstacle cells as 100
- enable_debug_logging: Verbose logs for debugging

Additional options exist in the config file for compatibility (e.g., boundary_width) and debug information.

### Fusion rules (high-level)
- Build a fused canvas based on the union of both maps’ world coverage (using origins and resolution)
- If needed, resample the additional map to the base map’s resolution (nearest-neighbor to preserve semantics)
- Conflict resolution: Occupied (100) > Free (0) > Unknown (-1); base map is preferred in conflicts
- Optional boundary protection and isolated noise filtering

### Launch examples
Default launch:
```bash
ros2 launch map_fusion map_fusion.launch.py
```

With remappings (example):
```bash
ros2 launch map_fusion map_fusion.launch.py \
  base_map:=/lidar_map affixion_map:=/depth_map fused_map:=/map
```

In a larger pipeline, the fusion output can be remapped to /map as the global map:
```bash
ros2 launch map_fusion map_fusion.launch.py fused_map:=/map
```

---

## End-to-end usage examples

### Scan fusion only (single scan for SLAM/localization)
1) Ensure /scan_lidar and /scan_depth are available and TF is correct.
2) Launch scan_fusion.
3) Configure your SLAM/localization to consume the fused /scan topic.

### Map fusion (build two maps and fuse)
1) Run your LiDAR mapping node publishing /lidar_map.
2) Run your depth-based mapping node publishing /depth_map.
3) Launch map_fusion to produce /fused_map (or remap to /map for Nav2).

---

## Troubleshooting

- Empty fused scan (scan_fusion):
  - Check TF transforms from both scans’ frame_ids to target_frame.
  - Increase tf_timeout or slop if data is slightly misaligned.
  - Verify angle limits and range limits are consistent with inputs.

- Empty fused map (map_fusion):
  - Verify both input maps are publishing on the expected topics.
  - Ensure their resolutions and origins are valid (non-degenerate).
  - Keep enable_debug_logging: true to print map sizes/origins in logs.

- Topic mismatches:
  - Prefer the provided launch files and remappings; make sure they align with your actual topic names.

---

## License and citation

- Please add your preferred license to both packages.
- If you use this repository in academic work, please cite your paper accordingly.

## Contact

- Maintainer: alex
- Email: 717863696@qq.com
