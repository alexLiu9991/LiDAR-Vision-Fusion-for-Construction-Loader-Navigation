# Loader Sim Map Fusion

地图融合节点，用于融合LRF（激光测距）生成的基础地图和伪激光生成的附加地图。

## 功能特性

- **边界保护**: 融合地图边界（宽度w_c）优先采用基础地图数据
- **状态一致性检查**: 相同状态的单元格直接采用
- **障碍物优先**: 基础地图的障碍物信息优先级更高
- **噪声过滤**: 自动过滤附加地图中的孤立噪声点

## 参数配置

### 核心参数
- `boundary_width`: 边界宽度w_c
- `base_map_topic`: 基础地图话题（默认: `/base_map`）
- `affixion_map_topic`: 附加地图话题（默认: `/affixion_map`）
- `fused_map_topic`: 融合地图输出话题（默认: `/fused_map`）

## 使用方法

### 1. 构建包
```bash
cd /path/to/your/workspace
colcon build --packages-select loader_sim_map_fusion
source install/setup.bash
```

### 2. 运行节点
```bash
# 使用默认参数启动
ros2 launch loader_sim_map_fusion map_fusion.launch.py

# 自定义参数启动
ros2 launch loader_sim_map_fusion map_fusion.launch.py boundary_width:=5 base_map_topic:=/my_base_map
```

### 3. 直接运行节点（不使用launch文件）
```bash
ros2 run loader_sim_map_fusion map_fusion_node
```

## 话题接口

### 订阅话题
- `/base_map` (nav_msgs/OccupancyGrid): 基础地图（LRF生成）
- `/affixion_map` (nav_msgs/OccupancyGrid): 附加地图（伪激光生成）

### 发布话题
- `/fused_map` (nav_msgs/OccupancyGrid): 融合后的地图

## 融合算法

实现了基于 Zhang et al. 2022 论文的四步融合规则：

1. **边界保护**: 距离地图边缘 ≤ w_c 的单元格采用基础地图数据
2. **状态一致**: 两地图状态相同的单元格直接采用
3. **障碍物优先**: 基础地图为障碍物(100)时优先采用基础地图
4. **噪声过滤**: 附加地图的孤立点视为噪声，采用基础地图数据

## 依赖

- ROS2 Foxy
- Python 3.8+
- numpy
- rclpy
- nav_msgs