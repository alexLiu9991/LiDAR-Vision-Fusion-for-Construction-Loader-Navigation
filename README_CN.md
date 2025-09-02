# 移动机器人激光与地图融合（ROS 2）

本仓库用于论文实验复现，包含用于传感器与地图融合的两个 ROS 2 包：
- scan_fusion：将真实激光雷达与伪激光（由深度相机转换）进行时间近似同步、坐标统一并融合输出单一 LaserScan。
- map_fusion：对两幅占据栅格地图（例如 Lidar 建图与伪激光建图）进行几何拼接融合，生成一致的全局融合地图。

注意：由于机器人描述文件涉密（URDF/TF/传感器标定等），本仓库未包含机器人模型与 TF 定义。使用时请根据你的机器人环境提供必要的 TF 变换与传感器数据话题。

---

## 目录结构

- src/scan_fusion
  - 基于 C++ 的激光扫描融合节点
  - 配置：src/scan_fusion/config/fusion_params.yaml
  - 启动：src/scan_fusion/launch/scan_fusion.launch.py
- src/map_fusion
  - 基于 Python 的占据栅格几何拼接融合节点
  - 配置：src/map_fusion/config/map_fusion_params.yaml
  - 启动：src/map_fusion/launch/map_fusion.launch.py

---

## 环境要求

- ROS 2（建议使用与你实验环境一致的版本）
- colcon 构建工具
- 依赖（随包声明自动解析）
  - scan_fusion（C++）: rclcpp, sensor_msgs, geometry_msgs, tf2, tf2_ros, tf2_geometry_msgs
  - map_fusion（Python）: rclpy, nav_msgs, numpy
- 你自己的机器人 TF 与传感器数据源（未随仓库提供）

---

## 构建

在你的 ROS 2 工作空间中将两个包放入 src 目录，执行：

```bash
colcon build --packages-select scan_fusion map_fusion
```

```bash
source install/setup.bash
```

---

## 快速开始

你需要确保以下数据与 TF 可用：
- 激光/伪激光扫描话题：
  - 真实激光：/scan_lidar（可在配置中改名）
  - 伪激光：/scan_depth（由深度图转换得到，可用你自己的转换节点）
- 地图话题（nav_msgs/OccupancyGrid）：
  - /lidar_map 与 /depth_map（在 map_fusion 的 launch 中可重映射）
- TF：传感器坐标系到目标坐标系（默认 base_footprint）的变换在时间上可查询

### 1) 运行 scan_fusion（激光融合）

使用默认配置启动（可修改 config/fusion_params.yaml）：

```bash
ros2 launch scan_fusion scan_fusion.launch.py
```

默认参数要点（可在 config/fusion_params.yaml 修改）：
- 输入话题：
  - scan1_topic: "/scan_lidar"
  - scan2_topic: "/scan_depth"
- 输出话题：
  - output_topic: "/scan"
- 坐标系与角度：
  - target_frame: "base_footprint"
  - grid_angle_min/max/increment：用于输出扫描角度网格（默认 0.25° 分辨率）
- 距离范围：
  - range_min, range_max
- 融合策略：
  - fusion_mode: "min" 或 "weighted"
  - weight_scan1 / weight_scan2：加权融合权重
- 时间近似同步：
  - slop: 0.05（秒）
- TF 超时：
  - tf_timeout: 0.2（秒）

节点行为简介：
- 对 scan1 与 scan2 进行时间近似匹配（slop 容差内择最近对）。
- 使用 TF 查询两者在 target_frame 下的点云坐标，并落入统一极坐标网格。
- 以 “最小值” 或 “加权” 策略融合到单一 LaserScan 输出（output_topic）。

注意：
- 若 TF 不可用（例如缺少传感器帧到 base_footprint 的静态变换），节点会因查询失败而不输出。请在你的系统中提供正确 TF。

### 2) 运行 map_fusion（地图融合）

使用默认配置启动（可修改 config/map_fusion_params.yaml）：

```bash
ros2 launch map_fusion map_fusion.launch.py
```

默认重映射：
- 订阅：/lidar_map → /base_map，/depth_map → /affixion_map
- 发布：/fused_map

关键参数（config/map_fusion_params.yaml）：
- occ_threshold: 65（>=65 视为障碍）
- binarize_write: false（是否把障碍统一为 100）
- enable_debug_logging: true（调试日志）
- base_map_topic、affixion_map_topic、fused_map_topic

算法要点（几何拼接）：
- 若两图分辨率不一致，先将“附加图”重采样到“基础图”分辨率（最近邻，保持占据信息语义）。
- 以两图 origin + resolution 计算世界覆盖范围并取并集，新画布 origin 取全局最小坐标。
- 贴图规则：优先保留基础图；遇到冲突时障碍优先；未知单元（-1）不覆盖；可选择二值化写入。
- 输出 /fused_map 的 frame_id 与基础图一致，origin 设为并集区域的左下角。

---

## 使用建议与复现实例

- 数据回放
  - 可使用 rosbag 播放包含 /scan_lidar 与 /scan_depth（用于 scan_fusion）以及 /lidar_map 与 /depth_map（用于 map_fusion）的数据。
  - 按需先启动节点，再播放数据。

- 深度图转伪激光
  - 本仓库未包含具体深度转 LaserScan 的实现（你可使用自有节点或常用工具包）。
  - 只需确保输出话题与 frame_id 与你的 TF 一致，并与 scan_fusion 的配置匹配。

- TF 提示
  - scan_fusion 会在输出时间查询从两个扫描的 frame_id 到 target_frame 的变换。
  - 请在系统中提供稳定的 TF（例如由机器人描述、驱动或静态 TF 发布器给出）。

---

## 常见问题排查

- scan_fusion 无输出或输出间断
  - 检查 TF 是否可用（传感器帧到 target_frame 的变换）。
  - 检查两路扫描时间戳是否接近（slop 容差内），可调大 slop 验证。
  - 检查 grid_angle_min/max 与 angle_increment 是否合理（覆盖视场且分辨率>0）。

- map_fusion 输出尺寸异常或偏移
  - 检查两幅地图的 origin 与 resolution 是否正确。节点会自动重采样“附加图”到“基础图”的分辨率，并以两图覆盖范围并集生成新画布。
  - 如需改变“障碍判定阈值”或“是否二值化”，修改 occ_threshold 与 binarize_write。

- 运行时日志太多
  - 将 map_fusion 的 enable_debug_logging 设为 false。

---

## 引用与许可

- 若你在论文或项目中使用了本仓库，请在文末添加引用（此处可放置你的论文 BibTeX 或参考格式）。
- License：请根据你的开源策略补充（目前 package.xml 中为 TODO）。

---

## 联系方式

如有问题、Bug 反馈或合作意向，请提交 Issue 或联系维护者：
- Maintainer: alex (717863696@qq.com)
