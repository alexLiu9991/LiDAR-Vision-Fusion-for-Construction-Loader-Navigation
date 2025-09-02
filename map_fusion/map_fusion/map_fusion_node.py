import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from typing import Optional, Tuple, Dict, Any


class ORBMapFusionNode(Node):
    """
    几何拼接地图融合（不要求两图尺寸一致）。
    - 使用各自的 origin + resolution 计算世界覆盖范围并取并集生成新画布；
    - 若两图 resolution 不一致，先将"附加图"用最近邻重采样到"基础图"的分辨率（保持占据信息语义）；
    - 冲突裁决：障碍(100) > 空闲(0) > 未知(-1)，冲突时保留基础图优先；
    - 订阅 /base_map 与 /affixion_map（可通过 launch 重映射到 /lidar_map 与 /depth_map）。
    """

    # 常量定义
    UNKNOWN_VALUE = -1
    FREE_VALUE = 0
    OCCUPIED_VALUE = 100
    
    # 默认配置参数
    DEFAULT_OCC_THRESHOLD = 65
    DEFAULT_BINARIZE_WRITE = False
    RESOLUTION_TOLERANCE = 1e-9
    COORDINATE_EPSILON = 1e-9
    
    def __init__(self):
        super().__init__('geometry_map_fusion_node')
        
        # 声明参数
        self.declare_parameter('occ_threshold', self.DEFAULT_OCC_THRESHOLD)
        self.declare_parameter('binarize_write', self.DEFAULT_BINARIZE_WRITE)
        self.declare_parameter('enable_debug_logging', True)
        
        # 获取参数
        self._occ_threshold = self.get_parameter('occ_threshold').get_parameter_value().integer_value
        self._binarize_write = self.get_parameter('binarize_write').get_parameter_value().bool_value
        self._debug_logging = self.get_parameter('enable_debug_logging').get_parameter_value().bool_value

        # 订阅基础地图和附加地图（通过launch可重映射）
        self._sub_mb = self.create_subscription(
            OccupancyGrid, '/base_map', self._mb_callback, 10
        )
        self._sub_ma = self.create_subscription(
            OccupancyGrid, '/affixion_map', self._ma_callback, 10
        )

        # 发布融合后的地图
        self._pub_fused_map = self.create_publisher(OccupancyGrid, '/fused_map', 10)

        # 缓存
        self._mb: Optional[OccupancyGrid] = None
        self._ma: Optional[OccupancyGrid] = None

        self.get_logger().info('Geometry-based Map Fusion Node initialized')

    def _mb_callback(self, msg: OccupancyGrid) -> None:
        """基础地图回调函数"""
        self._mb = msg
        self._try_fusion()

    def _ma_callback(self, msg: OccupancyGrid) -> None:
        """附加地图回调函数"""
        self._ma = msg
        self._try_fusion()

    def _try_fusion(self) -> None:
        """尝试进行地图融合"""
        if self._mb is None or self._ma is None:
            return

        try:
            if self._debug_logging:
                self._log_map_info()
            
            fused = self._geometry_based_fusion(self._mb, self._ma)
            if fused is not None:
                fused.header.stamp = self.get_clock().now().to_msg()
                self._pub_fused_map.publish(fused)
                
                if self._debug_logging:
                    self.get_logger().info(
                        f'Fused map published: size={fused.info.width}x{fused.info.height}, '
                        f'res={fused.info.resolution:.3f}'
                    )
        except Exception as e:
            self.get_logger().error(f'Error during map fusion: {str(e)}')

    def _log_map_info(self) -> None:
        """记录地图信息用于调试"""
        # 基础地图信息
        self.get_logger().info(
            f"Base map: origin=({self._mb.info.origin.position.x:.2f}, "
            f"{self._mb.info.origin.position.y:.2f}), "
            f"size={self._mb.info.width}x{self._mb.info.height}, "
            f"res={self._mb.info.resolution:.3f}"
        )
        
        # 附加地图信息
        self.get_logger().info(
            f"Add map: origin=({self._ma.info.origin.position.x:.2f}, "
            f"{self._ma.info.origin.position.y:.2f}), "
            f"size={self._ma.info.width}x{self._ma.info.height}, "
            f"res={self._ma.info.resolution:.3f}"
        )
        
        # 附加地图数据分析
        self._analyze_additional_map_data()
        
        # 世界覆盖范围
        self._log_world_coverage()

    def _analyze_additional_map_data(self) -> None:
        """分析附加地图的数据分布"""
        ma_data = np.array(self._ma.data, dtype=np.int16)
        
        # 统计标准值
        occupied_count = np.sum(ma_data == self.OCCUPIED_VALUE)
        free_count = np.sum(ma_data == self.FREE_VALUE)
        unknown_count = np.sum(ma_data == self.UNKNOWN_VALUE)
        other_count = len(ma_data) - occupied_count - free_count - unknown_count
        
        # 获取唯一值统计
        unique_vals, counts = np.unique(ma_data, return_counts=True)
        val_stats = dict(zip(unique_vals.tolist(), counts.tolist()))
        
        # 计算有效值范围
        valid_vals = ma_data[ma_data > self.UNKNOWN_VALUE]
        if len(valid_vals) > 0:
            max_obstacle = int(np.max(valid_vals))
            min_obstacle = int(np.min(valid_vals))
        else:
            max_obstacle = self.UNKNOWN_VALUE
            min_obstacle = self.UNKNOWN_VALUE
        
        # 记录统计信息
        self.get_logger().info(
            f"Add map stats: total={len(ma_data)}, occupied={occupied_count}, "
            f"free={free_count}, unknown={unknown_count}, other={other_count}"
        )
        self.get_logger().info(f"Add map unique values: {val_stats}")
        self.get_logger().info(f"Add map obstacle range: min={min_obstacle}, max={max_obstacle}")

    def _log_world_coverage(self) -> None:
        """记录世界覆盖范围信息"""
        w_b = self._mb.info.width * self._mb.info.resolution
        h_b = self._mb.info.height * self._mb.info.resolution
        w_a = self._ma.info.width * self._ma.info.resolution
        h_a = self._ma.info.height * self._ma.info.resolution
        
        self.get_logger().info(
            f"World coverage - Base: {w_b:.1f}x{h_b:.1f}m, Add: {w_a:.1f}x{h_a:.1f}m"
        )

    def _resample_occupancy_grid_nn(self, og: OccupancyGrid, target_res: float) -> OccupancyGrid:
        """
        将占据栅格用最近邻重采样到目标分辨率（保持语义：障碍/空闲/未知）。
        - origin 与坐标系不变，仅修改 width/height/resolution 与 data。
        """
        src_res = float(og.info.resolution)
        
        # 验证输入参数
        if target_res <= 0.0 or src_res <= 0.0:
            return og
        if abs(src_res - target_res) <= 1e-12:
            return og  # 无需重采样

        src_w, src_h = int(og.info.width), int(og.info.height)
        
        # 计算新尺寸
        world_w = src_w * src_res
        world_h = src_h * src_res
        dst_w = max(1, int(np.round(world_w / target_res)))
        dst_h = max(1, int(np.round(world_h / target_res)))

        # 准备数据
        src = np.array(og.data, dtype=np.int16).reshape(src_h, src_w)
        dst = np.full((dst_h, dst_w), self.UNKNOWN_VALUE, dtype=np.int16)

        # 最近邻重采样
        self._perform_nearest_neighbor_resampling(src, dst, src_res, target_res)

        # 构建新的OccupancyGrid
        return self._create_resampled_occupancy_grid(og, dst, target_res, dst_w, dst_h)

    def _perform_nearest_neighbor_resampling(
        self, 
        src: np.ndarray, 
        dst: np.ndarray, 
        src_res: float, 
        target_res: float
    ) -> None:
        """执行最近邻重采样"""
        dst_h, dst_w = dst.shape
        src_h, src_w = src.shape
        
        for iy in range(dst_h):
            y = (iy + 0.5) * target_res
            src_iy = int(np.floor(y / src_res))
            src_iy = np.clip(src_iy, 0, src_h - 1)
            
            src_row = src[src_iy]
            dst_row = dst[iy]
            
            for ix in range(dst_w):
                x = (ix + 0.5) * target_res
                src_ix = int(np.floor(x / src_res))
                src_ix = np.clip(src_ix, 0, src_w - 1)
                dst_row[ix] = src_row[src_ix]

    def _create_resampled_occupancy_grid(
        self, 
        original: OccupancyGrid, 
        data: np.ndarray, 
        resolution: float, 
        width: int, 
        height: int
    ) -> OccupancyGrid:
        """创建重采样后的OccupancyGrid"""
        new_og = OccupancyGrid()
        new_og.header = original.header
        new_og.info = original.info
        new_og.info.resolution = float(resolution)
        new_og.info.width = int(width)
        new_og.info.height = int(height)
        new_og.data = np.clip(data, self.UNKNOWN_VALUE, self.OCCUPIED_VALUE).astype(np.int8).flatten().tolist()
        return new_og

    def _is_occupied(self, value: int) -> bool:
        """判断给定值是否应被视为障碍"""
        return (value == self.OCCUPIED_VALUE or 
                (1 <= value <= 99 and value >= self._occ_threshold))

    def _calculate_world_bounds(self, mb: OccupancyGrid, ma: OccupancyGrid, res: float) -> Tuple[float, float, float, float, int, int]:
        """计算世界边界和融合画布尺寸"""
        # 基础地图边界
        ox_b, oy_b = float(mb.info.origin.position.x), float(mb.info.origin.position.y)
        w_b, h_b = int(mb.info.width), int(mb.info.height)
        maxx_b, maxy_b = ox_b + w_b * res, oy_b + h_b * res

        # 附加地图边界
        ox_a, oy_a = float(ma.info.origin.position.x), float(ma.info.origin.position.y)
        w_a, h_a = int(ma.info.width), int(ma.info.height)
        maxx_a, maxy_a = ox_a + w_a * res, oy_a + h_a * res

        # 计算并集边界
        min_x = min(ox_b, ox_a)
        min_y = min(oy_b, oy_a)
        max_x = max(maxx_b, maxx_a)
        max_y = max(maxy_b, maxy_a)

        # 计算融合画布尺寸
        fused_w = int(np.ceil((max_x - min_x) / res))
        fused_h = int(np.ceil((max_y - min_y) / res))
        
        return min_x, min_y, max_x, max_y, fused_w, fused_h

    def _paste_map(
        self, 
        fused: np.ndarray, 
        src_data: np.ndarray, 
        ox: float, 
        oy: float, 
        min_x: float, 
        min_y: float, 
        res: float, 
        prefer_existing: bool
    ) -> None:
        """将地图数据贴到融合画布上"""
        sh, sw = src_data.shape
        fused_h, fused_w = fused.shape
        
        # 计算偏移量
        offset_x = (ox - min_x) / res
        offset_y = (oy - min_y) / res

        for iy in range(sh):
            ty = int(np.floor(iy + offset_y + self.COORDINATE_EPSILON))
            if ty < 0 or ty >= fused_h:
                continue
                
            row_src = src_data[iy]
            row_dst = fused[ty]
            
            for ix in range(sw):
                tx = int(np.floor(ix + offset_x + self.COORDINATE_EPSILON))
                if tx < 0 or tx >= fused_w:
                    continue
                    
                val = row_src[ix]
                
                # 跳过非法值和未知值
                if val < self.UNKNOWN_VALUE or val > self.OCCUPIED_VALUE or val == self.UNKNOWN_VALUE:
                    continue

                cur = row_dst[tx]
                
                # 处理空画布位置
                if cur == self.UNKNOWN_VALUE:
                    row_dst[tx] = (self.OCCUPIED_VALUE if (self._binarize_write and self._is_occupied(val)) 
                                 else val)
                    continue

                # 处理已有值的情况
                if prefer_existing:
                    # 基础图优先，但允许障碍升级
                    if self._is_occupied(val) and not self._is_occupied(cur):
                        row_dst[tx] = self.OCCUPIED_VALUE if self._binarize_write else val
                else:
                    # 冲突裁决逻辑
                    self._resolve_conflict(row_dst, tx, cur, val)

    def _resolve_conflict(self, row_dst: np.ndarray, tx: int, cur: int, val: int) -> None:
        """解决像素值冲突"""
        if self._is_occupied(cur) or self._is_occupied(val):
            # 有一方是障碍：选择障碍值
            if self._binarize_write:
                row_dst[tx] = self.OCCUPIED_VALUE
            else:
                row_dst[tx] = val if self._is_occupied(val) else cur
        elif cur == self.FREE_VALUE and val == self.FREE_VALUE:
            row_dst[tx] = self.FREE_VALUE
        else:
            row_dst[tx] = cur  # 保留已有值（基础优先）

    def _create_fused_occupancy_grid(
        self, 
        mb: OccupancyGrid, 
        fused: np.ndarray, 
        res: float, 
        min_x: float, 
        min_y: float
    ) -> OccupancyGrid:
        """创建融合后的OccupancyGrid消息"""
        fused_h, fused_w = fused.shape
        
        fused_map = OccupancyGrid()
        fused_map.header = mb.header
        fused_map.header.stamp = self.get_clock().now().to_msg()
        
        # 设置地图信息
        fused_map.info.resolution = res
        fused_map.info.width = fused_w
        fused_map.info.height = fused_h
        fused_map.info.origin.position.x = float(min_x)
        fused_map.info.origin.position.y = float(min_y)
        fused_map.info.origin.position.z = 0.0
        fused_map.info.origin.orientation.x = 0.0
        fused_map.info.origin.orientation.y = 0.0
        fused_map.info.origin.orientation.z = 0.0
        fused_map.info.origin.orientation.w = 1.0

        fused_map.data = np.clip(fused, self.UNKNOWN_VALUE, self.OCCUPIED_VALUE).astype(np.int8).flatten().tolist()
        return fused_map

    def _geometry_based_fusion(self, mb: OccupancyGrid, ma: OccupancyGrid) -> Optional[OccupancyGrid]:
        """
        使用几何信息（origin + resolution）进行拼接融合：
        - 不要求两图尺寸一致；
        - 若分辨率不一致，先将"附加图"重采样到"基础图"的分辨率（最近邻）；
        - 新画布的origin取两幅地图覆盖区域的全局最小坐标，尺寸取并集范围；
        - 融合规则：障碍(100) > 空闲(0) > 未知(-1)，冲突时优先基础图。
        """
        try:
            # 1) 验证分辨率并进行重采样
            res_b = float(mb.info.resolution)
            res_a = float(ma.info.resolution)
            
            if res_b <= 0.0 or res_a <= 0.0:
                self.get_logger().warn('[geometry_based_fusion] Invalid resolution, return base map.')
                return mb
                
            if abs(res_b - res_a) > self.RESOLUTION_TOLERANCE:
                if self._debug_logging:
                    self.get_logger().warn(
                        f"[geometry_based_fusion] Resolution mismatch: base={res_b}, add={res_a}. "
                        "Resample additional to base."
                    )
                ma = self._resample_occupancy_grid_nn(ma, res_b)

            res = float(mb.info.resolution)

            # 2) 计算世界边界和融合画布尺寸
            min_x, min_y, max_x, max_y, fused_w, fused_h = self._calculate_world_bounds(mb, ma, res)
            
            if fused_w <= 0 or fused_h <= 0:
                self.get_logger().warn(
                    "[geometry_based_fusion] Computed fused canvas has non-positive size, return base map."
                )
                return mb

            # 3) 初始化融合画布
            fused = np.full((fused_h, fused_w), self.UNKNOWN_VALUE, dtype=np.int16)

            # 4) 准备数据
            mb_data = np.array(mb.data, dtype=np.int16).reshape(mb.info.height, mb.info.width)
            ma_data = np.array(ma.data, dtype=np.int16).reshape(ma.info.height, ma.info.width)

            # 5) 执行地图融合
            ox_b, oy_b = float(mb.info.origin.position.x), float(mb.info.origin.position.y)
            ox_a, oy_a = float(ma.info.origin.position.x), float(ma.info.origin.position.y)
            
            # 先贴基础图，再贴附加图
            self._paste_map(fused, mb_data, ox_b, oy_b, min_x, min_y, res, prefer_existing=True)
            self._paste_map(fused, ma_data, ox_a, oy_a, min_x, min_y, res, prefer_existing=False)

            # 6) 生成最终的OccupancyGrid
            return self._create_fused_occupancy_grid(mb, fused, res, min_x, min_y)
            
        except Exception as e:
            self.get_logger().error(f'Error in geometry_based_fusion: {str(e)}')
            return mb  # 返回基础地图作为fallback


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ORBMapFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()