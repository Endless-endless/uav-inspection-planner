"""
电网巡检路径规划系统 V3.0

功能：
- 电网提取与路径规划
- 地形感知路径优化
- 3D 可视化与交互导航
- 巡检任务生成
"""

import numpy as np
from PIL import Image, ImageFilter, ImageDraw
from collections import defaultdict, deque
import os

# 外部依赖容错处理
try:
    from skimage.morphology import skeletonize, dilation, erosion
except ImportError:
    print("[WARN] skimage.morphology 未安装，骨架化和膨胀功能将降级")
    skeletonize = None
    dilation = None
    erosion = None

try:
    from skimage.color import rgb2hsv
except ImportError:
    print("[WARN] skimage.color 未安装，HSV转换功能将降级")
    rgb2hsv = None

try:
    from skimage.transform import resize
except ImportError:
    print("[WARN] skimage.transform 未安装，图像缩放功能将降级")
    resize = None

try:
    from scipy.ndimage import gaussian_filter
except ImportError:
    print("[WARN] scipy.ndimage 未安装，高斯滤波功能将降级")
    gaussian_filter = None

try:
    from scipy.interpolate import splprep, splev
except ImportError:
    print("[WARN] scipy.interpolate 未安装，样条插值功能将降级")
    splprep = None
    splev = None


class PowerlinePlannerV3:
    """
    电网巡检路径规划器 V3.0 - 答辩级优化版

    核心增强：
    - 路径平滑（滑动平均/Spline）
    - 巡检语义（电塔节点）
    - 飞行波动模拟
    - 专业级可视化
    """

    def __init__(self, image_path, flight_height=30, smooth_window=5,
                 use_spline=False, add_flight_fluctuation=True,
                 wind_direction=0, wind_speed=5,
                 weather_scene="calm", weather_profile=None):
        """
        初始化规划器

        Args:
            image_path: 电网图片路径
            flight_height: 飞行高度（米）
            smooth_window: 滑动平均窗口大小
            use_spline: 是否使用样条平滑
            add_flight_fluctuation: 是否添加飞行波动
            wind_direction: 风向角度（0-360°，0=东风）
            wind_speed: 风速（m/s）
            weather_scene: 天气场景名称（calm, crosswind, headwind_strong, tailwind_efficient, gusty_high_risk）
            weather_profile: 天气配置字典（优先使用，如果提供则忽略 weather_scene）
        """
        self.image_path = image_path
        self.flight_height = flight_height
        self.smooth_window = smooth_window
        self.use_spline = use_spline
        self.add_flight_fluctuation = add_flight_fluctuation
        self.wind_direction = wind_direction
        self.wind_speed = wind_speed

        # 天气相关
        self.weather_scene = weather_scene
        if weather_profile is not None:
            self.weather_profile = weather_profile
        else:
            # 根据 weather_scene 自动获取 profile
            from weather.wind_model import get_weather_profile
            self.weather_profile = get_weather_profile(weather_scene)

        # 记录天气信息到 mission_wind
        self.mission_wind = {
            "scene": self.weather_profile.get("scene", weather_scene),
            "label": self.weather_profile.get("label", "未知"),
            "wind_speed": self.weather_profile.get("wind_speed", wind_speed),
            "wind_direction": self.weather_profile.get("wind_direction", wind_direction),
            "gust_factor": self.weather_profile.get("gust_factor", 1.0),
            "risk_level": self.weather_profile.get("risk_level", "unknown"),
            "energy_factor": self.weather_profile.get("energy_factor", 1.0),
            "description": self.weather_profile.get("description", "")
        }

        # 数据容器
        self.image = None
        self.img_array = None
        self.height_map = None
        self.height_map_smooth = None
        self.mask = None
        self.skeleton = None
        self.polyline = []
        self.waypoints = []
        self.waypoints_with_cost = []  # 带风成本的航点
        self.path_3d = []
        self.path_3d_smooth = []  # 平滑后的路径
        self.tower_points = []     # 电塔节点

        # 统计信息
        self.stats = {}

        # 图像尺寸
        self.width = 0
        self.height = 0

        # =====================================================
        # 阶段1新增：多独立线路支持（增量兼容）
        # =====================================================
        # 多独立线路数据
        self.independent_lines = []  # List[IndependentLine]
        self.line_inspection_points = []  # List[LineInspectionPoint]
        self.line_inspection_points_by_line = {}  # Dict[line_id, List[LineInspectionPoint]]
        self.primary_line_id = None  # 主线路ID（最长线路）

        # =====================================================
        # 阶段2新增：任务优化层（增量兼容）
        # =====================================================
        # 任务层数据
        self.tasks = []  # List[TaskLine]
        self.cost_mat = None  # np.ndarray, 任务间代价矩阵
        self.line_ord = []  # List[int], 优化后的任务顺序（索引）
        self.line_dir = {}  # Dict[str, int], 任务方向映射
        self.g_path_2d = []  # List[Tuple], 全局2D路径
        self.g_path_3d = []  # List[Tuple], 全局3D路径
        self.g_stats = {}  # Dict, 全局统计
        # self.mission_wind 已在天气初始化部分设置，不要覆盖

        # =====================================================
        # 阶段2可视化适配层（增量）
        # =====================================================
        self.vis_pts = []      # List[VisPoint]
        self.vis_tasks = []    # List[VisTask]
        self.vis_stats = {}    # Dict
        self.anim_path_3d = [] # List[Tuple]

        # =====================================================
        # 拓扑层（新增 - Phase 1）
        # =====================================================
        self.topo_nodes = []        # List[TopoNode], 拓扑节点
        self.topo_edges = []        # List[TopoEdge], 拓扑边
        self.topo_graph = None      # TopoGraph, 拓扑图
        self.edge_tasks = []        # List[EdgeTask], 边任务
        self.edge_ord = []          # List[str], 优化后的边任务顺序
        self.edge_dir = {}          # Dict[str, int], 边任务方向映射
        self.topo_path_3d = []      # List[Tuple], 拓扑规划3D路径
        self.topo_stats = {}        # Dict, 拓扑规划统计

    # =====================================================
    # 基础功能（继承自V2）
    # =====================================================

    def step1_extract_redline_hsv(self):
        """STEP 1: 使用HSV提取红色线路"""
        print("[STEP 1] 使用HSV提取红色线路...")

        self.image = Image.open(self.image_path).convert("RGB")
        self.width, self.height = self.image.size
        print(f"  图片尺寸: {self.width}x{self.height}")

        self.img_array = np.array(self.image).astype(np.float32) / 255.0

        # HSV转换（降级处理）
        if rgb2hsv is not None:
            try:
                hsv = rgb2hsv(self.img_array)
                mask1 = (hsv[:, :, 0] <= 0.028) & (hsv[:, :, 1] > 0.3) & (hsv[:, :, 2] > 0.2)
                mask2 = (hsv[:, :, 0] >= 0.94) & (hsv[:, :, 1] > 0.3) & (hsv[:, :, 2] > 0.2)
                self.mask = (mask1 | mask2).astype(np.uint8) * 255
            except Exception as e:
                print(f"[WARN] HSV转换失败: {e}，使用简单RGB阈值")
                self.mask = self._simple_red_threshold()
        else:
            print("[WARN] skimage.color 未安装，使用简单RGB阈值")
            self.mask = self._simple_red_threshold()

        pixel_count = np.sum(self.mask > 0)
        print(f"  提取红色像素: {pixel_count} 个")

        os.makedirs("result", exist_ok=True)
        mask_img = Image.fromarray(self.mask)
        mask_img.save("result/step1_hsv_mask.png")

        return self.mask

    def _simple_red_threshold(self):
        """简单RGB红色阈值提取（降级方案）"""
        # 简单的红色通道检测
        img = np.array(self.image)
        red_mask = (img[:, :, 0] > 150) & (img[:, :, 1] < 100) & (img[:, :, 2] < 100)
        return red_mask.astype(np.uint8) * 255

    def step2_fix_breaks(self):
        """STEP 2: 修复断裂（膨胀+腐蚀操作）"""
        print("[STEP 2] 修复断裂（膨胀+腐蚀操作）...")

        if self.mask is None:
            self.step1_extract_redline_hsv()

        # 膨胀连接断裂（降级处理）
        if dilation is not None:
            try:
                self.mask = dilation(self.mask > 0, footprint=np.ones((3, 3), dtype=bool)).astype(np.uint8) * 255
                self.mask = dilation(self.mask > 0, footprint=np.ones((3, 3), dtype=bool)).astype(np.uint8) * 255
            except Exception as e:
                print(f"[WARN] 膨胀操作失败: {e}，跳过断裂修复")
        else:
            print("[WARN] skimage.morphology 未安装，跳过膨胀操作")

        # 轻微腐蚀恢复宽度（降级处理）
        if erosion is not None:
            try:
                self.mask = erosion(self.mask > 0, footprint=np.ones((2, 2), dtype=bool)).astype(np.uint8) * 255
            except Exception as e:
                print(f"[WARN] 腐蚀操作失败: {e}，保持膨胀后状态")
        else:
            print("[WARN] erosion 未安装，跳过腐蚀操作")

        pixel_count = np.sum(self.mask > 0)
        print(f"  修复后像素: {pixel_count} 个")

        mask_img = Image.fromarray(self.mask)
        mask_img.save("result/step2_fixed_mask.png")

        return self.mask

    def step3_skeletonize(self):
        """STEP 3: 骨架化（提取中心线）"""
        print("[STEP 3] 骨架化（提取中心线）...")

        if self.mask is None:
            self.step2_fix_breaks()

        # 检查依赖是否可用
        if erosion is None or skeletonize is None:
            print("[WARN] skimage 未安装，跳过骨架化步骤，使用原始mask")
            self.skeleton = self.mask.astype(np.uint8)
            return self.skeleton

        # 收缩mask（去除毛边，减少噪点）
        try:
            binary = erosion(self.mask > 0, footprint=np.ones((2, 2), dtype=bool))
            # 骨架化
            self.skeleton = skeletonize(binary).astype(np.uint8) * 255
        except Exception as e:
            print(f"[WARN] 骨架化失败: {e}，使用原始mask")
            self.skeleton = self.mask.astype(np.uint8)

        skeleton_pixels = np.sum(self.skeleton > 0)
        print(f"  骨架像素: {skeleton_pixels} 个")

        skel_img = Image.fromarray(self.skeleton)
        skel_img.save("result/step3_skeleton.png")

        return self.skeleton

    def step4_extract_continuous_path(self):
        """STEP 4: 提取连续路径（核心算法）"""
        print("[STEP 4] 提取连续路径...")

        if self.skeleton is None:
            self.step3_skeletonize()

        skeleton_points = np.argwhere(self.skeleton > 0)

        if len(skeleton_points) < 2:
            raise ValueError("骨架点太少，无法生成路径")

        neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        adj = defaultdict(list)

        for row, col in skeleton_points:
            y, x = row, col
            for dy, dx in neighbors:
                ny, nx = y + dy, x + dx
                if 0 <= ny < self.skeleton.shape[0] and 0 <= nx < self.skeleton.shape[1]:
                    if self.skeleton[ny, nx] > 0:
                        adj[(x, y)].append((nx, ny))

        degrees = {point: len(neighs) for point, neighs in adj.items()}
        endpoints = [point for point, deg in degrees.items() if deg == 1]
        print(f"  检测到 {len(endpoints)} 个端点")

        if len(endpoints) == 0:
            endpoints = [list(adj.keys())[0]]

        visited = set()
        all_polylines = []

        for endpoint in endpoints:
            if endpoint in visited:
                continue

            queue = deque([endpoint])
            path = []

            while queue:
                point = queue.popleft()
                if point in visited:
                    continue

                visited.add(point)
                path.append(point)

                for neighbor in sorted(adj.get(point, [])):
                    if neighbor not in visited:
                        queue.append(neighbor)

            if len(path) > 1:
                all_polylines.append(path)

        print(f"  生成 {len(all_polylines)} 条路径")

        if all_polylines:
            self.polyline = max(all_polylines, key=len)
            print(f"  [选择] 主路径长度: {len(self.polyline)} 个点")

            # 检查polyline长度，如果太短则不使用
            if len(self.polyline) < 20:
                print(f"  [WARN] 路径过短 ({len(self.polyline)} < 20)，将跳过电网显示")
                self.polyline = []
        else:
            print(f"  [WARN] 未能生成有效路径，将跳过电网显示")
            self.polyline = []

        if self.polyline and len(self.polyline) > 0:
            self._save_pathVisualization()

        return self.polyline

    def _save_pathVisualization(self):
        """保存路径可视化"""
        vis = self.image.copy()
        draw = ImageDraw.Draw(vis)

        for x, y in self.polyline:
            draw.point((x, y), fill=(0, 255, 0))

        if len(self.polyline) > 0:
            x0, y0 = self.polyline[0]
            x1, y1 = self.polyline[-1]
            draw.ellipse([x0-5, y0-5, x0+5, y0+5], fill=(255, 0, 0))
            draw.ellipse([x1-5, y1-5, x1+5, y1+5], fill=(255, 0, 0))

        vis.save("result/step4_polyline.png")
        print(f"  [保存] result/step4_polyline.png")

    def step5_sample_waypoints(self, step=5):
        """STEP 5: 路径采样（生成巡检航点）"""
        print(f"[STEP 5] 路径采样（间隔={step}像素）...")

        if not self.polyline:
            self.step4_extract_continuous_path()

        self.waypoints = self.polyline[::step]

        if len(self.polyline) > 0 and (len(self.waypoints) == 0 or self.waypoints[-1] != self.polyline[-1]):
            self.waypoints.append(self.polyline[-1])

        print(f"  生成 {len(self.waypoints)} 个航点")

        return self.waypoints

    def step6_smooth_terrain(self, terrain_raw, gaussian_sigma=3.0, enhance_resolution=False):
        """
        STEP 6: 地形平滑（增强版）

        Args:
            terrain_raw: 原始地形
            gaussian_sigma: 高斯模糊sigma值
            enhance_resolution: 是否提高分辨率
        """
        print("[STEP 6] 地形平滑（增强版）...")

        # 高斯模糊（降级处理）
        if gaussian_filter is not None:
            try:
                self.height_map_smooth = gaussian_filter(terrain_raw.astype(np.float32), sigma=gaussian_sigma)
            except Exception as e:
                print(f"[WARN] 高斯滤波失败: {e}，使用原始地形")
                self.height_map_smooth = terrain_raw.astype(np.float32)
        else:
            print("[WARN] scipy.ndimage 未安装，使用原始地形（无平滑）")
            self.height_map_smooth = terrain_raw.astype(np.float32)

        # 可选：提高分辨率
        if enhance_resolution and resize is not None:
            try:
                h, w = self.height_map_smooth.shape
                self.height_map_smooth = resize(self.height_map_smooth, (h*2, w*2),
                                               preserve_range=True, order=1)
                print(f"  分辨率提升: {w}x{h} → {w*2}x{h*2}")
            except Exception as e:
                print(f"[WARN] 分辨率提升失败: {e}")
        elif enhance_resolution and resize is None:
            print("[WARN] skimage.transform 未安装，跳过分辨率提升")

        print(f"  平滑前范围: [{terrain_raw.min():.1f}, {terrain_raw.max():.1f}]")
        print(f"  平滑后范围: [{self.height_map_smooth.min():.1f}, {self.height_map_smooth.max():.1f}]")

        # 保存原始地形用于其他用途
        self.height_map = terrain_raw

        # 创建 terrain_3d 数组 (height, width, 3) 用于 UI 可视化
        # 每个像素包含 [x, y, z] 坐标
        h, w = self.height_map_smooth.shape
        self.terrain_3d = np.zeros((h, w, 3), dtype=np.float32)
        # 创建坐标网格
        y_coords, x_coords = np.mgrid[0:h, 0:w]
        self.terrain_3d[:, :, 0] = x_coords.astype(np.float32)  # x 坐标
        self.terrain_3d[:, :, 1] = y_coords.astype(np.float32)  # y 坐标
        self.terrain_3d[:, :, 2] = self.height_map_smooth       # z 坐标（高度）

        return self.height_map_smooth

    def apply_wind_cost(self, alpha=0.05):
        """
        计算天气影响（风）对航线的成本

        Args:
            alpha: 风速影响系数（默认0.05）

        Returns:
            list: [(x, y, cost), ...] 带风成本的航点列表
        """
        if not self.waypoints:
            return []

        print(f"[天气] 计算风影响成本（风向={self.wind_direction}°, 风速={self.wind_speed}m/s）...")

        waypoints_with_cost = []
        total_cost = 0

        for i in range(len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]

            # 计算路径方向（角度）
            path_angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            if path_angle < 0:
                path_angle += 360

            # 计算风向与路径方向的夹角（有向）
            angle_diff_signed = path_angle - self.wind_direction
            angle_diff_signed = (angle_diff_signed + 180) % 360 - 180

            # 转换为弧度
            angle_rad = np.radians(angle_diff_signed)
            cos_theta = np.cos(angle_rad)

            # 强化风惩罚机制：逆风强惩罚，顺风奖励
            if cos_theta < 0:
                # 逆风：强惩罚（基于风速，但保持正值）
                segment_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
            else:
                # 顺风：奖励（最小不低于0.5）
                segment_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)

            waypoints_with_cost.append((x1, y1, segment_cost))
            total_cost += segment_cost

        # 添加最后一个点
        if len(self.waypoints) > 0:
            waypoints_with_cost.append((self.waypoints[-1][0], self.waypoints[-1][1], 1.0))

        self.waypoints_with_cost = waypoints_with_cost

        avg_cost = total_cost / len(waypoints_with_cost) if waypoints_with_cost else 0
        print(f"  平均风成本: {avg_cost:.3f}")
        print(f"  总风成本: {total_cost:.3f}")

        return waypoints_with_cost

    def apply_weather_profile_to_cost(self, path_points, alpha=0.1):
        """
        应用天气 profile 到路径代价计算（增强版）

        Args:
            path_points: 路径点列表 [(x, y), ...]
            alpha: 天气影响系数（默认0.1）

        Returns:
            list: [(x, y, enhanced_cost), ...] 带天气增强代价的航点列表
        """
        if not path_points:
            return []

        print(f"[天气] 应用天气 profile: {self.weather_profile.get('label', '未知')}")
        print(f"  场景: {self.weather_profile.get('scene', 'unknown')}")
        print(f"  风速: {self.weather_profile.get('wind_speed', 0):.1f} m/s")
        print(f"  风向: {self.weather_profile.get('wind_direction', 0):.1f}°")
        print(f"  阵风因子: {self.weather_profile.get('gust_factor', 1.0):.2f}")
        print(f"风险等级: {self.weather_profile.get('risk_level', 'unknown')}")
        print(f"  能耗因子: {self.weather_profile.get('energy_factor', 1.0):.2f}")

        from weather.wind_model import build_wind_vector_from_profile, compute_segment_weather_penalty

        # 构建风向量
        wind_vector = build_wind_vector_from_profile(self.weather_profile)

        waypoints_with_enhanced_cost = []
        total_base_cost = 0
        total_weather_penalty = 0

        for i in range(len(path_points) - 1):
            p1 = np.array([path_points[i][0], path_points[i][1], 0])
            p2 = np.array([path_points[i+1][0], path_points[i+1][1], 0])

            # 计算天气惩罚
            penalty_info = compute_segment_weather_penalty(p1, p2, self.weather_profile)

            base_cost = penalty_info['base_cost']
            total_penalty = penalty_info['total']

            waypoints_with_enhanced_cost.append((path_points[i][0], path_points[i][1], total_penalty))

            total_base_cost += base_cost
            total_weather_penalty += (total_penalty - base_cost)

        # 添加最后一个点
        if len(path_points) > 0:
            waypoints_with_enhanced_cost.append((path_points[-1][0], path_points[-1][1], 1.0))

        avg_cost = np.mean([pt[2] for pt in waypoints_with_enhanced_cost]) if waypoints_with_enhanced_cost else 0

        print(f"  平均代价: {avg_cost:.3f}")
        print(f"  天气惩罚总计: {total_weather_penalty:.3f}")

        # 更新天气统计信息
        if hasattr(self, 'stats'):
            self.stats['weather_scene'] = self.weather_profile.get('scene', 'unknown')
            self.stats['weather_label'] = self.weather_profile.get('label', '未知')
            self.stats['weather_penalty_total'] = round(total_weather_penalty, 3)
            self.stats['weather_risk_level'] = self.weather_profile.get('risk_level', 'unknown')

        return waypoints_with_enhanced_cost

    def get_weather_info(self):
        """
        获取当前天气信息（用于导出到 mission JSON）

        Returns:
            dict: 天气信息字典
        """
        if hasattr(self, 'mission_wind') and self.mission_wind is not None:
            return dict(self.mission_wind)  # 创建副本
        return {}

    def optimize_path_with_wind(self, waypoints, alpha=0.05, beta=0.5):
        """
        基于风的局部路径优化

        Args:
            waypoints: 原始航点 [(x, y), ...]
            alpha: 风影响系数（默认0.05）
            beta: 距离约束系数（默认0.5）

        Returns:
            list: 优化后的航点 [(x, y), ...]
        """
        if not waypoints:
            return []

        print(f"[优化] 基于风的局部路径优化（风向={self.wind_direction}°, 风速={self.wind_speed}m/s）...")

        optimized = []
        total_adjustment = 0
        total_wind_cost_before = 0
        total_wind_cost_after = 0

        # 定义搜索范围（扩大至±5像素以获得更明显的风影响）
        search_range = 5  # ±5像素
        search_offsets = []
        for dx in range(-search_range, search_range + 1):
            for dy in range(-search_range, search_range + 1):
                search_offsets.append((dx, dy))

        # 计算原始风成本（使用强化惩罚机制）
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]
            path_angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            if path_angle < 0:
                path_angle += 360
            # 使用有向角度差
            angle_diff_signed = path_angle - self.wind_direction
            angle_diff_signed = (angle_diff_signed + 180) % 360 - 180
            angle_rad = np.radians(angle_diff_signed)
            cos_theta = np.cos(angle_rad)

            # 强化风惩罚（使用归一化公式，避免负值）
            if cos_theta < 0:
                # 逆风：强惩罚
                wind_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
            else:
                # 顺风：奖励
                wind_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)
            total_wind_cost_before += wind_cost

        for idx, (orig_x, orig_y) in enumerate(waypoints):
            if idx == 0 or idx == len(waypoints) - 1:
                # 起终点不优化
                # 确保起终点在边界内
                clamped_x = max(0, min(int(orig_x), self.width - 1))
                clamped_y = max(0, min(int(orig_y), self.height - 1))
                optimized.append((clamped_x, clamped_y))
                continue

            # 检查原始位置是否在边界内，如果不在则跳过此点
            if not (0 <= orig_x < self.width and 0 <= orig_y < self.height):
                # 使用夹紧的位置
                clamped_x = max(0, min(int(orig_x), self.width - 1))
                clamped_y = max(0, min(int(orig_y), self.height - 1))
                optimized.append((clamped_x, clamped_y))
                continue

            best_x, best_y = orig_x, orig_y
            best_cost = float('inf')

            # 首先计算原始位置的成本（使用强化惩罚机制）
            if idx < len(waypoints) - 1:
                next_x, next_y = waypoints[idx + 1]
                path_angle = np.degrees(np.arctan2(next_y - orig_y, next_x - orig_x))
                if path_angle < 0:
                    path_angle += 360
                angle_diff_signed = path_angle - self.wind_direction
                angle_diff_signed = (angle_diff_signed + 180) % 360 - 180
                angle_rad = np.radians(angle_diff_signed)
                cos_theta = np.cos(angle_rad)

                # 强化风惩罚（使用归一化公式，避免负值）
                if cos_theta < 0:
                    # 逆风：强惩罚
                    wind_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
                else:
                    # 顺风：奖励
                    wind_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)
                best_cost = 8 * wind_cost  # 原始位置距离代价为0，使用8倍风权重

            # 局部搜索
            for dx, dy in search_offsets:
                candidate_x = int(orig_x + dx)
                candidate_y = int(orig_y + dy)

                # 边界检查
                if (candidate_x < 0 or candidate_x >= self.width or
                    candidate_y < 0 or candidate_y >= self.height):
                    continue

                # 距离限制（防止跑飞，限制在8像素内）
                dist = np.sqrt(dx**2 + dy**2)
                if dist > 8:
                    continue

                dist_cost = beta * dist

                # 强化风惩罚机制（明显区分顺风/逆风）
                if idx < len(waypoints) - 1:
                    # 计算到下一点的方向
                    next_x, next_y = waypoints[idx + 1]
                    path_angle = np.degrees(np.arctan2(next_y - candidate_y, next_x - candidate_x))
                    if path_angle < 0:
                        path_angle += 360

                    # 计算风的影响（使用有向角度差）
                    angle_diff_signed = path_angle - self.wind_direction
                    angle_diff_signed = (angle_diff_signed + 180) % 360 - 180
                    angle_rad = np.radians(angle_diff_signed)
                    cos_theta = np.cos(angle_rad)

                    # 强化风惩罚：逆风强惩罚，顺风奖励
                    if cos_theta < 0:
                        # 逆风：强惩罚
                        wind_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
                    else:
                        # 顺风：奖励
                        wind_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)

                    # 增强风权重（8倍风成本）
                    segment_cost = dist_cost + 8 * wind_cost

                    if segment_cost < best_cost:
                        best_cost = segment_cost
                        best_x, best_y = candidate_x, candidate_y
                        total_adjustment += 1

            optimized.append((best_x, best_y))

        # 计算优化后的风成本（使用强化惩罚机制）
        for i in range(len(optimized) - 1):
            x1, y1 = optimized[i]
            x2, y2 = optimized[i + 1]
            path_angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            if path_angle < 0:
                path_angle += 360
            # 使用有向角度差
            angle_diff_signed = path_angle - self.wind_direction
            angle_diff_signed = (angle_diff_signed + 180) % 360 - 180
            angle_rad = np.radians(angle_diff_signed)
            cos_theta = np.cos(angle_rad)

            # 强化风惩罚（使用与其他地方一致的公式）
            if cos_theta < 0:
                # 逆风：强惩罚
                wind_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
            else:
                # 顺风：奖励
                wind_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)
            total_wind_cost_after += wind_cost

        avg_adjustment = total_adjustment / len(waypoints) if waypoints else 0
        print(f"  调整点数: {total_adjustment}/{len(waypoints)}")
        print(f"  平均调整: {avg_adjustment:.2f} 像素")
        print(f"  优化前风成本: {total_wind_cost_before:.3f}")
        print(f"  优化后风成本: {total_wind_cost_after:.3f}")
        improvement = ((1 - total_wind_cost_after / total_wind_cost_before) * 100
                    if total_wind_cost_before > 0 else 0)
        print(f"  改善: {improvement:.1f}%")

        return optimized

    # =====================================================
    # V3 新增功能：路径平滑
    # =====================================================

    def smooth_path_moving_average(self, path, window=None):
        """
        路径平滑：滑动平均

        Args:
            path: 路径数组 [[x, y, z], ...]
            window: 窗口大小（默认使用self.smooth_window）

        Returns:
            numpy.ndarray: 平滑后的路径
        """
        if window is None:
            window = self.smooth_window

        print(f"[平滑] 滑动平均（窗口={window}）...")

        path = np.array(path)
        smoothed = []

        for i in range(len(path)):
            # 获取窗口内的点
            start_idx = max(0, i - window)
            end_idx = min(len(path), i + window + 1)
            window_pts = path[start_idx:end_idx]

            # 计算平均值
            avg = np.mean(window_pts, axis=0)
            smoothed.append(avg)

        return np.array(smoothed)

    def smooth_path_spline(self, path, smooth_factor=5.0):
        """
        路径平滑：Spline插值（高级）

        Args:
            path: 路径数组 [[x, y, z], ...]
            smooth_factor: 平滑因子（越大越平滑）

        Returns:
            numpy.ndarray: 平滑后的路径
        """
        print(f"[平滑] Spline插值（平滑因子={smooth_factor}）...")

        # 检查依赖是否可用
        if splprep is None or splev is None:
            print("[WARN] scipy.interpolate 未安装，使用滑动平均")
            return self.smooth_path_moving_average(path)

        path = np.array(path)

        # 提取x, y, z坐标
        x, y, z = path[:, 0], path[:, 1], path[:, 2]

        try:
            # 使用Spline拟合
            tck, _ = splprep([x, y, z], s=smooth_factor, k=3)

            # 生成新的采样点
            u_new = np.linspace(0, 1, len(path))

            # 计算Spline
            x_new, y_new, z_new = splev(u_new, tck)

            return np.vstack([x_new, y_new, z_new]).T

        except Exception as e:
            print(f"[WARN] Spline平滑失败，使用滑动平均: {e}")
            return self.smooth_path_moving_average(path)

    # =====================================================
    # V3 新增功能：巡检语义增强
    # =====================================================

    def extract_tower_points(self, interval=10):
        """
        提取电塔节点（巡检语义）

        Args:
            interval: 采样间隔（每隔多少个点取一个电塔）

        Returns:
            list: 电塔点列表 [(x, y, z), ...]
        """
        print(f"[语义] 提取电塔节点（间隔={interval}）...")

        if self.path_3d_smooth is None or len(self.path_3d_smooth) == 0:
            if self.path_3d and len(self.path_3d) > 0:
                self.path_3d_smooth = np.array(self.path_3d)
            else:
                raise ValueError("请先构建3D路径")

        # 确保是numpy数组
        path = np.array(self.path_3d_smooth)

        # 间隔采样（只取x, y，重新计算z为塔顶高度）
        towers = []
        for pt in path[::interval]:
            x, y = pt[0], pt[1]
            # 塔顶高度 = 地形高度 + 50
            tower_z = float(self.height_map_smooth[int(y), int(x)]) + 50
            towers.append((x, y, tower_z))

        self.tower_points = towers

        # 确保包含最后一个点
        if len(path) > 0:
            last_pt = path[-1]
            x, y = last_pt[0], last_pt[1]
            last_z = float(self.height_map_smooth[int(y), int(x)]) + 50
            if not self.tower_points or self.tower_points[-1] != (x, y, last_z):
                self.tower_points.append((x, y, last_z))

        print(f"  提取 {len(self.tower_points)} 个电塔节点")

        return self.tower_points

    # =====================================================
    # V3 新增功能：路径高度优化
    # =====================================================

    def apply_flight_fluctuation(self):
        """
        应用飞行波动（模拟真实飞行）

        在基础高度上添加小的正弦波动
        """
        print("[优化] 应用飞行波动...")

        if not self.path_3d:
            raise ValueError("请先构建3D路径")

        path_3d_fluctuated = []

        for i, (x, y, z) in enumerate(self.path_3d):
            # 添加小的正弦波动（±3米）
            fluctuation = 3 * np.sin(i / 10.0)

            z_new = z + fluctuation
            path_3d_fluctuated.append((x, y, z_new))

        self.path_3d = path_3d_fluctuated

        print(f"  波动范围: ±3 米")

        return self.path_3d

    # =====================================================
    # V3 新增功能：能耗评估
    # =====================================================

    def compute_energy_cost(self, path_3d, wind_speed):
        """
        计算无人机能耗成本

        Args:
            path_3d: 3D路径点 [(x, y, z), ...] 或 numpy数组
            wind_speed: 风速 (m/s)

        Returns:
            dict: {
                'total_energy': 总能耗,
                'avg_energy_per_meter': 每米平均能耗
            }
        """
        print(f"[能耗] 计算航线能耗（风速={wind_speed}m/s）...")

        if path_3d is None or len(path_3d) < 2:
            return {'total_energy': 0, 'avg_energy_per_meter': 0}

        total_energy = 0
        total_distance = 0

        for i in range(len(path_3d) - 1):
            p1 = path_3d[i]
            p2 = path_3d[i + 1]

            # 计算水平距离
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            distance = np.sqrt(dx**2 + dy**2)

            # 计算爬升高度（只计算正向爬升）
            climb = max(0, p2[2] - p1[2])

            # 能耗模型：
            # - 基础能耗：距离 * (1 + 0.1 * 风速)
            # - 爬升能耗：爬升高度 * 2.0
            segment_energy = distance * (1 + 0.1 * wind_speed) + climb * 2.0

            total_energy += segment_energy
            total_distance += distance

        avg_energy_per_meter = total_energy / total_distance if total_distance > 0 else 0

        print(f"  总能耗: {total_energy:.1f} 单位")
        print(f"  每米能耗: {avg_energy_per_meter:.3f} 单位/米")

        return {
            'total_energy': total_energy,
            'avg_energy_per_meter': avg_energy_per_meter
        }

    # =====================================================
    # V3 新增功能：巡检任务生成
    # =====================================================

    def generate_inspection_tasks(self, path_3d, interval=10):
        """
        在路径上生成巡检任务点（拍照点）

        Args:
            path_3d: 3D路径点 [(x, y, z), ...]
            interval: 任务点间隔（每N个点生成一个任务）

        Returns:
            list: 任务列表 [
                {
                    "position": (x, y, z),
                    "action": "capture",
                    "camera_angle": -45,
                    "id": 0
                },
                ...
            ]
        """
        print(f"[任务] 生成巡检任务点（间隔={interval}）...")

        if path_3d is None or len(path_3d) < 1:
            print("  警告: 路径为空，无法生成任务点")
            return []

        tasks = []
        task_id = 0

        # 每隔interval个点生成一个任务点
        for i in range(0, len(path_3d), interval):
            point = path_3d[i]

            task = {
                "position": (float(point[0]), float(point[1]), float(point[2])),
                "action": "capture",
                "camera_angle": -45,  # 向下45°拍摄
                "id": task_id
            }
            tasks.append(task)
            task_id += 1

        # 确保最后一个点也是任务点
        if len(path_3d) > 0 and (len(path_3d) - 1) % interval != 0:
            last_point = path_3d[-1]
            task = {
                "position": (float(last_point[0]), float(last_point[1]), float(last_point[2])),
                "action": "capture",
                "camera_angle": -45,
                "id": task_id
            }
            tasks.append(task)

        self.tasks = tasks
        print(f"  任务点数量: {len(tasks)}")

        return tasks

    # =====================================================
    # V3 新增功能：统计信息
    # =====================================================

    def compute_statistics(self):
        """计算路径统计信息"""
        print("[统计] 计算路径统计信息...")

        if self.path_3d_smooth is None or len(self.path_3d_smooth) == 0:
            if self.path_3d and len(self.path_3d) > 0:
                self.path_3d_smooth = np.array(self.path_3d)
            else:
                raise ValueError("请先构建3D路径")

        path = np.array(self.path_3d_smooth)

        # 计算路径长度
        length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            dz = path[i][2] - path[i-1][2]
            length += np.sqrt(dx**2 + dy**2 + dz**2)

        # 高度统计
        heights = path[:, 2]
        min_height = np.min(heights)
        max_height = np.max(heights)
        avg_height = np.mean(heights)

        # 坡度变化
        slopes = []
        for i in range(1, len(path)):
            dh = path[i][2] - path[i-1][2]
            dd = np.sqrt((path[i][0] - path[i-1][0])**2 + (path[i][1] - path[i-1][1])**2)
            if dd > 0:
                slopes.append(abs(dh / dd) * 100)  # 百分比

        avg_slope = np.mean(slopes) if slopes else 0
        max_slope = np.max(slopes) if slopes else 0

        # 计算风成本统计
        total_wind_cost = 0
        avg_wind_cost = 0
        if self.waypoints_with_cost:
            costs = [pt[2] for pt in self.waypoints_with_cost]
            total_wind_cost = sum(costs)
            avg_wind_cost = np.mean(costs)

        # 计算能耗成本
        energy_result = self.compute_energy_cost(path, self.wind_speed)

        # 获取任务点数量
        task_count = len(self.tasks) if hasattr(self, 'tasks') and self.tasks else 0

        self.stats = {
            'waypoint_count': len(path),
            'tower_count': len(self.tower_points) if self.tower_points else 0,
            'task_count': task_count,
            'path_length': length,
            'min_height': min_height,
            'max_height': max_height,
            'avg_height': avg_height,
            'height_range': max_height - min_height,
            'avg_slope': avg_slope,
            'max_slope': max_slope,
            'total_wind_cost': total_wind_cost,
            'avg_wind_cost': avg_wind_cost,
            'energy_cost': energy_result['total_energy'],
            'avg_energy_per_meter': energy_result['avg_energy_per_meter']
        }

        # 添加天气统计信息
        if hasattr(self, 'weather_profile') and self.weather_profile:
            self.stats['weather_scene'] = self.weather_profile.get('scene', 'unknown')
            self.stats['weather_label'] = self.weather_profile.get('label', '未知')
            self.stats['weather_risk_level'] = self.weather_profile.get('risk_level', 'unknown')
            # 如果 weather_penalty_total 还没有设置，初始化为 0
            if 'weather_penalty_total' not in self.stats:
                self.stats['weather_penalty_total'] = 0.0
            # 计算能耗评分（基于能耗因子和路径长度）
            energy_factor = self.weather_profile.get('energy_factor', 1.0)
            self.stats['estimated_energy_score'] = round(energy_result['total_energy'] * energy_factor, 2)

        print(f"  航点数量: {self.stats['waypoint_count']}")
        print(f"  电塔数量: {self.stats['tower_count']}")
        print(f"  巡检任务点: {self.stats['task_count']}")
        print(f"  路径长度: {self.stats['path_length']:.1f} 米")
        print(f"  高度范围: [{self.stats['min_height']:.1f}, {self.stats['max_height']:.1f}] 米")
        print(f"  平均坡度: {self.stats['avg_slope']:.1f}%")
        print(f"  最大坡度: {self.stats['max_slope']:.1f}%")
        print(f"  风成本总和: {self.stats['total_wind_cost']:.3f}")
        print(f"  平均风成本: {self.stats['avg_wind_cost']:.3f}")
        print(f"  总能耗: {self.stats['energy_cost']:.1f} 单位")
        print(f"  每米能耗: {self.stats['avg_energy_per_meter']:.3f} 单位/米")

        return self.stats

    # =====================================================
    # V3 新增功能：高级3D可视化
    # =====================================================

    def step7_build_3d_path(self):
        """STEP 7: 构建3D路径（增强版）"""
        print("[STEP 7] 构建3D路径（增强版）...")

        if not self.waypoints:
            self.step5_sample_waypoints()

        if self.height_map_smooth is None:
            raise ValueError("请先设置地形（调用step6_smooth_terrain）")

        h, w = self.height_map_smooth.shape
        print(f"  地形尺寸: {w}x{h}")

        self.path_3d = []
        terrain_heights = []

        for i, (x, y) in enumerate(self.waypoints):
            # 边界检查
            if not (0 <= x < w and 0 <= y < h):
                print(f"  [WARN] 跳过越界点: ({x}, {y})")
                continue

            # 获取地形高度
            terrain_height = float(self.height_map_smooth[y, x])

            # 计算飞行高度（悬浮于地形之上）
            z = terrain_height + 25

            self.path_3d.append((x, y, z))
            terrain_heights.append(terrain_height)

        print(f"  3D航点数: {len(self.path_3d)}")

        if terrain_heights:
            print(f"  地形高度范围: [{min(terrain_heights):.1f}, {max(terrain_heights):.1f}] 米")
            print(f"  飞行高度范围: Z=[{min(p[2] for p in self.path_3d):.1f}, {max(p[2] for p in self.path_3d):.1f}] 米")

            # 验证
            terrain_range = max(terrain_heights) - min(terrain_heights)
            flight_range = max(p[2] for p in self.path_3d) - min(p[2] for p in self.path_3d)
            print(f"  高度变化: 地形差={terrain_range:.1f}m, 飞行差={flight_range:.1f}m")

        # 应用飞行波动（可选）
        if self.add_flight_fluctuation:
            self.apply_flight_fluctuation()

        # 路径平滑
        if self.use_spline:
            self.path_3d_smooth = self.smooth_path_spline(self.path_3d)
        else:
            self.path_3d_smooth = self.smooth_path_moving_average(self.path_3d)

        return self.path_3d_smooth

    def step8_visualize_3d_enhanced(self, save_path="figures/powerline_v3_final.html"):
        """
        STEP 8: Plotly 3D可视化（答辩级）

        增强功能：
        - 地形裁剪（聚焦路径区域）
        - 地形光照效果
        - 电塔节点标记
        - 专业相机角度
        - 统计信息显示
        """
        print("[STEP 8] 3D可视化（答辩级）...")

        import plotly.graph_objects as go
        from config.settings import FIGURES_DIR, FAST_MODE, FAST_MODE_NO_POPUP

        fig = go.Figure()

        h, w = self.height_map_smooth.shape

        # ===== 地形裁剪（聚焦路径区域） =====
        margin = 100
        path_x_coords = [p[0] for p in self.waypoints] if self.waypoints else []
        path_y_coords = [p[1] for p in self.waypoints] if self.waypoints else []

        if path_x_coords and path_y_coords:
            x_min = max(0, int(min(path_x_coords)) - margin)
            x_max = min(w, int(max(path_x_coords)) + margin)
            y_min = max(0, int(min(path_y_coords)) - margin)
            y_max = min(h, int(max(path_y_coords)) + margin)

            # 裁剪地形
            terrain_cropped = self.height_map_smooth[y_min:y_max, x_min:x_max]
            h_crop, w_crop = terrain_cropped.shape

            # 平移偏移量
            offset_x = x_min
            offset_y = y_min
        else:
            terrain_cropped = self.height_map_smooth
            h_crop, w_crop = h, w
            offset_x = 0
            offset_y = 0

        # ===== 1. 地形表面（裁剪后） =====
        X, Y = np.meshgrid(np.arange(w_crop), np.arange(h_crop))
        Z = terrain_cropped * 2.5  # 增强起伏

        # 地形配色（深色系）
        terrain_colorscale = [
            [0.0, '#1f4e79'],   # 深蓝（低）
            [0.3, '#2b8cbe'],   # 中蓝
            [0.6, '#41ab5d'],   # 绿色
            [1.0, '#005a32']    # 深绿（高）
        ]

        fig.add_trace(go.Surface(
            x=X,
            y=Y,
            z=Z,
            colorscale=terrain_colorscale,
            colorbar=dict(
                title='地形高度 (m)',
                x=0.0,
                y=0.5,
                len=0.55,
                thickness=14
            ),
            opacity=0.8,
            name='地形',
            showscale=True,
            hovertemplate='X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>',
            # 光照增强
            lighting=dict(
                ambient=0.2,
                diffuse=1.0,
                roughness=0.7,
                specular=0.4
            ),
            lightposition=dict(x=200, y=200, z=150)
        ))

        # ===== 2. 巡检路径（双层：白色底线 + 蓝色路径） =====
        if self.path_3d_smooth is not None and len(self.path_3d_smooth) > 0:
            path = np.array(self.path_3d_smooth)
            path_x = [x - offset_x for x in path[:, 0].tolist()]
            path_y = [y - offset_y for y in path[:, 1].tolist()]
            # 重新计算z值：地形部分缩放2.5倍，飞行高度保持不变
            path_z = []
            for x, y, z in path:
                orig_x, orig_y = int(x), int(y)
                terrain_h = float(self.height_map_smooth[orig_y, orig_x])
                flight_h = z - terrain_h  # 提取飞行高度
                new_z = terrain_h * 2.5 + flight_h  # 地形缩放，飞行高度不变
                path_z.append(new_z)

            # 白色底线（增强对比）
            fig.add_trace(go.Scatter3d(
                x=path_x,
                y=path_y,
                z=path_z,
                mode='lines',
                line=dict(color='white', width=12),
                showlegend=False,
                hovertemplate='路径<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
            ))

            # 蓝色路径（主体）
            fig.add_trace(go.Scatter3d(
                x=path_x,
                y=path_y,
                z=path_z,
                mode='lines',
                line=dict(color='#0066ff', width=6),
                name=f'巡检路径 ({len(path)}点)',
                hovertemplate='路径<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
            ))

            # 风影响可视化（分段着色）
            if self.waypoints_with_cost and len(self.waypoints_with_cost) > 1:
                wind_x = []
                wind_y = []
                wind_z = []
                wind_colors = []

                for i, (wx, wy, cost) in enumerate(self.waypoints_with_cost[:-1]):
                    orig_x, orig_y = int(wx), int(wy)
                    terrain_h = float(self.height_map_smooth[orig_y, orig_x])
                    new_z = terrain_h * 2.5 + 25  # 飞行高度
                    wind_x.append(wx - offset_x)
                    wind_y.append(wy - offset_y)
                    wind_z.append(new_z)
                    wind_colors.append(cost)

                if wind_x:
                    # 添加下一段的终点（闭合）
                    last_pt = self.waypoints_with_cost[-1]
                    orig_x, orig_y = int(last_pt[0]), int(last_pt[1])
                    terrain_h = float(self.height_map_smooth[orig_y, orig_x])
                    last_z = terrain_h * 2.5 + 25
                    wind_x.append(last_pt[0] - offset_x)
                    wind_y.append(last_pt[1] - offset_y)
                    wind_z.append(last_z)

                # 分段绘制风影响
                for i in range(len(wind_x) - 1):
                    fig.add_trace(go.Scatter3d(
                        x=[wind_x[i], wind_x[i+1]],
                        y=[wind_y[i], wind_y[i+1]],
                        z=[wind_z[i], wind_z[i+1]],
                        mode='lines',
                        line=dict(
                            width=3,
                            color=wind_colors[i],
                        ),
                        showlegend=False,
                        hovertemplate=f'段{i+1}<br>Cost: {wind_colors[i]:.3f}<extra></extra>'
                    ))

                # 添加风影响图例
                if wind_colors:
                    fig.add_trace(go.Scatter3d(
                        x=[None], y=[None], z=[None],
                        mode='markers',
                        marker=dict(
                            size=0,
                            color=wind_colors[0],
                            colorscale='RdYlBu_r',
                            cmin=min(wind_colors),
                            cmax=max(wind_colors),
                            colorbar=dict(
                                title='Wind Cost<br>(Red=Headwind)',
                                x=0.08,
                                y=0.48,
                                len=0.35,
                                thickness=12
                            )
                        ),
                        name='Wind Effect'
                    ))

            # 起点标记
            fig.add_trace(go.Scatter3d(
                x=[path_x[0]], y=[path_y[0]], z=[path_z[0]],
                mode='markers',
                marker=dict(size=8, color='green', symbol='diamond'),
                name='Start'
            ))

            # 终点标记
            fig.add_trace(go.Scatter3d(
                x=[path_x[-1]], y=[path_y[-1]], z=[path_z[-1]],
                mode='markers',
                marker=dict(size=8, color='orange', symbol='diamond'),
                name='End'
            ))

        # ===== 3. 电网（红色虚线） =====
        if self.polyline and len(self.polyline) >= 20:
            poly_x = [p[0] - offset_x for p in self.polyline]
            poly_y = [p[1] - offset_y for p in self.polyline]
            # 使用原始坐标获取地形高度
            poly_z = [self.height_map_smooth[p[1], p[0]] * 2.5 + 2 for p in self.polyline]

            fig.add_trace(go.Scatter3d(
                x=poly_x,
                y=poly_y,
                z=poly_z,
                mode='lines',
                line=dict(color='red', width=4, dash='dot'),
                name=f'Power Line ({len(self.polyline)}点)',
                hovertemplate='电网<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
            ))

        # ===== 4. 电塔（竖线结构+塔顶小点+塔间连线） =====
        if self.tower_points:
            towers = np.array(self.tower_points)
            tower_x = [x - offset_x for x in towers[:, 0].tolist()]
            tower_y = [y - offset_y for y in towers[:, 1].tolist()]
            # 重新计算z值：地形部分缩放2.5倍，塔高50保持不变
            tower_z = []
            for x, y, z in towers:
                orig_x, orig_y = int(x), int(y)
                terrain_h = float(self.height_map_smooth[orig_y, orig_x])
                tower_h = z - terrain_h  # 提取塔高
                new_z = terrain_h * 2.5 + tower_h  # 地形缩放，塔高不变
                tower_z.append(new_z)

            # 绘制电塔竖线（从地面到塔顶）
            for i, (tx, ty, tz) in enumerate(zip(tower_x, tower_y, tower_z)):
                # 转换回原始坐标获取地形高度，并应用相同的缩放
                orig_x = int(tx + offset_x)
                orig_y = int(ty + offset_y)
                ground_z = float(self.height_map_smooth[orig_y, orig_x]) * 2.5
                fig.add_trace(go.Scatter3d(
                    x=[tx, tx],
                    y=[ty, ty],
                    z=[ground_z, tz],
                    mode='lines',
                    line=dict(color='black', width=4),
                    showlegend=False,
                    hoverinfo='skip'
                ))

            # 塔顶小点
            fig.add_trace(go.Scatter3d(
                x=tower_x,
                y=tower_y,
                z=tower_z,
                mode='markers',
                marker=dict(size=3, color='black'),
                showlegend=False,
                hovertemplate='塔顶<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
            ))

            # 塔之间连线（电网）
            fig.add_trace(go.Scatter3d(
                x=tower_x,
                y=tower_y,
                z=tower_z,
                mode='lines',
                line=dict(color='gray', width=2),
                name=f'Power Line ({len(self.tower_points)}塔)',
                hovertemplate='电网<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
            ))

        # ===== 5. 计算并显示统计信息 =====
        stats = self.compute_statistics()

        title_text = (
            f"UAV Powerline Inspection Path Planning (3D Terrain-Aware)<br>"
            f"<sup>UAV电网巡检路径规划</sup><br>"
            f"<sup style='font-size: 11px;'>"
            f"Path Length: {stats['path_length']:.1f}m | "
            f"Waypoints: {stats['waypoint_count']} | "
            f"Towers: {stats['tower_count']} | "
            f"Altitude: {stats['min_height']:.1f}-{stats['max_height']:.1f}m | "
            f"Avg Slope: {stats['avg_slope']:.1f}%"
            f"</sup>"
        )

        # ===== 6. 专业相机与场景设置 =====
        fig.update_layout(
            scene=dict(
                domain=dict(
                    x=[0.12, 0.95],
                    y=[0.20, 0.99]
                ),
                aspectmode='data',
                aspectratio=dict(x=1, y=1, z=1.2),
                camera=dict(
                    up=dict(x=0, y=0, z=1),
                    eye=dict(x=1.8, y=1.8, z=1.3),
                    center=dict(x=0, y=0, z=0)
                ),
                dragmode='turntable',
                uirevision='lock',
                xaxis=dict(
                    title='X',
                    showgrid=False,
                    showbackground=True,
                    backgroundcolor="rgb(240, 240, 240)"
                ),
                yaxis=dict(
                    title='Y',
                    showgrid=False,
                    showbackground=True,
                    backgroundcolor="rgb(240, 240, 240)"
                ),
                zaxis=dict(
                    title='Z',
                    showgrid=False,
                    showbackground=True,
                    backgroundcolor="rgb(240, 240, 240)",
                    range=[0, (terrain_cropped * 2.5).max() * 1.2]
                )
            ),
            width=1400,
            height=1000,
            margin=dict(l=120, r=40, t=70, b=35),
            title=dict(
                text=title_text,
                font=dict(size=20, color='rgb(50, 50, 50)', family="'Times New Roman', serif")
            ),
            paper_bgcolor='white',
            plot_bgcolor='white',
            font=dict(family="'Times New Roman', serif", size=12),
            legend=dict(
                x=0.0,
                y=0.98,
                bgcolor="rgba(255,255,255,0.8)",
                bordercolor="rgb(200, 200, 200)",
                borderwidth=1
            )
        )

        # ===== 7. 新增：UAV动态巡检动画 + 导航输入 =====
        print("  [动画] 添加UAV巡检动画...")

        # 新增：导航标记点 traces（初始隐藏）
        # 输入点（紫色）
        input_point_trace_index = len(fig.data)
        fig.add_trace(go.Scatter3d(
            x=[], y=[], z=[],
            mode='markers',
            marker=dict(size=6, color='purple', symbol='circle', line=dict(color='white', width=2)),
            name='Input Point',
            hovertemplate='Input Point<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # 路径接入点（红色标记）
        nearest_point_trace_index = len(fig.data)
        fig.add_trace(go.Scatter3d(
            x=[], y=[], z=[],
            mode='markers',
            marker=dict(
                size=6,
                color='rgba(220, 80, 80, 0.75)',
                symbol='circle',
                line=dict(color='rgba(255,255,255,0.8)', width=1)
            ),
            name='Path Anchor Point',
            hovertemplate='Path Anchor<br>(Tower对应的路径点)<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # UAV trace 索引
        uav_trace_index = len(fig.data)

        # UAV 初始位置
        fig.add_trace(go.Scatter3d(
            x=[path_x[0]],
            y=[path_y[0]],
            z=[path_z[0]],
            mode='markers',
            marker=dict(size=4, color='orange', opacity=1.0),
            name='UAV',
            showlegend=False
        ))

        # 页面初始静态，点击"播放"后才开始动画
        fig.frames = []
        print(f"  [导航] 页面初始静态，等待用户输入坐标")

        # 保存（使用自定义方法构建导航HTML）
        os.makedirs(FIGURES_DIR, exist_ok=True)
        html_path = os.path.join(FIGURES_DIR, os.path.basename(save_path))

        # 准备显示坐标数据（实际绘图使用的坐标）
        import json

        # displayPath: 实际绘图使用的路径坐标（与 trace 2 完全一致）
        if self.path_3d_smooth is not None and len(self.path_3d_smooth) > 0:
            # 将 path_x, path_y, path_z 组合成点列表
            display_path = [[float(path_x[i]), float(path_y[i]), float(path_z[i])]
                           for i in range(len(path_x))]
        else:
            display_path = []

        # inspectionPoints: 提取巡检任务点的显示坐标（与 displayPath 完全一致的转换）
        inspection_points = []
        if hasattr(self, 'tasks') and self.tasks:
            for task in self.tasks:
                pos = task['position']  # (x, y, z) 原始坐标
                orig_x, orig_y = int(pos[0]), int(pos[1])

                # 应用与 displayPath 相同的坐标转换
                display_x = float(pos[0]) - offset_x
                display_y = float(pos[1]) - offset_y

                # Z 坐标转换：地形缩放 + 飞行高度保持
                terrain_h = float(self.height_map_smooth[orig_y, orig_x])
                flight_h = float(pos[2]) - terrain_h
                display_z = terrain_h * 2.5 + flight_h

                inspection_points.append([display_x, display_y, display_z])

        # towerPoints: 提取电塔点的显示坐标（与图上黑点完全一致的转换）
        tower_points_display = []
        if hasattr(self, 'tower_points') and self.tower_points:
            for tower in self.tower_points:
                orig_x, orig_y = int(tower[0]), int(tower[1])

                # 应用与 displayPath 相同的坐标转换
                display_x = float(tower[0]) - offset_x
                display_y = float(tower[1]) - offset_y

                # Z 坐标转换：地形缩放 + 塔高保持不变
                terrain_h = float(self.height_map_smooth[orig_y, orig_x])
                tower_h = tower[2] - terrain_h  # 塔高
                display_z = terrain_h * 2.5 + tower_h

                tower_points_display.append([display_x, display_y, display_z])

        # 生成 Plotly div（不包含完整HTML）
        plot_div = fig.to_html(full_html=False, div_id='powerline-plot')

        # 构建带导航面板的完整HTML
        full_html = self._build_navigation_html(
            plot_div=plot_div,
            display_path=display_path,  # 使用实际显示坐标
            inspection_points=inspection_points,  # 巡检点显示坐标
            tower_points_display=tower_points_display,  # 电塔显示坐标
            offset_x=offset_x,          # 裁剪偏移量
            offset_y=offset_y,
            uav_trace_index=uav_trace_index,
            input_point_trace_index=input_point_trace_index,
            nearest_point_trace_index=nearest_point_trace_index,
            path_trace_index=2  # 蓝色路径是 Trace 2
        )

        # 一次性写入文件
        with open(html_path, 'w', encoding='utf-8') as f:
            f.write(full_html)

        print(f"  [保存] {html_path}")

        # 显示
        if not (FAST_MODE and FAST_MODE_NO_POPUP):
            fig.show()

        return fig

    def _build_navigation_html(self, plot_div, display_path, inspection_points, offset_x, offset_y,
                              uav_trace_index, input_point_trace_index,
                              nearest_point_trace_index, path_trace_index,
                              tower_points_display=None):
        """
        构建带导航输入面板的完整HTML

        Args:
            plot_div: Plotly生成的图表div HTML
            display_path: 实际显示使用的路径坐标 [[x,y,z], ...]（与 trace 2 完全一致）
            inspection_points: 巡检点显示坐标 [[x,y,z], ...]
            tower_points_display: 电塔显示坐标 [[x,y,z], ...]
            offset_x: X轴裁剪偏移量
            offset_y: Y轴裁剪偏移量
            uav_trace_index: UAV trace索引
            input_point_trace_index: 输入点trace索引
            nearest_point_trace_index: 最近巡检点trace索引
            path_trace_index: 执行路径trace索引（蓝色路径）

        Returns:
            完整HTML字符串
        """
        import json

        # 导航面板HTML
        nav_panel_html = """
        <div id="nav-panel">
            <h3>&#23548;&#36890;&#25511;&#21046;</h3>
            <p style="font-size:11px;color:#666;margin-bottom:10px;">
                &#36755;&#20837;&#36215;&#28857;&#22352;&#26631;,&#31995;&#32479;&#23558;&#25509;&#20837;&#26368;&#36817;&#30002;&#23613;&#23545;&#24212;&#36335;&#24452;&#28857;
            </p>
            <div id="range-info" style="font-size:11px;color:#555;margin:8px 0;padding:6px;background:#f5f5f5;border-radius:4px;">
                <div>X范围: <span id="x-range">计算中...</span></div>
                <div>Y范围: <span id="y-range">计算中...</span></div>
                <div>Z范围: <span id="z-range">计算中...</span></div>
            </div>
            <div class="input-row">
                <label>X:</label>
                <input type="number" id="input-x" step="1">
            </div>
            <div class="input-row">
                <label>Y:</label>
                <input type="number" id="input-y" step="1">
            </div>
            <div class="input-row">
                <label>Z:</label>
                <input type="number" id="input-z" step="1">
            </div>
            <div class="button-row">
                <button class="btn-generate" onclick="generatePath()">&#29983;&#25104;&#36335;&#24452;</button>
            </div>
            <div class="button-row-row">
                <button class="btn-play" onclick="playAnimation()">&#25773;&#25918;</button>
                <button class="btn-pause" onclick="pauseAnimation()">&#26242;&#20572;</button>
            </div>
            <div id="nav-info">
                &#31561;&#24453;&#36755;&#20837;...
            </div>
        </div>
        """

        # CSS样式（悬浮面板，避免与色条重叠）
        css_styles = """
        <style>
        #nav-panel {
            position: absolute;
            top: 40px;
            right: 10px;
            background: rgba(255,255,255,0.95);
            border: 2px solid #4CAF50;
            border-radius: 8px;
            padding: 15px;
            z-index: 1000;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            min-width: 200px;
            font-family: Arial, sans-serif;
        }
        #nav-panel h3 {
            margin: 0 0 5px 0;
            color: #333;
            font-size: 16px;
            text-align: center;
        }
        #nav-panel .input-row {
            margin: 8px 0;
        }
        #nav-panel label {
            display: inline-block;
            width: 25px;
            font-weight: bold;
            color: #555;
        }
        #nav-panel input {
            width: 80px;
            padding: 5px;
            border: 1px solid #ddd;
            border-radius: 4px;
            font-size: 12px;
        }
        #nav-panel .button-row {
            margin-top: 10px;
        }
        #nav-panel .button-row-row {
            display: flex;
            gap: 5px;
            margin-top: 5px;
        }
        #nav-panel button {
            flex: 1;
            background: #4CAF50;
            color: white;
            border: none;
            padding: 8px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 13px;
            font-weight: bold;
        }
        #nav-panel .btn-generate:hover {
            background: #0b7dda;
        }
        #nav-panel .btn-play {
            background: #4CAF50;
        }
        #nav-panel .btn-play:hover {
            background: #45a049;
        }
        #nav-panel .btn-pause {
            background: #ff9800;
        }
        #nav-panel .btn-pause:hover {
            background: #e68900;
        }
        #nav-info {
            margin-top: 12px;
            padding-top: 10px;
            border-top: 1px solid #ddd;
            font-size: 12px;
            color: #666;
            line-height: 1.5;
        }
        </style>
        """

        # JavaScript数据 - 保存实际显示坐标
        js_data = f"""
        <script>
        window.POWERLINE_DATA = {{
            // 实际显示使用的路径坐标（与 trace 2 完全一致）
            displayPath: {json.dumps(display_path)},
            // 巡检点显示坐标
            inspectionPoints: {json.dumps(inspection_points)},
            // 电塔显示坐标（与图上黑点完全一致）
            towerPoints: {json.dumps(tower_points_display) if tower_points_display else []},
            // 坐标偏移量（用于调试）
            offsetX: {offset_x},
            offsetY: {offset_y},
            // Trace 索引
            uavTraceIndex: {uav_trace_index},
            inputPointTraceIndex: {input_point_trace_index},
            nearestPointTraceIndex: {nearest_point_trace_index},
            pathTraceIndex: {path_trace_index},
            // 导航参数
            connectionPoints: 8
        }};

        // 全局变量：保存当前动画的 frames 和 frameNames
        window.currentFrames = null;
        window.currentFrameNames = null;

        // 全局变量：自定义定时器动画状态
        window.navAnimationState = {{
            path: [],
            timer: null,
            currentIndex: 0,
            isPlaying: false,
            frameDuration: 120
        }};
        </script>
        """

        # JavaScript导航函数
        js_functions = """
        <script>
        // ========== 计算并显示路径范围 ==========
        function initializeInputRanges() {
            try {
                var path = POWERLINE_DATA.displayPath;
                var xMin = Infinity, xMax = -Infinity;
                var yMin = Infinity, yMax = -Infinity;
                var zMin = Infinity, zMax = -Infinity;

            for (var i = 0; i < path.length; i++) {
                var p = path[i];
                if (p[0] < xMin) xMin = p[0];
                if (p[0] > xMax) xMax = p[0];
                if (p[1] < yMin) yMin = p[1];
                if (p[1] > yMax) yMax = p[1];
                if (p[2] < zMin) zMin = p[2];
                if (p[2] > zMax) zMax = p[2];
            }

            // 计算中点作为默认值
            var xMid = (xMin + xMax) / 2;
            var yMid = (yMin + yMax) / 2;
            var zMid = (zMin + zMax) / 2;

            // DOM 元素
            var xRangeEl = document.getElementById('x-range');
            var yRangeEl = document.getElementById('y-range');
            var zRangeEl = document.getElementById('z-range');

            // 显示范围信息
            xRangeEl.textContent = xMin.toFixed(0) + ' ~ ' + xMax.toFixed(0);
            yRangeEl.textContent = yMin.toFixed(0) + ' ~ ' + yMax.toFixed(0);
            zRangeEl.textContent = zMin.toFixed(0) + ' ~ ' + zMax.toFixed(0);

            // 设置输入框默认值
            document.getElementById('input-x').value = xMid.toFixed(0);
            document.getElementById('input-y').value = yMid.toFixed(0);
            document.getElementById('input-z').value = zMid.toFixed(0);

            // 存储范围用于校验
            POWERLINE_DATA.range = {
                xMin: xMin, xMax: xMax,
                yMin: yMin, yMax: yMax,
                zMin: zMin, zMax: zMax
            };
            } catch (e) {
            console.error('[initializeInputRanges ERROR]', e);
            }
        }

        // ========== 页面初始化：删除旧动画 + 计算范围 ==========
        (function() {
            initializeInputRanges();

            setTimeout(function() {
                try {
                    Plotly.deleteFrames('powerline-plot');
                } catch (e) {
                    console.warn('Delete frames error:', e);
                }
            }, 500);
        })();

        // 1. 找最近电塔（黑点）
        function findNearestTower(inputPos) {
            if (!POWERLINE_DATA.towerPoints || POWERLINE_DATA.towerPoints.length === 0) {
                return findNearestPathPoint(inputPos);
            }

            var minDist = Infinity;
            var nearestIdx = -1;
            var nearestPoint = null;

            POWERLINE_DATA.towerPoints.forEach(function(p, i) {
                var dist = Math.sqrt(
                    Math.pow(inputPos[0] - p[0], 2) +
                    Math.pow(inputPos[1] - p[1], 2) +
                    Math.pow(inputPos[2] - p[2], 2)
                );
                if (dist < minDist) {
                    minDist = dist;
                    nearestIdx = i;
                    nearestPoint = p;
                }
            });

            return {index: nearestIdx, point: nearestPoint, distance: minDist};
        }

        // 2. 找电塔对应的路径点（在displayPath中距离该电塔最近的点）
        function findPathPointForTower(towerResult) {
            var towerPoint = towerResult.point;
            var minDist = Infinity;
            var closestPathIdx = 0;
            var closestPathPoint = null;

            POWERLINE_DATA.displayPath.forEach(function(p, i) {
                var dist = Math.sqrt(
                    Math.pow(towerPoint[0] - p[0], 2) +
                    Math.pow(towerPoint[1] - p[1], 2) +
                    Math.pow(towerPoint[2] - p[2], 2)
                );
                if (dist < minDist) {
                    minDist = dist;
                    closestPathIdx = i;
                    closestPathPoint = p;
                }
            });

            return {
                pathIndex: closestPathIdx,
                pathPoint: closestPathPoint,
                distanceToPath: minDist,
                towerIndex: towerResult.index,
                towerPoint: towerPoint
            };
        }

        // 3. 找最近路径点（fallback）
        function findNearestPathPoint(inputPos) {
            var minDist = Infinity;
            var nearestIdx = -1;
            var nearestPoint = null;

            POWERLINE_DATA.displayPath.forEach(function(p, i) {
                var dist = Math.sqrt(
                    Math.pow(inputPos[0] - p[0], 2) +
                    Math.pow(inputPos[1] - p[1], 2) +
                    Math.pow(inputPos[2] - p[2], 2)
                );
                if (dist < minDist) {
                    minDist = dist;
                    nearestIdx = i;
                    nearestPoint = p;
                }
            });

            return {index: nearestIdx, point: nearestPoint, distance: minDist};
        }

        // 3. 找巡检点在 displayPath 中的最近位置
        function findClosestDisplayPathIndex(targetPoint) {
            var minDist = Infinity;
            var closestIdx = 0;

            POWERLINE_DATA.displayPath.forEach(function(p, i) {
                var dist = Math.sqrt(
                    Math.pow(targetPoint[0] - p[0], 2) +
                    Math.pow(targetPoint[1] - p[1], 2) +
                    Math.pow(targetPoint[2] - p[2], 2)
                );
                if (dist < minDist) {
                    minDist = dist;
                    closestIdx = i;
                }
            });

            return closestIdx;
        }

        // 4. 生成连接段（线性插值）
        function generateConnectionSegment(inputPos, targetPos, numPoints) {
            var segment = [];
            for (var i = 1; i <= numPoints; i++) {
                var t = i / (numPoints + 1);
                segment.push([
                    inputPos[0] + t * (targetPos[0] - inputPos[0]),
                    inputPos[1] + t * (targetPos[1] - inputPos[1]),
                    inputPos[2] + t * (targetPos[2] - inputPos[2])
                ]);
            }
            segment.push([targetPos[0], targetPos[1], targetPos[2]]);
            return segment;
        }

        // 5. 构建执行路径：连接段 + 巡检路径
        function buildExecutionPath(inputPos, pathAnchorResult) {
            // pathAnchorResult: {pathIndex, pathPoint, towerIndex, towerPoint}
            var connection = generateConnectionSegment(
                inputPos,
                pathAnchorResult.pathPoint,
                POWERLINE_DATA.connectionPoints
            );

            var remaining = POWERLINE_DATA.displayPath.slice(pathAnchorResult.pathIndex);
            return connection.concat(remaining.slice(1));
        }

        // 6. 生成动画帧
        function regenerateAnimationFrames(execPath, startIndex) {
            var frames = [];
            var connectionLength = POWERLINE_DATA.connectionPoints + 1;

            execPath.forEach(function(point, i) {
                var color = 'orange';
                var size = 5;

                if (i < connectionLength) {
                    // 接入段
                    color = (i === connectionLength - 1) ? 'red' : 'orange';
                }

                frames.push({
                    data: [{
                        x: [point[0]],
                        y: [point[1]],
                        z: [point[2]],
                        mode: 'markers',
                        marker: {size: size, color: color, opacity: 1.0}
                    }],
                    traces: [POWERLINE_DATA.uavTraceIndex],
                    name: 'NavFrame_' + (i + 1)
                });
            });

            return frames;
        }

        // ========== 1. 生成路径（不播放动画） ==========
        function generatePath() {
            // 读取输入
            var inputX = parseFloat(document.getElementById('input-x').value);
            var inputY = parseFloat(document.getElementById('input-y').value);
            var inputZ = parseFloat(document.getElementById('input-z').value);

            if (isNaN(inputX) || isNaN(inputY) || isNaN(inputZ)) {
                document.getElementById('nav-info').innerHTML = '<span style="color:red;">&#35831;&#36755;&#20837;&#26377;&#25928;&#30340;&#22352;&#26631;</span>';
                return;
            }

            // ========== 输入范围校验 ==========
            var range = POWERLINE_DATA.range;
            if (inputX < range.xMin || inputX > range.xMax ||
                inputY < range.yMin || inputY > range.yMax ||
                inputZ < range.zMin || inputZ > range.zMax) {
                document.getElementById('nav-info').innerHTML =
                    '<span style="color:red;">&#36755;&#20837;&#22352;&#26631;&#36229;&#20986;&#22320;&#22270;&#33539;&#22260;<br>' +
                    'X: ' + range.xMin.toFixed(0) + '~' + range.xMax.toFixed(0) + ', ' +
                    'Y: ' + range.yMin.toFixed(0) + '~' + range.yMax.toFixed(0) + ', ' +
                    'Z: ' + range.zMin.toFixed(0) + '~' + range.zMax.toFixed(0) + '</span>';
                return;
            }

            var inputPos = [inputX, inputY, inputZ];

            // 塔基导航逻辑：先找最近塔尖，再找该塔对应的路径点
            var nearestTower = findNearestTower(inputPos);
            if (nearestTower.index < 0) {
                document.getElementById('nav-info').innerHTML = '<span style="color:red;">&#26410;&#25214;&#21040;&#36335;&#24452;</span>';
                return;
            }

            // 找该塔对应的路径点（在displayPath中最近的点）
            var pathAnchor = findPathPointForTower(nearestTower);

            // 构建执行路径（连接到塔对应的路径点，不是塔尖）
            var execPath = buildExecutionPath(inputPos, pathAnchor);

            var execX = execPath.map(p => p[0]);
            var execY = execPath.map(p => p[1]);
            var execZ = execPath.map(p => p[2]);

            // 更新各个 trace
            // 1. 更新执行路径（Trace 2）
            Plotly.restyle('powerline-plot', {
                x: [execX],
                y: [execY],
                z: [execZ],
                line: {color: 'green', width: 6}
            }, [POWERLINE_DATA.pathTraceIndex]);

            // 2. 更新输入点（紫色）
            Plotly.restyle('powerline-plot', {
                x: [[inputX]],
                y: [[inputY]],
                z: [[inputZ]]
            }, [POWERLINE_DATA.inputPointTraceIndex]);

            // 3. 更新接入点（红色）- 显示塔对应的路径点，不是塔尖
            Plotly.restyle('powerline-plot', {
                x: [[pathAnchor.pathPoint[0]]],
                y: [[pathAnchor.pathPoint[1]]],
                z: [[pathAnchor.pathPoint[2]]]
            }, [POWERLINE_DATA.nearestPointTraceIndex]);

            // 4. 更新 UAV 起点
            Plotly.restyle('powerline-plot', {
                x: [[inputX]],
                y: [[inputY]],
                z: [[inputZ]]
            }, [POWERLINE_DATA.uavTraceIndex]);

            // ========== 清除旧动画帧 ==========
            const gd = document.getElementById('powerline-plot');
            Plotly.deleteFrames(gd);

            // ========== 生成新动画帧 ==========
            var frames = regenerateAnimationFrames(execPath, pathAnchor.pathIndex);

            // ========== 保存到全局变量 ==========
            window.currentFrames = frames;
            window.currentFrameNames = frames.map(f => f.name);

            // ========== 添加新动画帧到图表 ==========
            Plotly.addFrames(gd, frames);

            // ========== 保存执行路径到自定义动画状态 ==========
            // 清理旧 timer
            if (window.navAnimationState.timer) {
                clearInterval(window.navAnimationState.timer);
                window.navAnimationState.timer = null;
            }

            // 保存执行路径
            window.navAnimationState.path = execPath.slice();
            window.navAnimationState.currentIndex = 0;
            window.navAnimationState.isPlaying = false;

            // 更新信息面板（不自动播放）
            document.getElementById('nav-info').innerHTML =
                '<strong>&#36335;&#24452;&#24050;&#29983;&#25104;</strong><br><br>' +
                '&#25509;&#20837;&#28857;&#32034;&#24341;: ' + pathAnchor.pathIndex + '<br>' +
                '&#25509;&#20837;&#36335;&#24452;&#36317;&#31163;: ' + pathAnchor.distanceToPath.toFixed(1) + 'm<br>' +
                '&#25191;&#34892;&#33267;&#28857;: ' + execPath.length + '<br>' +
                '<br>&#28857;&#20987;"&#25773;&#25918;"&#24320;&#22987;&#21160;&#30011;';
        }

        // ========== 2. 播放动画（自定义定时器机制）==========
        function playAnimation() {
            const state = window.navAnimationState;
            const info = document.getElementById('nav-info');

            if (!state || !state.path || state.path.length === 0) {
                if (info) {
                    info.innerHTML = '<span style="color:orange;">&#35831;&#20808;&#28857;&#20985;"&#29983;&#25104;&#36335;&#24452;"</span>';
                }
                return;
            }

            if (state.isPlaying) {
                return;
            }

            // 如果已经播放到末尾，重新开始
            if (state.currentIndex >= state.path.length) {
                state.currentIndex = 0;
            }

            state.isPlaying = true;

            if (info) {
                info.innerHTML =
                    '<strong>&#21160;&#30011;&#25773;&#25918;&#20013;</strong><br>' +
                    '&#24635;&#36335;&#24452;&#28857;: ' + state.path.length + '<br>' +
                    '&#24403;&#21069;&#24067;: ' + (state.currentIndex + 1);
            }

            state.timer = setInterval(function() {
                // 检查是否应该停止
                if (!state.isPlaying || state.currentIndex >= state.path.length) {
                    clearInterval(state.timer);
                    state.timer = null;
                    state.isPlaying = false;

                    if (state.currentIndex >= state.path.length && info) {
                        info.innerHTML =
                            '<strong>&#21160;&#30011;&#25773;&#25918;&#23436;&#25104;</strong><br>' +
                            '&#24635;&#36335;&#24452;&#28857;: ' + state.path.length;
                    }
                    return;
                }

                // 获取当前点并更新 UAV 位置
                const p = state.path[state.currentIndex];

                Plotly.restyle('powerline-plot', {
                    x: [[p[0]]],
                    y: [[p[1]]],
                    z: [[p[2]]]
                }, [POWERLINE_DATA.uavTraceIndex]);

                state.currentIndex += 1;

                // 更新进度显示
                if (info && state.currentIndex <= state.path.length) {
                    info.innerHTML =
                        '<strong>&#21160;&#30011;&#25773;&#25918;&#20013;</strong><br>' +
                        '&#24635;&#36335;&#24452;&#28857;: ' + state.path.length + '<br>' +
                        '&#24403;&#21069;&#24067;: ' + state.currentIndex;
                }
            }, state.frameDuration);
        }

        // ========== 3. 暂停动画（自定义定时器机制）==========
        function pauseAnimation() {
            const state = window.navAnimationState;
            const info = document.getElementById('nav-info');

            if (!state) return;

            state.isPlaying = false;

            if (state.timer) {
                clearInterval(state.timer);
                state.timer = null;
            }

            if (info) {
                info.innerHTML =
                    '<strong>&#21160;&#30011;&#24050;&#26242;&#20572;</strong><br>' +
                    '&#24403;&#21069;&#24067;: ' + state.currentIndex + '<br>' +
                    '&#20877;&#27425;&#28857;&#20987;"&#25773;&#25918;"&#32493;';
            }
        }
        </script>
        """

        # 拼接完整HTML（悬浮面板布局）
        full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    {css_styles}
</head>
<body>
    {nav_panel_html}
    {plot_div}
    {js_data}
    {js_functions}
</body>
</html>
"""

        return full_html

    # =====================================================
    # 阶段1新增：多独立线路支持（增量兼容）
    # =====================================================

    def step4_extract_independent_lines(self, min_pixels=20):
        """
        STEP 4 (新): 提取多条独立线路

        识别图中的所有独立红色电网线路，每条线保持独立，不自动连接。

        Args:
            min_pixels: 最小像素数阈值，小于此值的连通域视为噪声

        Returns:
            List[IndependentLine]: 独立线路列表
        """
        print("[STEP 4] 提取多条独立线路...")

        if self.skeleton is None:
            self.step3_skeletonize()

        from core.independent_lines import (
            extract_independent_lines_from_skeleton,
            save_independent_lines_visualization,
            get_lines_statistics
        )

        # 提取独立线路
        self.independent_lines = extract_independent_lines_from_skeleton(
            self.skeleton,
            min_pixels=min_pixels,
            sort_polylines=True
        )

        # 设置主线路ID（最长线路）
        if self.independent_lines:
            self.primary_line_id = self.independent_lines[0].id

        # 保存可视化
        save_independent_lines_visualization(
            self.independent_lines,
            self.image_path,
            "result/step4_independent_lines.png"
        )

        # 打印统计信息
        stats = get_lines_statistics(self.independent_lines)
        print(f"[独立线路] 统计：")
        print(f"  总线路数: {stats['total_lines']}")
        print(f"  总长度: {stats['total_length']:.1f}px")
        print(f"  平均长度: {stats['avg_length']:.1f}px")
        print(f"  总点数: {stats['total_points']}")

        for detail in stats['lines_detail']:
            print(f"  - {detail['id']}: {detail['length']:.1f}px, {detail['points']}点")

        return self.independent_lines

    def step5_generate_line_inspection_points(
        self,
        spacing=100.0,
        angle_threshold_deg=25.0,
        max_points_per_line=50
    ):
        """
        STEP 5 (新): 为多条独立线路生成巡检点

        规则：
        - 起点、终点必设（endpoint，high priority）
        - 明显拐点必设（turning，high priority）
        - 长线段等距补点（sample，normal priority）
        - 限制每条线最大点数，避免过密

        Args:
            spacing: 采样间距（像素）
            angle_threshold_deg: 拐点角度阈值
            max_points_per_line: 每条线最大点数限制

        Returns:
            List[LineInspectionPoint]: 所有巡检点
        """
        print("[STEP 5] 生成线路巡检点...")

        if not self.independent_lines:
            print("[WARN] 尚未提取独立线路，请先调用 step4_extract_independent_lines()")
            return []

        from core.inspection_point_generator import (
            generate_all_inspection_points,
            save_inspection_points_visualization,
            get_inspection_points_statistics
        )

        # 使用当前地形（如果可用）
        terrain = self.height_map_smooth if hasattr(self, 'height_map_smooth') else None

        # 生成巡检点
        self.line_inspection_points, self.line_inspection_points_by_line = (
            generate_all_inspection_points(
                self.independent_lines,
                terrain=terrain,
                flight_height=self.flight_height,
                spacing=spacing,
                angle_threshold_deg=angle_threshold_deg,
                max_points_per_line=max_points_per_line
            )
        )

        # 保存可视化
        save_inspection_points_visualization(
            self.independent_lines,
            self.line_inspection_points_by_line,
            self.image_path,
            "result/step5_line_inspection_points.png"
        )

        # 打印统计信息
        stats = get_inspection_points_statistics(
            self.line_inspection_points,
            self.line_inspection_points_by_line
        )
        print(f"[巡检点] 统计：")
        print(f"  总点数: {stats['total_points']}")
        print(f"  线路数: {stats['total_lines']}")
        print(f"  - 端点: {stats['endpoint_count']}")
        print(f"  - 拐点: {stats['turning_count']}")
        print(f"  - 采样点: {stats['sample_count']}")
        print(f"  - 高优先级: {stats['high_priority_count']}")

        return self.line_inspection_points

    def step6_map_line_points_to_3d(
        self,
        terrain_raw,
        gaussian_sigma=3.0,
        enhance_resolution=False,
        spacing=100.0,
        angle_threshold_deg=25.0,
        max_points_per_line=50
    ):
        """
        STEP 6 (新): 将多线路巡检点映射到3D

        复用现有 step6_smooth_terrain 逻辑，将多线路巡检点映射到3D高程。

        Args:
            terrain_raw: 原始地形
            gaussian_sigma: 高斯模糊sigma值
            enhance_resolution: 是否提高分辨率
            spacing: 采样间距（像素）
            angle_threshold_deg: 拐点角度阈值
            max_points_per_line: 每条线最大点数限制

        Returns:
            Dict[line_id, List[LineInspectionPoint]]: 映射后的巡检点
        """
        print("[STEP 6] 映射巡检点到3D...")

        # 复用现有地形平滑逻辑
        self.step6_smooth_terrain(terrain_raw, gaussian_sigma, enhance_resolution)

        # 更新巡检点的3D坐标
        from core.inspection_point_generator import generate_all_inspection_points

        self.line_inspection_points, self.line_inspection_points_by_line = (
            generate_all_inspection_points(
                self.independent_lines,
                terrain=self.height_map_smooth,
                flight_height=self.flight_height,
                spacing=spacing,
                angle_threshold_deg=angle_threshold_deg,
                max_points_per_line=max_points_per_line
            )
        )

        print(f"[3D映射] 完成，{len(self.line_inspection_points)} 个点已映射到3D")

        return self.line_inspection_points_by_line

    # =====================================================
    # 阶段2：任务优化（增量）
    # =====================================================

    def step7_build_tasks(self):
        """
        STEP 7: 构建任务层

        从阶段1的独立线路构建任务单元。
        """
        print("[STEP 7] 构建任务层...")

        if not self.independent_lines:
            print("[WARN] 尚未提取独立线路")
            return

        from core.mission_opt import build_tasks

        self.tasks = build_tasks(
            self.independent_lines,
            self.line_inspection_points_by_line
        )

        print(f"  [任务] 构建了 {len(self.tasks)} 个任务单元")

        return self.tasks

    def step8_opt_mission(self, start_pos=None, wind=None):
        """
        STEP 8: 优化任务顺序和方向

        Args:
            start_pos: 起始位置 (可选)
            wind: 风向信息 {'direction': 角度, 'speed': 速度}
        """
        print("[STEP 8] 优化任务顺序和方向...")

        if not self.tasks:
            print("[WARN] 尚未构建任务层，请先调用 step7_build_tasks()")
            return

        # 保存风向信息
        self.mission_wind = wind

        from core.mission_opt import init_ord_nn, opt_ord_2opt, opt_line_dir
        from core.costs import build_cost_mat

        # 构建代价矩阵
        self.cost_mat = build_cost_mat(
            self.tasks,
            self.line_inspection_points_by_line,
            wind
        )

        # 初始化顺序（最近邻）
        init_order = init_ord_nn(self.tasks, self.cost_mat, start_pos)

        # 2-opt 优化
        self.line_ord = opt_ord_2opt(init_order, self.cost_mat)

        # 优化方向
        self.line_dir = opt_line_dir(
            self.line_ord,
            self.tasks,
            self.line_inspection_points_by_line,
            wind
        )

        print(f"  [优化] 任务顺序: {[self.tasks[i].id for i in self.line_ord]}")
        print(f"  [优化] 任务方向: {self.line_dir}")

        return self.line_ord, self.line_dir

    def step9_build_g_path(self, start_pos=None, add_transitions=True):
        """
        STEP 9: 构建全局路径（优化版）

        Args:
            start_pos: 起始位置 (可选)
            add_transitions: 是否添加任务间过渡点
        """
        print("[STEP 9] 构建全局路径...")

        if not self.line_ord:
            print("[WARN] 尚未优化任务，请先调用 step8_opt_mission()")
            return

        from core.mission_build import build_g_path, save_g_path_visualization

        # 构建全局路径
        self.g_path_2d, self.g_path_3d, self.g_stats = build_g_path(
            self.line_ord,
            self.line_dir,
            self.tasks,
            self.line_inspection_points_by_line,
            start_pos,
            add_transitions,
            self.mission_wind  # 传递风向信息用于成本计算
        )

        print(f"  [路径] 全局路径长度: {self.g_stats['total_len']:.1f}px")
        print(f"  [路径] 总点数: {self.g_stats['n_points']}")

        # 保存可视化
        save_g_path_visualization(
            self.g_path_2d,
            self.tasks,
            self.line_ord,
            self.line_dir,
            self.image_path,
            "result/step9_g_path.png"
        )

        return self.g_path_2d, self.g_path_3d

    def step10_prepare_vis(self):
        """
        STEP 10: 准备可视化数据

        将阶段2结果转换为可视化系统可接受的格式。
        """
        print("[STEP 10] 准备可视化数据...")

        if not self.g_path_3d:
            print("[WARN] 尚未构建全局路径，请先调用 step9_build_g_path()")
            return

        from core.vis_adapter import adapt_stage2_to_vis

        self.vis_pts, self.vis_tasks, self.vis_stats, self.anim_path_3d = \
            adapt_stage2_to_vis(self)

        print(f"  [可视化] 巡检点: {len(self.vis_pts)}")
        print(f"  [可视化] 任务段: {len(self.vis_tasks)}")
        print(f"  [可视化] 动画路径: {len(self.anim_path_3d)} 点")

        return self.vis_pts, self.vis_tasks, self.vis_stats, self.anim_path_3d

    def step11_export_stage2_demo(self, output_html="result/mission_stage2_demo.html"):
        """
        STEP 11: 导出Stage2 HTML演示

        生成阶段2任务优化的3D交互式HTML演示文件。

        Args:
            output_html: 输出HTML文件路径
        """
        print("[STEP 11] 导出Stage2 HTML演示...")

        if not self.anim_path_3d:
            print("[WARN] 尚未准备可视化数据，请先调用 step10_prepare_vis()")
            return

        from core.visualization_enhanced import create_stage2_mission_view
        import os

        # 确保输出目录存在
        os.makedirs(os.path.dirname(output_html) or '.', exist_ok=True)

        # 获取地形（使用平滑地形用于显示）
        terrain = self.height_map_smooth if hasattr(self, 'height_map_smooth') else None

        # 获取图像对象（用于底图显示）
        image_obj = self.image if hasattr(self, 'image') else None

        # 生成HTML
        html = create_stage2_mission_view(
            anim_path_3d=self.anim_path_3d,
            vis_pts=self.vis_pts,
            vis_tasks=self.vis_tasks,
            vis_stats=self.vis_stats,
            terrain=terrain,
            image_obj=image_obj,
            image_path=self.image_path
        )

        # 保存文件
        with open(output_html, 'w', encoding='utf-8') as f:
            f.write(html)

        print(f"  [保存] {output_html}")

        return output_html


    # =====================================================
    # 拓扑层方法（新增 - Phase 1）
    # =====================================================

    def step7_5_build_topo(self):
        """
        STEP 7.5: 构建拓扑图（保守版本 v1.1 + 节点去重）

        从独立线路提取拓扑节点和边，构建节点-边图结构。

        保守设计特点：
        - 长度阈值: 600px
        - 角度阈值: 60°
        - 最小边长: 150px
        - 取消曲率切分
        - 角度去重窗口: 10px
        - 切分优先级: 端点 > 角度切分 > 长度切分
        - 节点去重: 25px 距离阈值合并重复端点

        Returns:
            TopoGraph: 拓扑图
        """
        from core.topo import (
            detect_topo_nodes, merge_duplicate_nodes, update_edges_after_merge,
            split_lines_to_edges, build_topo_graph,
            update_node_3d, visualize_topo_graph, compute_topo_stats
        )

        print("[STEP 7.5] 构建拓扑图（保守版本 v1.1 + 节点去重）...")

        # 检测拓扑节点
        self.topo_nodes = detect_topo_nodes(self.independent_lines)

        # 合并重复节点（距离相近的端点）
        self.topo_nodes, old_to_new_id = merge_duplicate_nodes(self.topo_nodes, thresh=25.0)

        # 更新3D坐标
        if self.height_map_smooth is not None:
            update_node_3d(self.topo_nodes, self.height_map_smooth)

        # 切分线路为拓扑边
        raw_edges = split_lines_to_edges(self.independent_lines, self.topo_nodes)

        # 更新边的端点引用（节点合并后）
        self.topo_edges = update_edges_after_merge(raw_edges, old_to_new_id)

        # 构建拓扑图
        self.topo_graph = build_topo_graph(self.topo_nodes, self.topo_edges)

        # 计算统计信息
        self.topo_stats = compute_topo_stats(self.topo_graph, self.independent_lines)

        # 可视化拓扑图（显示边编号）
        visualize_topo_graph(self.topo_graph, self.independent_lines,
                            "result/step7_5_topo_nodes.png",
                            show_edge_numbers=True)

        # 输出详细统计
        print()
        print("[拓扑统计] 保守版本 v1.1 + 节点去重")
        print("-" * 50)
        print(f"节点总数: {self.topo_stats['total_nodes']}")
        print(f"  - 端点节点: {self.topo_stats['node_kinds'].get('endpoint', 0)}")
        print(f"  - 切分节点: {self.topo_stats['node_kinds'].get('split', 0)}")
        print(f"边总数: {self.topo_stats['total_edges']}")
        print(f"  - 直线边: {self.topo_stats['straight_edges']}")
        print(f"  - 曲线边: {self.topo_stats['curved_edges']}")
        print(f"边长度: 平均={self.topo_stats['avg_edge_length']:.1f}px, "
              f"最小={self.topo_stats['min_edge_length']:.1f}px, "
              f"最大={self.topo_stats['max_edge_length']:.1f}px")
        print(f"切分原因: {self.topo_stats['split_reasons']}")
        print(f"平均每条线路: {self.topo_stats['avg_edges_per_line']:.1f} 条边")
        print()

        # 输出每条线路的边数
        print("[线路细分详情]")
        for line_id, num_edges in sorted(self.topo_stats['edges_by_line'].items()):
            print(f"  {line_id}: {num_edges} 条边")
        print()

        return self.topo_graph

    def step8_5_build_edge_tasks(self):
        """
        STEP 8.5: 构建边任务

        将拓扑边映射为任务单元，分配巡检点。

        Returns:
            List[EdgeTask]: 边任务列表
        """
        from core.topo_task import (
            build_edge_tasks, summarize_edge_tasks,
            visualize_edge_numbers, visualize_edge_task_summary
        )

        print("[STEP 8.5] 构建边任务...")

        # 确保拓扑图已构建
        if self.topo_graph is None:
            print("[WARN] 拓扑图未构建，请先调用 step7_5_build_topo()")
            return []

        # 构建边任务
        self.edge_tasks = build_edge_tasks(
            self.topo_graph,
            self.line_inspection_points_by_line
        )

        # 统计
        self.topo_stats = summarize_edge_tasks(self.edge_tasks)

        print()
        print("[边任务统计]")
        print(f"  总任务数: {self.topo_stats['total_edges']}")
        print(f"  总巡检点: {self.topo_stats['total_points']}")
        print(f"  总长度: {self.topo_stats['total_length']:.1f}px")
        print(f"  平均点/边: {self.topo_stats['avg_points_per_edge']:.1f}")
        print()

        # 输出每条边的点数
        print("[每条边的巡检点数]")
        for edge_id, num_points in self.topo_stats['points_distribution'].items():
            print(f"  {edge_id}: {num_points} 个点")
        print()

        # 输出线路细分情况
        if self.topo_stats['line_split_info']:
            print("[线路细分详情]")
            for line_id, num_edges in sorted(self.topo_stats['line_split_info'].items()):
                print(f"  {line_id}: {num_edges} 条边")
            print()

        # 可视化边和巡检点
        visualize_edge_numbers(self.topo_graph,
                               self.line_inspection_points_by_line,
                               "result/step8_5_topo_edges.png")

        # 可视化统计信息
        visualize_edge_task_summary(self.edge_tasks,
                                     "result/step8_5_edge_task_summary.png")

        return self.edge_tasks

    def step9_0_plan_topo_mission_greedy(self, start_edge_id=None):
        """
        STEP 9.0: 拓扑任务规划（贪心版本 - 最小可运行）

        策略：
        1. 优先访问相邻边
        2. 无相邻边时跳转至最近边
        3. 直到所有边都被访问

        Args:
            start_edge_id: 起始边ID（可选）

        Returns:
            TopoMission: 任务规划结果
        """
        from core.topo_plan import (
            plan_topo_mission_greedy,
            visualize_topo_plan_greedy,
            print_edge_adjacency_summary,
            print_mission_details
        )

        print("\n" + "="*70)
        print("STEP 9.0: 拓扑任务规划（贪心版本）")
        print("="*70)

        # 确保拓扑图和边任务已构建
        if self.topo_graph is None:
            print("[WARN] 拓扑图未构建，请先调用 step7_5_build_topo()")
            return None

        if not hasattr(self, 'edge_tasks') or not self.edge_tasks:
            print("[WARN] 边任务未构建，请先调用 step8_5_build_edge_tasks()")
            return None

        # 执行贪心规划
        self.topo_mission = plan_topo_mission_greedy(
            self.topo_graph,
            self.edge_tasks,
            start_edge_id
        )

        # 打印边邻接表
        print_edge_adjacency_summary(self.topo_mission.edge_adjacency)

        # 打印任务详情
        print_mission_details(self.topo_mission)

        # 打印摘要
        print(self.topo_mission.summary())

        # 可视化
        visualize_topo_plan_greedy(
            self.topo_graph,
            self.edge_tasks,
            self.topo_mission,
            "result/step9_0_topo_plan_greedy.png"
        )

        print("\n" + "="*70)
        print("STEP 9.0: 完成")
        print("="*70 + "\n")

        return self.topo_mission

    def step9_1_plan_continuous_mission(self, start_edge_id=None):
        """
        STEP 9.1: 连续完整航迹规划

        输出一条真实连续的 mission path：
        - inspect segment + connect segment + inspect segment + ...

        Args:
            start_edge_id: 起始边ID（可选）

        Returns:
            ContinuousMission: 连续任务（包含 segments 和 full_path）
        """
        from core.topo_plan import (
            build_edge_adjacency_simple,
            build_mission_segments,
            flatten_mission_segments_to_path,
            visualize_topo_plan_continuous,
            print_mission_segments_detail,
            print_mission_statistics
        )

        print("\n" + "="*70)
        print("STEP 9.1: 连续完整航迹规划")
        print("="*70)

        # 确保拓扑图和边任务已构建
        if self.topo_graph is None:
            print("[WARN] 拓扑图未构建，请先调用 step7_5_build_topo()")
            return None

        if not hasattr(self, 'edge_tasks') or not self.edge_tasks:
            print("[WARN] 边任务未构建，请先调用 step8_5_build_edge_tasks()")
            return None

        # 构建边邻接表
        adjacency = build_edge_adjacency_simple(self.topo_graph)

        # 构建连续任务的 mission segments
        self.continuous_mission = build_mission_segments(
            self.topo_graph,
            self.edge_tasks,
            adjacency,
            start_edge_id
        )

        # 打印 segments 详情
        print_mission_segments_detail(self.continuous_mission)

        # 打印统计信息
        print_mission_statistics(self.continuous_mission)

        # 展平为完整路径
        self.continuous_mission.full_path = flatten_mission_segments_to_path(self.continuous_mission)

        print(f"[Full Path] 包含 {len(self.continuous_mission.full_path)} 个路径点")

        # 可视化
        visualize_topo_plan_continuous(
            self.topo_graph,
            self.edge_tasks,
            self.continuous_mission,
            "result/step9_1_topo_plan_continuous.png"
        )

        print("\n" + "="*70)
        print("STEP 9.1: 完成")
        print("="*70 + "\n")

        return self.continuous_mission

    def step9_2_plan_grouped_continuous_mission(self, eps: float = 150.0,
                                                 old_connect_length: float = None):
        """
        STEP 9.2: 分组感知连续航迹规划

        策略：
        1. 对 edges 进行空间分组（DBSCAN）
        2. 确定 group 访问顺序
        3. 每个 group 内连续完成
        4. 减少 inter-group long distance connects

        Args:
            eps: DBSCAN 聚类距离阈值
            old_connect_length: 旧版本 connect 长度（用于对比）

        Returns:
            GroupedContinuousMission: 分组连续任务
        """
        from core.topo_plan import (
            build_edge_adjacency_simple,
            compute_edge_centroids,
            group_edges_spatially,
            order_groups_greedy,
            build_grouped_continuous_mission,
            visualize_edge_groups,
            visualize_grouped_topo_plan,
            print_group_details,
            print_grouped_mission_statistics
        )

        print("\n" + "="*70)
        print("STEP 9.2: 分组感知连续航迹规划")
        print("="*70)

        # 确保拓扑图和边任务已构建
        if self.topo_graph is None:
            print("[WARN] 拓扑图未构建，请先调用 step7_5_build_topo()")
            return None

        if not hasattr(self, 'edge_tasks') or not self.edge_tasks:
            print("[WARN] 边任务未构建，请先调用 step8_5_build_edge_tasks()")
            return None

        # 计算边中心点
        centroids = compute_edge_centroids(self.edge_tasks)

        # 对 edges 进行分组
        groups = group_edges_spatially(self.edge_tasks, centroids, eps=eps)

        # 打印分组详情
        print_group_details(groups)

        # 可视化分组
        visualize_edge_groups(
            self.topo_graph,
            self.edge_tasks,
            groups,
            "result/step9_2_edge_groups.png"
        )

        # 确定 group 访问顺序
        group_visit_order = order_groups_greedy(groups, {t.edge_id: t for t in self.edge_tasks})

        # 构建边邻接表
        adjacency = build_edge_adjacency_simple(self.topo_graph)

        # 构建分组连续任务
        self.grouped_mission = build_grouped_continuous_mission(
            self.topo_graph,
            self.edge_tasks,
            groups,
            group_visit_order,
            adjacency
        )

        # 打印统计信息
        print_grouped_mission_statistics(self.grouped_mission, old_connect_length)

        # 可视化
        visualize_grouped_topo_plan(
            self.topo_graph,
            self.edge_tasks,
            self.grouped_mission,
            groups,
            "result/step9_2_topo_plan_grouped.png"
        )

        print("\n" + "="*70)
        print("STEP 9.2: 完成")
        print("="*70 + "\n")

        return self.grouped_mission

    def step9_3_plan_grouped_mission_optimized(self, eps: float = 150.0,
                                                 old_mission=None):
        """
        STEP 9.3: 优化的分组感知连续航迹规划

        优化目标：
        1. 优化 group visit order（基于候选方案评估）
        2. 优化 group entry/exit strategy

        Args:
            eps: DBSCAN 聚类距离阈值
            old_mission: step9_2 的 mission（用于对比）

        Returns:
            GroupedContinuousMission: 优化后的分组连续任务
        """
        from core.topo_plan import (
            build_edge_adjacency_simple,
            compute_edge_centroids,
            group_edges_spatially,
            build_grouped_continuous_mission_optimized,
            visualize_edge_groups,
            visualize_grouped_topo_plan_optimized,
            visualize_group_entry_exit_debug,
            print_group_entry_exit_summary,
            print_optimized_mission_statistics
        )

        print("\n" + "="*70)
        print("STEP 9.3: 优化的分组感知连续航迹规划")
        print("="*70)

        # 确保拓扑图和边任务已构建
        if self.topo_graph is None:
            print("[WARN] 拓扑图未构建，请先调用 step7_5_build_topo()")
            return None

        if not hasattr(self, 'edge_tasks') or not self.edge_tasks:
            print("[WARN] 边任务未构建，请先调用 step8_5_build_edge_tasks()")
            return None

        # 计算边中心点
        centroids = compute_edge_centroids(self.edge_tasks)

        # 对 edges 进行分组
        groups = group_edges_spatially(self.edge_tasks, centroids, eps=eps)

        # 构建边邻接表
        adjacency = build_edge_adjacency_simple(self.topo_graph)

        # 构建优化后的分组连续任务
        self.optimized_mission = build_grouped_continuous_mission_optimized(
            self.topo_graph,
            self.edge_tasks,
            groups,
            adjacency
        )

        # 打印 entry/exit 摘要
        print_group_entry_exit_summary(self.optimized_mission, groups)

        # 打印统计信息
        print_optimized_mission_statistics(self.optimized_mission, old_mission)

        # 可视化分组（复用 step9_2 的图）
        visualize_edge_groups(
            self.topo_graph,
            self.edge_tasks,
            groups,
            "result/step9_3_edge_groups.png"
        )

        # 可视化优化后的航迹
        visualize_grouped_topo_plan_optimized(
            self.topo_graph,
            self.edge_tasks,
            self.optimized_mission,
            groups,
            "result/step9_3_topo_plan_grouped_optimized.png"
        )

        # 可视化 entry/exit debug
        visualize_group_entry_exit_debug(
            self.topo_graph,
            self.edge_tasks,
            self.optimized_mission,
            groups,
            "result/step9_3_group_entry_exit_debug.png"
        )

        print("\n" + "="*70)
        print("STEP 9.3: 完成")
        print("="*70 + "\n")

        return self.optimized_mission

    def step9_4_plan_global_topology_optimized(self, start_edge_id=None,
                                               enable_sa=False, eps=150.0):
        """
        STEP 9.4: 新旧算法对比 + 自动择优

        策略：
        1. 同时运行旧算法 (step9_2) 和新算法
        2. 按分层优先级比较结果：
           - 一级: connect_length（越小越好）
           - 二级: connect_segment数量
           - 三级: total_length
        3. 自动选择更优方案输出

        Args:
            start_edge_id: 指定起始边ID（可选，不指定则自动优化）
            enable_sa: 新算法是否启用模拟退火（默认False，因在此数据集上未表现优势）
            eps: 空间分组距离阈值

        Returns:
            GroupedContinuousMission: 择优后的任务
        """
        from core.topo_plan import (
            build_edge_adjacency_simple,
            compute_edge_centroids,
            group_edges_spatially,
            build_grouped_continuous_mission,
            order_groups_greedy
        )
        from core.topo_global_optimizer import plan_global_topology_optimized_mission

        print("\n" + "="*70)
        print("STEP 9.4: 新旧算法对比 + 自动择优")
        print("="*70)

        # 确保拓扑图和边任务已构建
        if self.topo_graph is None:
            print("[WARN] 拓扑图未构建，请先调用 step7_5_build_topo()")
            return None

        if not hasattr(self, 'edge_tasks') or not self.edge_tasks:
            print("[WARN] 边任务未构建，请先调用 step8_5_build_edge_tasks()")
            return None

        # =====================================================
        # 方案1: 旧算法 (step9_2 贪心分组)
        # =====================================================
        print("\n[方案1] 运行旧算法 (step9_2: 贪心分组)...")

        centroids = compute_edge_centroids(self.edge_tasks)
        groups_old = group_edges_spatially(self.edge_tasks, centroids, eps=eps)
        adjacency_old = build_edge_adjacency_simple(self.topo_graph)

        try:
            from sklearn.cluster import DBSCAN
            _has_sklearn = True
        except ImportError:
            _has_sklearn = False

        if _has_sklearn:
            group_visit_order_old = order_groups_greedy(groups_old, {e.edge_id: e for e in self.edge_tasks})
            mission_old = build_grouped_continuous_mission(
                self.topo_graph, self.edge_tasks, groups_old,
                group_visit_order_old, adjacency_old
            )
        else:
            print("  [SKIP] scikit-learn未安装，跳过旧算法")
            mission_old = None

        # =====================================================
        # 方案2: 新算法 (全局拓扑优化)
        # =====================================================
        print("\n[方案2] 运行新算法 (全局拓扑优化)...")

        mission_new = plan_global_topology_optimized_mission(
            topo_graph=self.topo_graph,
            edge_tasks=self.edge_tasks,
            start_edge_id=start_edge_id,
            enable_sa=enable_sa,
            eps=eps
        )

        # =====================================================
        # 自动择优：分层比较
        # =====================================================
        print("\n[择优] 比较两个方案...")

        if mission_old is None:
            print("  [决策] 旧算法不可用，使用新算法")
            final_mission = mission_new
            winner = "新算法"
        else:
            # 提取关键指标
            old_connect = mission_old.connect_length
            new_connect = mission_new.connect_length
            old_total = mission_old.total_length
            new_total = mission_new.total_length
            old_connect_count = sum(1 for s in mission_old.segments if s.type == 'connect')
            new_connect_count = sum(1 for s in mission_new.segments if s.type == 'connect')

            print(f"\n  [旧算法] Connect={old_connect:.1f}px ({old_connect_count}段), Total={old_total:.1f}px")
            print(f"  [新算法] Connect={new_connect:.1f}px ({new_connect_count}段), Total={new_total:.1f}px")

            # 分层比较
            if abs(new_connect - old_connect) < 1.0:  # connect长度接近（误差<1px）
                # 二级：比较connect段数
                if new_connect_count < old_connect_count:
                    final_mission = mission_new
                    winner = "新算法 (connect段数更少)"
                elif new_connect_count > old_connect_count:
                    final_mission = mission_old
                    winner = "旧算法 (connect段数更少)"
                else:
                    # 三级：比较总长度
                    if new_total < old_total:
                        final_mission = mission_new
                        winner = "新算法 (总长度更短)"
                    else:
                        final_mission = mission_old
                        winner = "旧算法 (总长度更短或持平)"
            elif new_connect < old_connect:
                final_mission = mission_new
                winner = "新算法 (connect更短)"
            else:
                final_mission = mission_old
                winner = "旧算法 (connect更短或持平)"

            # 计算改进幅度
            if winner.startswith("新算法"):
                conn_improve = (old_connect - new_connect) / old_connect * 100 if old_connect > 0 else 0
                total_improve = (old_total - new_total) / old_total * 100 if old_total > 0 else 0
                print(f"\n  [结果] 采用: {winner}")
                print(f"  [改进] Connect: {conn_improve:+.1f}%, Total: {total_improve:+.1f}%")
            else:
                conn_improve = (new_connect - old_connect) / old_connect * 100 if old_connect > 0 else 0
                total_improve = (new_total - old_total) / old_total * 100 if old_total > 0 else 0
                print(f"\n  [结果] 采用: {winner}")
                print(f"  [差异] Connect: {conn_improve:+.1f}%, Total: {total_improve:+.1f}%")
                if conn_improve > 0:
                    print(f"  [说明] 新算法未产生改进，保持旧算法")

        # 保存最终结果
        self.grouped_mission = final_mission
        self.global_optimized_mission = final_mission

        print("\n" + "="*70)
        print(f"STEP 9.4: 完成 (采用 {winner})")
        print("="*70 + "\n")

        return final_mission

    def step9_5_plan_topo_mission(self, start_pos=None, wind=None):
        """
        STEP 9.5: 拓扑路径规划

        基于拓扑图进行路径规划，体现图约束。

        Args:
            start_pos: 起始位置 (可选)
            wind: 风况信息 (可选)

        Returns:
            Tuple: (边ID顺序, 方向映射, 3D路径)
        """
        from core.topo_plan import plan_topo_mission, evaluate_plan

        print("[STEP 9.5] 拓扑路径规划...")

        # 确保边任务已构建
        if not self.edge_tasks:
            print("[WARN] 边任务未构建，请先调用 step8_5_build_edge_tasks()")
            return [], {}, []

        # 保存风向信息
        self.mission_wind = wind

        # 执行拓扑规划
        self.edge_ord, self.edge_dir, self.topo_path_3d = plan_topo_mission(
            self.topo_graph,
            self.edge_tasks
        )

        # 评估结果
        eval_result = evaluate_plan(
            self.topo_graph,
            self.edge_ord,
            self.edge_dir,
            self.edge_tasks
        )

        print(f"  [拓扑规划] 连续过渡: {eval_result['continuous_transitions']}/{len(self.edge_ord)-1}")
        print(f"  [拓扑规划] 连续比例: {eval_result['continuous_ratio']:.2%}")

        return self.edge_ord, self.edge_dir, self.topo_path_3d


# =====================================================
# 便捷函数（完整流程）
# =====================================================

def plan_powerline_inspection_v3(
    image_path,
    terrain_raw,
    flight_height=30,
    sample_step=5,
    smooth_window=5,
    use_spline=False,
    add_flight_fluctuation=True,
    tower_interval=10,
    gaussian_sigma=3.0,
    enhance_resolution=False,
    output_file=None,
    wind_direction=0,
    wind_speed=5
):
    """
    便捷函数：完整的电网巡检路径规划 V3.0

    Args:
        image_path: 电网图片路径
        terrain_raw: 原始地形图
        flight_height: 飞行高度（米）
        sample_step: 采样间隔（像素）
        smooth_window: 平滑窗口大小
        use_spline: 是否使用Spline平滑
        add_flight_fluctuation: 是否添加飞行波动
        tower_interval: 电塔采样间隔
        gaussian_sigma: 地形高斯模糊sigma
        enhance_resolution: 是否提高地形分辨率
        output_file: 自定义输出文件名
        wind_direction: 风向角度（0-360°，0=东风）
        wind_speed: 风速（m/s）

    Returns:
        PowerlinePlannerV3对象
    """
    print("=" * 70)
    print("电网巡检路径规划 V3.0 - 答辩级优化版")
    print("=" * 70)

    planner = PowerlinePlannerV3(
        image_path=image_path,
        flight_height=flight_height,
        smooth_window=smooth_window,
        use_spline=use_spline,
        add_flight_fluctuation=add_flight_fluctuation,
        wind_direction=wind_direction,
        wind_speed=wind_speed
    )

    # 完整流程
    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_continuous_path()
    planner.step5_sample_waypoints(sample_step)
    planner.step6_smooth_terrain(terrain_raw, gaussian_sigma=gaussian_sigma,
                                enhance_resolution=enhance_resolution)
    planner.apply_wind_cost()  # 天气影响评估
    planner.waypoints = planner.optimize_path_with_wind(planner.waypoints, alpha=0.05, beta=0.1)  # 基于风优化路径（降低距离惩罚）
    planner.apply_wind_cost()  # 重新计算优化后的风成本（用于可视化）
    planner.step7_build_3d_path()
    planner.extract_tower_points(interval=tower_interval)
    planner.generate_inspection_tasks(planner.path_3d, interval=10)  # 生成巡检任务点
    planner.compute_statistics()
    planner.step8_visualize_3d_enhanced(save_path=output_file)

    print("\n" + "=" * 70)
    print("规划完成！")
    print("=" * 70)

    return planner


def plan_powerline_independent_lines_v1(
    image_path,
    terrain_raw,
    flight_height=30,
    min_pixels=20,
    inspection_spacing=100.0,
    angle_threshold_deg=25.0,
    max_points_per_line=50,
    gaussian_sigma=3.0,
    enhance_resolution=False,
    allow_break_fix=False,
    wind_direction=0,
    wind_speed=5
):
    """
    便捷函数：多独立线路巡检规划 V1.0（阶段1）

    新增功能：
    - 识别多条独立红色电网线路
    - 每条线路保持独立，不自动连接
    - 为每条线路生成巡检点（端点、拐点、采样点）
    - 映射到3D高程

    Args:
        image_path: 电网图片路径
        terrain_raw: 原始地形图
        flight_height: 飞行高度（米）
        min_pixels: 最小像素数阈值（过滤噪声）
        inspection_spacing: 巡检点采样间距（像素）
        angle_threshold_deg: 拐点角度阈值
        max_points_per_line: 每条线最大点数限制
        gaussian_sigma: 地形高斯模糊sigma
        enhance_resolution: 是否提高地形分辨率
        allow_break_fix: 是否启用断裂修复（默认关闭）
        wind_direction: 风向角度（0-360°，0=东风）
        wind_speed: 风速（m/s）

    Returns:
        PowerlinePlannerV3对象（包含 independent_lines 和 line_inspection_points）
    """
    print("=" * 70)
    print("多独立线路巡检规划 V1.0")
    print("=" * 70)

    planner = PowerlinePlannerV3(
        image_path=image_path,
        flight_height=flight_height,
        wind_direction=wind_direction,
        wind_speed=wind_speed
    )

    # 步骤1-3：提取骨架（复用现有逻辑）
    planner.step1_extract_redline_hsv()
    if allow_break_fix:
        planner.step2_fix_breaks()
    planner.step3_skeletonize()

    # 步骤4：提取多条独立线路（新功能）
    planner.step4_extract_independent_lines(min_pixels=min_pixels)

    # 步骤5：生成巡检点（新功能）
    planner.step5_generate_line_inspection_points(
        spacing=inspection_spacing,
        angle_threshold_deg=angle_threshold_deg,
        max_points_per_line=max_points_per_line
    )

    # 步骤6：映射到3D（新功能）
    planner.step6_map_line_points_to_3d(
        terrain_raw,
        gaussian_sigma=gaussian_sigma,
        enhance_resolution=enhance_resolution,
        spacing=inspection_spacing,
        angle_threshold_deg=angle_threshold_deg,
        max_points_per_line=max_points_per_line
    )

    print("\n" + "=" * 70)
    print("多独立线路规划完成！")
    print("=" * 70)

    return planner


def plan_powerline_mission_v2(
    image_path,
    terrain_raw,
    flight_height=30,
    min_pixels=20,
    inspection_spacing=100.0,
    angle_threshold_deg=25.0,
    max_points_per_line=50,
    gaussian_sigma=3.0,
    enhance_resolution=False,
    allow_break_fix=False,
    start_pos=None,
    wind=None,
    wind_direction=0,
    wind_speed=5,
    export_html=False,
    html_output_path="result/mission_stage2_demo.html"
):
    """
    便捷函数：多独立线路巡检任务规划 V2.0（阶段2）

    新增功能：
    - 阶段1：识别多条独立线路 + 生成巡检点
    - 阶段2：任务顺序优化 + 方向优化 + 全局路径拼接
    - 可选：导出Stage2 HTML演示

    Args:
        image_path: 电网图片路径
        terrain_raw: 原始地形图
        flight_height: 飞行高度（米）
        min_pixels: 最小像素数阈值（过滤噪声）
        inspection_spacing: 巡检点采样间距（像素）
        angle_threshold_deg: 拐点角度阈值
        max_points_per_line: 每条线最大点数限制
        gaussian_sigma: 地形高斯模糊sigma
        enhance_resolution: 是否提高地形分辨率
        allow_break_fix: 是否启用断裂修复（默认关闭）
        start_pos: 起始位置 (可选)
        wind: 风向信息 {'direction': 角度, 'speed': 速度}
        wind_direction: 风向角度（0-360°，0=东风）
        wind_speed: 风速（m/s）
        export_html: 是否导出Stage2 HTML演示（默认False）
        html_output_path: HTML输出路径

    Returns:
        PowerlinePlannerV3对象（包含阶段1+阶段2的所有结果）
    """
    print("=" * 70)
    print("多独立线路巡检任务规划 V2.0")
    print("=" * 70)

    # 准备风向信息
    if wind is None:
        wind = {'direction': wind_direction, 'speed': wind_speed}

    # 阶段1：独立线路 + 巡检点
    planner = plan_powerline_independent_lines_v1(
        image_path=image_path,
        terrain_raw=terrain_raw,
        flight_height=flight_height,
        min_pixels=min_pixels,
        inspection_spacing=inspection_spacing,
        angle_threshold_deg=angle_threshold_deg,
        max_points_per_line=max_points_per_line,
        gaussian_sigma=gaussian_sigma,
        enhance_resolution=enhance_resolution,
        allow_break_fix=allow_break_fix,
        wind_direction=wind_direction,
        wind_speed=wind_speed
    )

    # 阶段2：任务优化
    print("\n" + "=" * 70)
    print("阶段2：任务优化")
    print("=" * 70)

    planner.step7_build_tasks()
    planner.step8_opt_mission(start_pos=start_pos, wind=wind)
    planner.step9_build_g_path(start_pos=start_pos)

    # 可选：导出Stage2 HTML演示
    if export_html:
        print("\n" + "=" * 70)
        print("生成Stage2可视化演示...")
        print("=" * 70)
        planner.step10_prepare_vis()
        planner.step11_export_stage2_demo(output_html=html_output_path)

    print("\n" + "=" * 70)
    print("任务规划完成！")
    print("=" * 70)

    return planner
