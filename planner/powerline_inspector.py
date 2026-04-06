"""
=====================================================
电网巡检航线规划器 (Powerline Inspection Planner)
=====================================================

该模块实现从电网图片中提取线路并生成巡检路径。

核心逻辑:
    1. 从图片中提取红色线路（电网）
    2. 检测拐点作为电线杆位置
    3. 生成巡检任务（按顺序访问电线杆）
    4. 使用A*规划路径

Cost Function:
    电网决定"任务"（访问哪些点）
    A*决定"路径"（如何到达这些点）
=====================================================
"""

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from collections import defaultdict
from planner.astar3d import AStar3D


class PowerlineInspector:
    """
    电网巡检规划器

    从电网图片中提取线路并生成A*巡检路径
    """

    def __init__(self, image_path, flight_height=20):
        """
        初始化电网巡检规划器

        Args:
            image_path: 电网图片路径
            flight_height: 飞行高度（米）
        """
        self.image_path = image_path
        self.flight_height = flight_height
        self.image = None
        self.pixels = None
        self.poles = []
        self.powerline_mask = None
        self.width = 0
        self.height = 0

    def load_image(self):
        """加载电网图片"""
        self.image = Image.open(self.image_path).convert("RGB")
        self.width, self.height = self.image.size
        self.pixels = self.image.load()
        print(f"[INFO] 加载电网图片: {self.width}x{self.height}")
        return self.image

    def extract_powerline(self, red_threshold=180, green_threshold=100, blue_threshold=100):
        """
        从图片中提取红色线路（电网）

        Args:
            red_threshold: 红色通道阈值
            green_threshold: 绿色通道阈值
            blue_threshold: 蓝色通道阈值

        Returns:
            numpy.ndarray: 电网掩码（二值图像）
        """
        if self.image is None:
            self.load_image()

        # 创建掩码图像
        self.powerline_mask = Image.new('L', (self.width, self.height), 0)
        mask_pixels = self.powerline_mask.load()

        # 提取红色: R高, G低, B低
        powerline_points = []
        for y in range(self.height):
            for x in range(self.width):
                r, g, b = self.pixels[x, y]
                if r > red_threshold and g < green_threshold and b < blue_threshold:
                    mask_pixels[x, y] = 255
                    powerline_points.append((x, y))

        print(f"[INFO] 提取电网掩码: {len(powerline_points)} 个像素点")

        # 将PIL图像转换为numpy数组
        self.powerline_array = np.array(self.powerline_mask)

        return self.powerline_mask

    def detect_poles(self, angle_threshold=45, min_segment_length=10):
        """
        检测电网拐点作为电线杆位置

        原理:
            1. 对电网像素进行聚类
            2. 检测方向变化点
            3. 得到拐点作为电线杆

        Args:
            angle_threshold: 角度阈值（度）
            min_segment_length: 最小线段长度

        Returns:
            list: 电线杆位置列表 [(x, y), ...]
        """
        if self.powerline_mask is None:
            self.extract_powerline()

        # 获取所有电网像素点
        points = np.argwhere(self.powerline_array > 0)

        if len(points) < 3:
            print("[WARN] 电网点太少，使用端点")
            if len(points) > 0:
                self.poles = [
                    (int(points[0][1]), int(points[0][0])),
                    (int(points[-1][1]), int(points[-1][0]))
                ]
            else:
                self.poles = [(self.width // 2, self.height // 2)]
            return self.poles

        # 简化方法：将点分成网格，找每个网格的中心
        grid_size = 20
        grid_points = defaultdict(list)

        for point in points:
            y, x = point
            grid_x = x // grid_size
            grid_y = y // grid_size
            grid_points[(grid_x, grid_y)].append((x, y))

        # 对每个网格，计算中心点
        centers = []
        for grid_pos, pts in grid_points.items():
            if len(pts) > 5:  # 至少5个点才认为是有效区域
                avg_x = int(np.mean([p[0] for p in pts]))
                avg_y = int(np.mean([p[1] for p in pts]))
                centers.append((avg_x, avg_y))

        # 按y坐标排序
        centers = sorted(centers, key=lambda p: p[1])

        # 进一步筛选：只保留方向变化大的点
        if len(centers) > 2:
            self.poles = [centers[0]]
            for i in range(1, len(centers) - 1):
                prev = self.poles[-1]
                curr = centers[i]
                next_p = centers[i + 1]

                # 计算角度变化
                v1 = np.array([curr[0] - prev[0], curr[1] - prev[1]])
                v2 = np.array([next_p[0] - curr[0], next_p[1] - curr[1]])

                norm1 = np.linalg.norm(v1)
                norm2 = np.linalg.norm(v2)

                if norm1 > 0 and norm2 > 0:
                    cos_angle = np.dot(v1, v2) / (norm1 * norm2)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.degrees(np.arccos(cos_angle))

                    # 如果角度变化大，保留这个点
                    if angle < (180 - angle_threshold):
                        self.poles.append(curr)

            self.poles.append(centers[-1])
        else:
            self.poles = centers

        # 去重（相近的点合并）
        self.poles = self._merge_nearby_poles(distance_threshold=30)

        print(f"[INFO] 检测到 {len(self.poles)} 个电线杆")
        return self.poles

    def _merge_nearby_poles(self, distance_threshold=30):
        """合并距离相近的电线杆"""
        if len(self.poles) <= 1:
            return self.poles

        merged = [self.poles[0]]
        for pole in self.poles[1:]:
            last = merged[-1]
            dist = np.sqrt((pole[0] - last[0])**2 + (pole[1] - last[1])**2)
            if dist > distance_threshold:
                merged.append(pole)

        return merged

    def generate_inspection_waypoints(self, grid_map):
        """
        生成巡检航点

        将图片坐标转换为世界坐标，并设置飞行高度

        Args:
            grid_map: 栅格地图对象

        Returns:
            list: 巡检航点列表 [(x, y, z), ...]
        """
        # 不再自动检测，使用已有的poles
        if not self.poles:
            raise ValueError("没有电线杆数据，请先设置poles或调用detect_poles()")

        # 计算缩放比例（将图片坐标映射到世界坐标）
        map_size_x = grid_map.b_max[0] - grid_map.b_min[0]
        map_size_y = grid_map.b_max[1] - grid_map.b_min[1]

        scale_x = map_size_x / self.width
        scale_y = map_size_y / self.height

        waypoints = []
        for pole_x, pole_y in self.poles:
            # 转换为世界坐标
            world_x = grid_map.b_min[0] + pole_x * scale_x
            world_y = grid_map.b_min[1] + pole_y * scale_y

            # 设置飞行高度（地形高度 + 安全高度）
            if hasattr(grid_map, 'height_map'):
                grid_x = int((world_x - grid_map.b_min[0]) / grid_map.resolution)
                grid_y = int((world_y - grid_map.b_min[1]) / grid_map.resolution)
                if 0 <= grid_y < grid_map.height_map.shape[0] and 0 <= grid_x < grid_map.height_map.shape[1]:
                    terrain_height = grid_map.height_map[grid_y, grid_x]
                else:
                    terrain_height = 0
            else:
                terrain_height = 0

            world_z = terrain_height + self.flight_height

            # 转换为栅格坐标
            grid_z = int((world_z - grid_map.b_min[2]) / grid_map.resolution)

            waypoints.append((grid_x, grid_y, grid_z))

        print(f"[INFO] 生成 {len(waypoints)} 个巡检航点")
        return waypoints

    def plan_inspection_path(self, grid_map, turn_weight=0.3, weight=1.0):
        """
        规划电网巡检路径

        使用A*算法规划从起点到终点，经过所有巡检航点的路径

        Args:
            grid_map: 栅格地图对象
            turn_weight: 转弯权重
            weight: A*启发式权重

        Returns:
            list: 完整巡检路径
        """
        # 生成巡检航点
        waypoints = self.generate_inspection_waypoints(grid_map)

        if len(waypoints) < 2:
            raise ValueError("巡检航点太少，无法规划路径")

        # 创建A*规划器
        planner = AStar3D(grid_map, turn_weight=turn_weight, weight=weight)

        # 逐段规划路径
        full_path = []

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            goal = waypoints[i + 1]

            print(f"[INFO] 规划段 {i+1}/{len(waypoints)-1}: {start} -> {goal}")

            path = planner.plan(start, goal)

            if path is None:
                print(f"[WARN] 段 {i+1} 规划失败，使用直线路径")
                path = self._plan_straight_path(start, goal)

            # 将路径添加到完整路径（避免重复连接点）
            if i == 0:
                full_path.extend(path)
            else:
                full_path.extend(path[1:])

        print(f"[INFO] 完整巡检路径: {len(full_path)} 个航点")
        return full_path

    def _plan_straight_path(self, start, goal):
        """生成直线路径（备用方案）"""
        path = []
        x0, y0, z0 = start
        x1, y1, z1 = goal

        distance = np.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
        steps = max(int(distance), 2)

        for i in range(steps + 1):
            t = i / steps
            x = int(x0 + t * (x1 - x0))
            y = int(y0 + t * (y1 - y0))
            z = int(z0 + t * (z1 - z0))
            path.append((x, y, z))

        return path

    def visualize_extraction(self, save_path="result/powerline_extraction.png"):
        """
        可视化电网提取结果

        Args:
            save_path: 保存路径
        """
        import os

        if self.image is None:
            self.load_image()

        # 创建可视化图像
        vis = self.image.copy()
        draw = ImageDraw.Draw(vis)

        # 绘制检测到的电线杆
        for pole in self.poles:
            x, y = pole
            # 绿色圆圈
            draw.ellipse([x-10, y-10, x+10, y+10], fill=(0, 200, 100))
            draw.ellipse([x-12, y-12, x+12, y+12], outline=(255, 255, 255), width=2)

        # 绘制连接线
        if len(self.poles) > 1:
            for i in range(len(self.poles) - 1):
                draw.line([self.poles[i], self.poles[i + 1]], fill=(255, 255, 0), width=3)

        # 保存结果
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        vis.save(save_path)
        print(f"[INFO] 提取结果已保存: {save_path}")

        return vis
