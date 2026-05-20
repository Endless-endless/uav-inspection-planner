"""
=====================================================
环境地图模块 (Grid Map Module)
=====================================================
功能说明:
    该模块实现了3D栅格地图的构建和管理。
    从world.json文件加载地图数据，构建3D栅格，
    并提供障碍物检测、邻居查询等核心功能。

类说明:
    GridMap3D: 3D栅格地图类，负责地图加载、障碍物处理和空间查询。

主要功能:
    - 从JSON文件加载地图边界和障碍物定义
    - 构建3D栅格数组
    - 判断栅格单元是否可通行
    - 获取26连通邻域
=====================================================
"""

import json
import numpy as np
import time


class GridMap3D:
    """
    3D栅格地图类

    该类负责构建和管理3D环境地图，主要功能包括：
    1. 从world.json文件加载地图数据
    2. 构建3D栅格数组表示环境
    3. 处理障碍物（墙体）数据
    4. 提供空间查询接口（有效性检查、邻域查询）
    """

    def __init__(self, map_path, resolution):
        """
        初始化3D栅格地图

        参数:
            map_path: str - world.json文件的路径
            resolution: float - 栅格分辨率（米），决定每个栅格单元的物理尺寸

        属性:
            resolution: 栅格分辨率
            free_space: 自由空间的值标记
            occupied_space: 障碍物空间的值标记
            b_min: 空间边界最小值 (x_min, y_min, z_min)
            b_max: 空间边界最大值 (x_max, y_max, z_max)
            width/height/depth: 栅格地图的尺寸
            map_data: 3D numpy数组，存储栅格状态
            json_world: 原始JSON数据
        """
        print(f"[DEBUG] GridMap3D.__init__ started with resolution={resolution}")

        self.resolution = resolution
        self.free_space = 0           # 自空间标记值
        self.occupied_space = 100     # 障碍物标记值

        # 从JSON文件加载地图数据
        print(f"[DEBUG] Loading map from: {map_path}")
        t0 = time.time()
        with open(map_path, 'r') as f:
            self.json_world = json.load(f)
        print(f"[DEBUG] Map loaded in {time.time()-t0:.3f}s")

        # 获取空域边界
        self.b_min = tuple(self.json_world["airspace"]["min"])
        self.b_max = tuple(self.json_world["airspace"]["max"])
        print(f"[DEBUG] Map bounds: min={self.b_min}, max={self.b_max}")

        # 计算栅格地图的维度（单位：栅格数）
        self.width = int((self.b_max[0] - self.b_min[0]) / resolution)   # X方向栅格数
        self.height = int((self.b_max[1] - self.b_min[1]) / resolution)  # Y方向栅格数
        self.depth = int((self.b_max[2] - self.b_min[2]) / resolution)   # Z方向栅格数

        print(f"[DEBUG] Grid dimensions: width={self.width}, height={self.height}, depth={self.depth}")
        print(f"[DEBUG] Total grid cells: {self.width * self.height * self.depth:,}")

        # 性能保护：如果栅格数过大，发出警告
        total_cells = self.width * self.height * self.depth
        if total_cells > 500000:
            print(f"[WARNING] Large grid size detected ({total_cells:,} cells). This may be slow.")

        # 初始化3D栅格数组，默认全部为自由空间
        # 数组形状为 (depth, height, width)，对应numpy的 (z, y, x) 索引顺序
        print(f"[DEBUG] Initializing map_data array...")
        t0 = time.time()
        self.map_data = np.full(
            (self.depth, self.height, self.width),
            self.free_space,
            dtype=np.int8
        )
        print(f"[DEBUG] map_data initialized in {time.time()-t0:.3f}s")

        # 将墙体障碍物插入到栅格地图中
        print(f"[DEBUG] Reading walls...")
        t0 = time.time()
        self.read_walls()
        print(f"[DEBUG] Walls processed in {time.time()-t0:.3f}s")
        print(f"[DEBUG] GridMap3D.__init__ completed")

    def read_walls(self):
        """
        将墙体定义转换为被占据的栅格单元

        该方法使用numpy向量化操作遍历JSON中的所有墙体定义，
        计算每个墙体所占据的栅格单元，并将其标记为障碍物。

        性能优化：使用numpy切片代替三重for循环
        """
        walls = self.json_world["walls"]
        print(f"[DEBUG] Processing {len(walls)} walls...")

        for i, wall in enumerate(walls):
            # 获取墙体的起始和结束坐标
            start = wall["plane"]["start"]
            stop = wall["plane"]["stop"]

            # 将物理坐标转换为栅格索引
            sx = int((start[0] - self.b_min[0]) / self.resolution)
            sy = int((start[1] - self.b_min[1]) / self.resolution)
            sz = int((start[2] - self.b_min[2]) / self.resolution)

            ex = int((stop[0] - self.b_min[0]) / self.resolution)
            ey = int((stop[1] - self.b_min[1]) / self.resolution)
            ez = int((stop[2] - self.b_min[2]) / self.resolution)

            # 确保索引顺序正确（从小到大）
            x_min, x_max = min(sx, ex), max(sx, ex) + 1
            y_min, y_max = min(sy, ey), max(sy, ey) + 1
            z_min, z_max = min(sz, ez), max(sz, ez) + 1

            # 边界检查
            x_min = max(0, x_min)
            y_min = max(0, y_min)
            z_min = max(0, z_min)
            x_max = min(self.width, x_max)
            y_max = min(self.height, y_max)
            z_max = min(self.depth, z_max)

            # 使用numpy向量化操作标记障碍物
            # 这比三重for循环快很多
            self.map_data[z_min:z_max, y_min:y_max, x_min:x_max] = self.occupied_space

        print(f"[DEBUG] All walls marked as occupied")

    def is_valid(self, index):
        """
        检查栅格单元是否在地图范围内且为自由空间

        参数:
            index: tuple - 栅格索引 (x, y, z)

        返回:
            bool - 如果栅格有效且为自由空间返回True，否则返回False
        """
        x, y, z = index
        return (
            0 <= x < self.width and
            0 <= y < self.height and
            0 <= z < self.depth and
            self.map_data[z, y, x] == self.free_space
        )

    def get_neighbors(self, index):
        """
        获取当前栅格单元的26连通邻域

        在3D空间中，每个栅格单元最多有26个邻居（包括对角线方向）。
        该方法返回所有有效且可通行的邻居索引。

        参数:
            index: tuple - 当前栅格索引 (x, y, z)

        返回:
            list - 有效邻居的索引列表
        """
        neighbors = []

        # 遍历3x3x3邻域（-1, 0, 1）的三个维度
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    # 跳过自身
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    # 计算邻居索引
                    neighbor = (
                        index[0] + dx,
                        index[1] + dy,
                        index[2] + dz
                    )

                    # 检查邻居是否有效
                    if self.is_valid(neighbor):
                        neighbors.append(neighbor)

        return neighbors
