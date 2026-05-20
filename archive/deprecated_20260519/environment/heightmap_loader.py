"""
=====================================================
HeightMap-based 3D Map Generator (基于高度图的3D地图生成器)
=====================================================
功能说明:
    该模块实现了从灰度高度图生成3D栅格地图的功能。
    支持将真实地形数据转换为UAV路径规划环境。

类说明:
    HeightMap3D: 基于高度图的3D栅格地图类

主要功能:
    - 从PNG/JPG灰度图读取高度数据
    - 将像素值映射为物理高度（米）
    - 构建3D occupancy grid
    - 与GridMap3D接口兼容，可直接替换使用

=====================================================
使用示例:
=====================================================

from environment.heightmap_loader import HeightMap3D

# 方式1：直接从图片创建（推荐）
grid = HeightMap3D.from_image(
    image_path="heightmap.png",
    resolution=1.0,          # 栅格分辨率（米）
    flight_height=50,        # 飞行高度（米）
    ground_clearance=5       # 地面 clearance（米）
)

# 方式2：从numpy数组创建
import numpy as np
height_data = np.random.rand(100, 100) * 100  # 100x100, 0-100m
grid = HeightMap3D.from_array(
    height_array=height_data,
    resolution=1.0,
    flight_height=50,
    ground_clearance=5
)

# 之后正常使用
from planner import AStar3D
planner = AStar3D(grid)
path = planner.plan(start, goal)

=====================================================
高度图说明:
=====================================================

输入：灰度图（单通道）
- 像素值范围：0-255
- 0 = 黑色 = 最低高度
- 255 = 白色 = 最高高度

高度映射：
    height(m) = min_height + (pixel_value / 255) * (max_height - min_height)

障碍物判断：
    - z < height(x,y) - clearance → 障碍物
    - z >= height(x,y) + safety_margin → 自由空间

=====================================================
"""

import numpy as np
from PIL import Image
import os


class HeightMap3D:
    """
    基于高度图的3D栅格地图类

    该类从灰度高度图生成3D环境，适用于真实地形数据的路径规划。
    接口与GridMap3D兼容，可直接替换使用。

    核心原理：
        2D高度图 → 3D occupancy grid
        对于每个栅格(x,y,z)：
            - z < terrain_height(x,y) → 障碍物（地下）
            - z >= terrain_height(x,y) → 自由空间（空中）
    """

    def __init__(self, height_array, resolution=1.0, flight_height=20,
                 ground_clearance=1, safety_margin=1,
                 xy_bounds=None, min_elevation=0, max_elevation=100):
        """
        从高度数组初始化3D栅格地图

        参数:
            height_array: np.ndarray - 2D高度数组，shape=(H, W)
                        每个元素表示该位置的高度（米）
            resolution: float - 栅格分辨率（米），决定每个栅格的物理尺寸
            flight_height: float - 默认飞行高度（米）
                        用于计算空域上限
            ground_clearance: float - 地面间隙（米）
                        无人机至少离地高度
            safety_margin: float - 安全裕度（米）
                        额外增加的高度缓冲区
            xy_bounds: tuple - (x_min, y_min) 原点坐标（米）
                        如果为None，则默认为(0, 0)
            min_elevation: float - 最小高度值（米），用于归一化
            max_elevation: float - 最大高度值（米），用于归一化

        属性:
            resolution: 栅格分辨率（米）
            free_space: 自由空间标记值
            occupied_space: 障碍物标记值
            b_min: 空间边界最小值 (x_min, y_min, z_min)
            b_max: 空间边界最大值 (x_max, y_max, z_max)
            width/height/depth: 栅格地图尺寸
            map_data: 3D numpy数组，存储栅格状态
            height_map: 原始2D高度图
        """
        print(f"[DEBUG] HeightMap3D.__init__ started")
        print(f"[DEBUG] Input height_array shape: {height_array.shape}")

        # 基本参数
        self.resolution = resolution
        self.flight_height = flight_height
        self.ground_clearance = ground_clearance
        self.safety_margin = safety_margin

        # 栅格状态标记
        self.free_space = 0
        self.occupied_space = 100

        # 保存高度图
        self.height_map = height_array.astype(np.float32)

        # 地图尺寸（像素）
        self.img_height, self.img_width = height_array.shape

        # 设置XY平面原点
        if xy_bounds is None:
            self.x_origin = 0.0
            self.y_origin = 0.0
        else:
            self.x_origin, self.y_origin = xy_bounds

        # 计算XY平面边界
        self.b_min = (self.x_origin, self.y_origin, min_elevation)
        self.b_max = (
            self.x_origin + self.img_width * resolution,
            self.y_origin + self.img_height * resolution,
            max(flight_height, np.max(height_array) + ground_clearance)
        )

        # 计算栅格维度
        self.width = self.img_width
        self.height = self.img_height
        self.depth = int((self.b_max[2] - self.b_min[2]) / resolution)

        print(f"[DEBUG] Grid dimensions: width={self.width}, height={self.height}, depth={self.depth}")
        print(f"[DEBUG] Total grid cells: {self.width * self.height * self.depth:,}")
        print(f"[DEBUG] Bounds: min={self.b_min}, max={self.b_max}")

        # 性能警告
        total_cells = self.width * self.height * self.depth
        if total_cells > 500000:
            print(f"[WARNING] Large grid size ({total_cells:,} cells). Consider increasing resolution.")

        # 构建栅格数据
        print(f"[DEBUG] Building 3D occupancy grid...")
        self._build_grid()

        print(f"[DEBUG] HeightMap3D.__init__ completed")

    def _build_grid(self):
        """
        构建3D occupancy grid

        将2D高度图转换为3D栅格数组：
        - 对于每个位置，z < height(x,y) 的栅格标记为障碍物
        - z >= height(x,y) 的栅格标记为自由空间
        """
        # 初始化为自由空间
        self.map_data = np.full(
            (self.depth, self.height, self.width),
            self.free_space,
            dtype=np.int8
        )

        # 计算每个栅格对应的高度层
        # z_grid 对应的实际高度 = b_min[2] + z * resolution
        z_levels = self.b_min[2] + np.arange(self.depth) * self.resolution

        # 对每个高度层进行处理
        for z_idx, z_level in enumerate(z_levels):
            # 计算该高度层对应的地形障碍物
            # 只阻挡地面本身（+1米作为地面表面），上方空间全部可飞
            obstacle_mask = z_level < (self.height_map + 1)

            # 标记障碍物
            # obstacle_mask shape: (height, width)
            # map_data[z_idx] shape: (height, width)
            self.map_data[z_idx][obstacle_mask] = self.occupied_space

        # 标记最低层以下全部为障碍物（安全起见）
        ground_level = int((np.min(self.height_map) - self.ground_clearance - self.b_min[2]) / self.resolution)
        if ground_level > 0:
            self.map_data[:ground_level, :, :] = self.occupied_space

        # 统计信息
        total_cells = self.map_data.size
        occupied_cells = np.sum(self.map_data == self.occupied_space)
        free_cells = total_cells - occupied_cells
        print(f"[DEBUG] Grid statistics:")
        print(f"[DEBUG]   - Total cells: {total_cells:,}")
        print(f"[DEBUG]   - Free cells: {free_cells:,} ({100*free_cells/total_cells:.1f}%)")
        print(f"[DEBUG]   - Occupied cells: {occupied_cells:,} ({100*occupied_cells/total_cells:.1f}%)")

    @classmethod
    def from_image(cls, image_path, resolution=1.0, flight_height=20,
                   ground_clearance=1, safety_margin=1,
                   xy_bounds=None, min_elevation=0, max_elevation=100):
        """
        从灰度图创建HeightMap3D实例

        参数:
            image_path: str - 灰度图文件路径（支持PNG、JPG等）
            resolution: float - 栅格分辨率（米）
            flight_height: float - 飞行高度（米）
            ground_clearance: float - 地面间隙（米）
            safety_margin: float - 安全裕度（米）
            xy_bounds: tuple - (x_min, y_min) 原点坐标（米）
            min_elevation: float - 最小高度（米），对应像素值0
            max_elevation: float - 最大高度（米），对应像素值255

        返回:
            HeightMap3D实例

        图片要求:
            - 灰度图（单通道）
            - 像素值 0-255
            - 0 = 黑色 = min_elevation
            - 255 = 白色 = max_elevation

        示例:
            >>> grid = HeightMap3D.from_image(
            ...     "mountain.png",
            ...     resolution=2.0,
            ...     min_elevation=0,
            ...     max_elevation=500
            ... )
        """
        print(f"[DEBUG] Loading heightmap from: {image_path}")

        # 检查文件存在性
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image file not found: {image_path}")

        # 加载图片
        img = Image.open(image_path)

        # 转换为灰度图（如果不是）
        if img.mode != 'L':
            print(f"[DEBUG] Converting image from {img.mode} to grayscale")
            img = img.convert('L')

        # 转换为numpy数组
        img_array = np.array(img, dtype=np.float32)

        print(f"[DEBUG] Image loaded: shape={img_array.shape}, mode={img.mode}")
        print(f"[DEBUG] Pixel value range: [{img_array.min()}, {img_array.max()}]")

        # 归一化到 [0, 1]
        img_array = img_array / 255.0

        # 映射到实际高度范围
        height_array = min_elevation + img_array * (max_elevation - min_elevation)

        print(f"[DEBUG] Height range: [{height_array.min():.1f}, {height_array.max():.1f}] meters")

        # 使用height_array创建实例
        return cls(
            height_array=height_array,
            resolution=resolution,
            flight_height=flight_height,
            ground_clearance=ground_clearance,
            safety_margin=safety_margin,
            xy_bounds=xy_bounds,
            min_elevation=min_elevation,
            max_elevation=max_elevation
        )

    @classmethod
    def from_array(cls, height_array, resolution=1.0, flight_height=20,
                   ground_clearance=1, safety_margin=1,
                   xy_bounds=None, min_elevation=None, max_elevation=None):
        """
        从numpy数组创建HeightMap3D实例

        参数:
            height_array: np.ndarray - 2D高度数组（米）
            resolution: float - 栅格分辨率（米）
            flight_height: float - 飞行高度（米）
            ground_clearance: float - 地面间隙（米）
            safety_margin: float - 安全裕度（米）
            xy_bounds: tuple - (x_min, y_min) 原点坐标（米）
            min_elevation: float - 最小高度（米），如果为None则使用数组最小值
            max_elevation: float - 最大高度（米），如果为None则使用数组最大值

        返回:
            HeightMap3D实例

        示例:
            >>> import numpy as np
            >>> heights = np.random.rand(100, 100) * 100  # 0-100米
            >>> grid = HeightMap3D.from_array(heights, resolution=1.0)
        """
        if min_elevation is None:
            min_elevation = np.min(height_array)
        if max_elevation is None:
            max_elevation = np.max(height_array)

        return cls(
            height_array=np.array(height_array, dtype=np.float32),
            resolution=resolution,
            flight_height=flight_height,
            ground_clearance=ground_clearance,
            safety_margin=safety_margin,
            xy_bounds=xy_bounds,
            min_elevation=min_elevation,
            max_elevation=max_elevation
        )

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

        参数:
            index: tuple - 当前栅格索引 (x, y, z)

        返回:
            list - 有效邻居的索引列表
        """
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    neighbor = (
                        index[0] + dx,
                        index[1] + dy,
                        index[2] + dz
                    )

                    if self.is_valid(neighbor):
                        neighbors.append(neighbor)

        return neighbors

    def get_height_at(self, x, y):
        """
        获取指定位置的地形高度

        参数:
            x: int or float - x坐标（栅格索引或米）
            y: int or float - y坐标（栅格索引或米）

        返回:
            float - 该位置的地形高度（米）
        """
        # 如果输入是浮点数，转换为栅格索引
        if isinstance(x, float):
            x = int((x - self.x_origin) / self.resolution)
        if isinstance(y, float):
            y = int((y - self.y_origin) / self.resolution)

        # 边界检查
        x = max(0, min(x, self.width - 1))
        y = max(0, min(y, self.height - 1))

        return self.height_map[y, x]

    def is_above_ground(self, x, y, z, clearance=None):
        """
        检查指定位置是否在地面上方

        参数:
            x: float - x坐标（米）
            y: float - y坐标（米）
            z: float - z坐标（米）
            clearance: float - 所需间隙（米），如果为None则使用ground_clearance

        返回:
            bool - 如果位置在地面+clearance上方返回True
        """
        if clearance is None:
            clearance = self.ground_clearance

        ground_height = self.get_height_at(x, y)
        return z >= (ground_height + clearance)

    def get_safe_flight_height(self, x, y, min_clearance=None):
        """
        获取指定位置的安全飞行高度

        参数:
            x: float - x坐标（米）
            y: float - y坐标（米）
            min_clearance: float - 最小间隙（米），如果为None则使用ground_clearance

        返回:
            float - 安全飞行高度（米）
        """
        if min_clearance is None:
            min_clearance = self.ground_clearance

        ground_height = self.get_height_at(x, y)
        return ground_height + min_clearance + self.safety_margin


def create_sample_heightmap(output_path="sample_heightmap.png", size=256):
    """
    创建一个示例高度图用于测试

    生成一个包含山丘和山谷的合成地形

    参数:
        output_path: str - 输出文件路径
        size: int - 图像尺寸（像素）
    """
    import matplotlib.pyplot as plt

    print(f"[DEBUG] Creating sample heightmap: {output_path}")

    # 创建坐标网格
    x = np.linspace(-5, 5, size)
    y = np.linspace(-5, 5, size)
    X, Y = np.meshgrid(x, y)

    # 生成合成地形（高斯峰组合）
    Z = (
        50 * np.exp(-(X**2 + Y**2) / 10) +  # 主峰
        30 * np.exp(-((X-2)**2 + (Y-1)**2) / 5) +  # 次峰1
        20 * np.exp(-((X+1)**2 + (Y+2)**2) / 8)    # 次峰2
    )

    # 添加一些噪声
    Z += np.random.normal(0, 2, Z.shape)

    # 确保非负
    Z = np.maximum(Z, 0)

    # 归一化到 0-255
    Z_normalized = ((Z - Z.min()) / (Z.max() - Z.min()) * 255).astype(np.uint8)

    # 保存为图片
    img = Image.fromarray(Z_normalized, mode='L')
    img.save(output_path)

    print(f"[DEBUG] Sample heightmap saved to: {output_path}")
    print(f"[DEBUG] Height range: [{Z.min():.1f}, {Z.max():.1f}] meters")

    return Z


# =====================================================
# 便捷函数
# =====================================================

def load_heightmap(image_path, **kwargs):
    """
    便捷函数：从图片加载高度图

    参数:
        image_path: str - 图片路径
        **kwargs: 传递给HeightMap3D.from_image的参数

    返回:
        HeightMap3D实例

    示例:
        >>> grid = load_heightmap("terrain.png", resolution=2.0, flight_height=100)
    """
    return HeightMap3D.from_image(image_path, **kwargs)


def load_from_array(height_array, **kwargs):
    """
    便捷函数：从numpy数组加载高度图

    参数:
        height_array: np.ndarray - 高度数组
        **kwargs: 传递给HeightMap3D.from_array的参数

    返回:
        HeightMap3D实例

    示例:
        >>> heights = np.random.rand(100, 100) * 50
        >>> grid = load_from_array(heights, resolution=1.0)
    """
    return HeightMap3D.from_array(height_array, **kwargs)


__all__ = [
    'HeightMap3D',
    'load_heightmap',
    'load_from_array',
    'create_sample_heightmap'
]
