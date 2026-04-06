"""
巡检任务生成模块

功能：在路径上生成巡检任务点（拍照点）
"""

import numpy as np


class MissionGenerator:
    """巡检任务生成器"""

    @staticmethod
    def generate_inspection_tasks(path_3d, interval=10):
        """
        在路径上生成巡检任务点（拍照点）

        Args:
            path_3d: 3D路径点 [(x, y, z), ...] 或 numpy数组
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
        if path_3d is None or len(path_3d) < 1:
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

        return tasks
