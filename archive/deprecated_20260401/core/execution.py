"""
巡检执行模块

功能：模拟无人机沿路径执行巡检任务
"""

import numpy as np


class InspectionExecutor:
    """巡检执行器"""

    @staticmethod
    def find_closest_index(path_3d, current_pos):
        """
        找到路径中距离当前位置最近的点索引

        Args:
            path_3d: 3D路径点 [(x, y, z), ...]
            current_pos: 当前位置 (x, y)

        Returns:
            int: 最近点的索引
        """
        if not path_3d or len(path_3d) == 0:
            return 0

        min_dist = float('inf')
        closest_idx = 0

        current_x, current_y = current_pos

        for idx, point in enumerate(path_3d):
            point_x, point_y = point[0], point[1]
            dist = np.sqrt((current_x - point_x)**2 + (current_y - point_y)**2)

            if dist < min_dist:
                min_dist = dist
                closest_idx = idx

        return closest_idx

    @staticmethod
    def get_sub_path(path_3d, tasks, current_pos):
        """
        从当前位置截取剩余子路径和任务

        Args:
            path_3d: 完整3D路径 [(x, y, z), ...]
            tasks: 完整任务列表 [{"position": (x,y,z), ...}, ...]
            current_pos: 当前位置 (x, y)

        Returns:
            tuple: (sub_path, sub_tasks, start_index)
        """
        # 找到最近的路径点索引
        start_idx = InspectionExecutor.find_closest_index(path_3d, current_pos)

        # 截取子路径
        sub_path = path_3d[start_idx:]

        # 截取子任务（找到第一个未完成的任务）
        sub_tasks = []
        for task in tasks:
            task_pos = task["position"]
            # 检查任务位置是否在子路径中
            for i in range(start_idx, len(path_3d)):
                path_point = path_3d[i]
                if (abs(path_point[0] - task_pos[0]) < 1 and
                    abs(path_point[1] - task_pos[1]) < 1):
                    sub_tasks.append(task)
                    break

        return sub_path, sub_tasks, start_idx

    @staticmethod
    def simulate_execution(sub_path, sub_tasks):
        """
        模拟巡检执行过程

        Args:
            sub_path: 子路径 [(x, y, z), ...]
            sub_tasks: 子任务列表

        Returns:
            dict: 执行统计 {
                'total_steps': 总步数,
                'tasks_completed': 完成任务数,
                'total_distance': 总飞行距离
            }
        """
        print("=" * 70)
        print("开始模拟巡检执行")
        print("=" * 70)

        # 创建任务位置集合用于快速查找
        task_positions = set()
        for task in sub_tasks:
            pos = task["position"]
            # 使用四舍五入处理浮点数
            task_positions.add((round(pos[0]), round(pos[1]), round(pos[2])))

        total_steps = 0
        tasks_completed = 0
        total_distance = 0.0

        # 遍历子路径
        for i, point in enumerate(sub_path):
            x, y, z = point
            total_steps += 1

            # 计算累计飞行距离
            if i > 0:
                prev_point = sub_path[i - 1]
                dist = np.sqrt(
                    (x - prev_point[0])**2 +
                    (y - prev_point[1])**2 +
                    (z - prev_point[2])**2
                )
                total_distance += dist

            # 打印当前位置
            print(f"[执行] 步骤 {total_steps}: UAV位置 = ({x:.1f}, {y:.1f}, {z:.1f})")

            # 检查是否是任务点
            rounded_pos = (round(x), round(y), round(z))
            if rounded_pos in task_positions:
                tasks_completed += 1

                # 找到对应的任务
                for task in sub_tasks:
                    task_pos = task["position"]
                    if (round(task_pos[0]) == rounded_pos[0] and
                        round(task_pos[1]) == rounded_pos[1] and
                        round(task_pos[2]) == rounded_pos[2]):

                        print(f"  [任务] 执行拍照 - 相机角度: {task['camera_angle']}°")
                        print(f"  [任务] 任务ID: {task['id']}")
                        # 从集合中移除，避免重复执行
                        task_positions.remove(rounded_pos)
                        break

        print("=" * 70)
        print("巡检执行完成")
        print("=" * 70)
        print(f"总步骤: {total_steps}")
        print(f"完成任务: {tasks_completed}")
        print(f"飞行距离: {total_distance:.1f} 米")

        return {
            'total_steps': total_steps,
            'tasks_completed': tasks_completed,
            'total_distance': total_distance
        }


# 便捷函数
def find_closest_index(path_3d, current_pos):
    """找到路径中最近的点索引"""
    return InspectionExecutor.find_closest_index(path_3d, current_pos)


def get_sub_path(path_3d, tasks, current_pos):
    """获取从当前位置开始的子路径"""
    return InspectionExecutor.get_sub_path(path_3d, tasks, current_pos)


def simulate_execution(sub_path, sub_tasks):
    """模拟巡检执行"""
    return InspectionExecutor.simulate_execution(sub_path, sub_tasks)
