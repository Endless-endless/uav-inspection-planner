"""
=====================================================
CSV Logger Module (CSV日志记录模块)
=====================================================
功能说明:
    该模块提供了将实验结果保存为CSV文件的功能。
    支持单次运行结果记录和批量实验数据记录。

主要功能:
    - 保存单次实验结果
    - 保存批量实验数据
    - 自动创建结果目录
    - 带时间戳的记录
=====================================================
"""

import os
import csv
from typing import Dict, List, Any, Optional
from datetime import datetime


# 获取项目根目录
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")


def ensure_results_dir():
    """确保结果目录存在"""
    os.makedirs(RESULTS_DIR, exist_ok=True)


def get_results_path(filename: str) -> str:
    """
    获取结果文件的完整路径

    参数:
        filename: str - 文件名

    返回:
        str - 结果文件的完整路径
    """
    ensure_results_dir()
    return os.path.join(RESULTS_DIR, filename)


def save_single_result(filename: str, data: Dict[str, Any], mode: str = 'w'):
    """
    保存单次实验结果到CSV文件

    参数:
        filename: str - 文件名
        data: Dict[str, Any] - 实验数据字典
        mode: str - 写入模式，'w'覆盖或'a'追加
    """
    filepath = get_results_path(filename)

    # 确定文件是否已存在，以决定是否写入表头
    write_header = not os.path.exists(filepath) or mode == 'w'

    with open(filepath, mode, newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=data.keys())
        if write_header:
            writer.writeheader()
        writer.writerow(data)


def save_batch_results(filename: str, data_list: List[Dict[str, Any]],
                       mode: str = 'w'):
    """
    保存批量实验结果到CSV文件

    参数:
        filename: str - 文件名
        data_list: List[Dict[str, Any]] - 实验数据列表
        mode: str - 写入模式，'w'覆盖或'a'追加
    """
    if not data_list:
        return

    filepath = get_results_path(filename)

    # 确定文件是否已存在，以决定是否写入表头
    write_header = not os.path.exists(filepath) or mode == 'w'

    with open(filepath, mode, newline='', encoding='utf-8') as f:
        if data_list:
            writer = csv.DictWriter(f, fieldnames=data_list[0].keys())
            if write_header:
                writer.writeheader()
            writer.writerows(data_list)


def append_result(filename: str, data: Dict[str, Any]):
    """
    追加实验结果到CSV文件

    参数:
        filename: str - 文件名
        data: Dict[str, Any] - 实验数据字典
    """
    save_single_result(filename, data, mode='a')


def load_results(filename: str) -> List[Dict[str, Any]]:
    """
    从CSV文件加载实验结果

    参数:
        filename: str - 文件名

    返回:
        List[Dict[str, Any]] - 实验数据列表
    """
    filepath = get_results_path(filename)

    if not os.path.exists(filepath):
        return []

    results = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            results.append(dict(row))

    return results


class CSVLogger:
    """
    CSV日志记录器类

    提供更便捷的接口来记录和管理实验结果
    """

    def __init__(self, filename: str, auto_timestamp: bool = False):
        """
        初始化CSV日志记录器

        参数:
            filename: str - 文件名
            auto_timestamp: bool - 是否自动添加时间戳列
        """
        self.filename = filename
        self.auto_timestamp = auto_timestamp
        self._buffer = []

    def log(self, data: Dict[str, Any], append: bool = True):
        """
        记录一条日志

        参数:
            data: Dict[str, Any] - 日志数据
            append: bool - 是否追加到文件
        """
        if self.auto_timestamp:
            data = data.copy()
            data['timestamp'] = datetime.now().isoformat()

        if append:
            append_result(self.filename, data)
        else:
            self._buffer.append(data)

    def log_batch(self, data_list: List[Dict[str, Any]], append: bool = True):
        """
        批量记录日志

        参数:
            data_list: List[Dict[str, Any]] - 日志数据列表
            append: bool - 是否追加到文件
        """
        if self.auto_timestamp:
            timestamp = datetime.now().isoformat()
            data_list = [{**d, 'timestamp': timestamp} for d in data_list]

        if append:
            save_batch_results(self.filename, data_list, mode='a')
        else:
            self._buffer.extend(data_list)

    def save(self, mode: str = 'w'):
        """
        保存缓冲区中的数据

        参数:
            mode: str - 写入模式
        """
        if self._buffer:
            save_batch_results(self.filename, self._buffer, mode=mode)
            self._buffer.clear()

    def clear(self):
        """清空缓冲区"""
        self._buffer.clear()

    def load(self) -> List[Dict[str, Any]]:
        """
        加载所有日志

        返回:
            List[Dict[str, Any]] - 日志数据列表
        """
        return load_results(self.filename)
