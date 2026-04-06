"""
Mission Output 清洗脚本

功能：
1. 删除零长度 connect segment
2. 删除起点终点相同的冗余 segment
3. 删除 geometry 中的连续重复点
4. 重新导出清洗后的 mission_output.json
"""

import json
import numpy as np
from pathlib import Path


def clean_geometry(geometry):
    """删除 geometry 中的连续重复点"""
    if not geometry or len(geometry) < 2:
        return geometry

    cleaned = [geometry[0]]
    for point in geometry[1:]:
        # 检查是否与上一个点重复
        last = cleaned[-1]
        if isinstance(point, list):
            if isinstance(last, list):
                if abs(point[0] - last[0]) > 0.01 or abs(point[1] - last[1]) > 0.01:
                    cleaned.append(point)
            else:
                cleaned.append(point)
        else:
            if point != last:
                cleaned.append(point)

    return cleaned


def clean_segment(seg):
    """清洗单个 segment"""
    # 清洗 geometry_2d
    if 'geometry_2d' in seg and seg['geometry_2d']:
        seg['geometry_2d'] = clean_geometry(seg['geometry_2d'])

    # 清洗 geometry_3d
    if 'geometry_3d' in seg and seg['geometry_3d']:
        seg['geometry_3d'] = clean_geometry(seg['geometry_3d'])

    # 重新计算长度
    if seg['geometry_2d'] and len(seg['geometry_2d']) >= 2:
        length = 0.0
        for i in range(len(seg['geometry_2d']) - 1):
            p1 = np.array(seg['geometry_2d'][i])
            p2 = np.array(seg['geometry_2d'][i + 1])
            length += np.linalg.norm(p2 - p1)
        seg['length'] = round(length, 2)

    return seg


def should_remove_segment(seg):
    """判断是否应该删除该 segment"""
    # 删除零长度或接近零长度的 connect segment
    if seg.get('type') == 'connect':
        length = seg.get('length', 0)
        if length < 0.1:  # 零长度
            return True, 'zero_length'

        # 检查是否只有重复点
        geo = seg.get('geometry_2d', [])
        if len(geo) >= 2:
            first = geo[0]
            last = geo[-1]
            if isinstance(first, list) and isinstance(last, list):
                if abs(first[0] - last[0]) < 0.01 and abs(first[1] - last[1]) < 0.01:
                    # 起点终点相同，且中间没有有效路径
                    unique_points = set()
                    for pt in geo:
                        unique_points.add(tuple(pt))
                    if len(unique_points) <= 2:
                        return True, 'same_start_end'

    return False, None


def clean_mission_json(input_path, output_path):
    """清洗 mission JSON"""
    print("="*60)
    print("Mission Output 清洗")
    print("="*60)

    # 读取
    with open(input_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    segments = data.get('segments', [])
    original_count = len(segments)
    original_stats = data.get('statistics', {}).copy()

    print(f"\n原始: {original_count} 个 segments")

    # 清洗每个 segment
    cleaned_segments = []
    removed_reasons = []

    for i, seg in enumerate(segments):
        seg_id = seg.get('segment_id', 'unknown')

        # 先检查是否需要删除
        should_remove, reason = should_remove_segment(seg)
        if should_remove:
            removed_reasons.append(f"{seg_id}: {reason}")
            print(f"  [删除] {seg_id} - {reason}")
            continue

        # 清洗 segment
        cleaned_seg = clean_segment(seg)
        cleaned_segments.append(cleaned_seg)

        # 检查是否有变化
        orig_len = seg.get('length', 0)
        new_len = cleaned_seg.get('length', 0)
        if abs(orig_len - new_len) > 0.1:
            print(f"  [清洗] {seg_id}: {orig_len:.1f}px -> {new_len:.1f}px")

    # 更新 segments
    data['segments'] = cleaned_segments

    # 重新计算统计信息
    inspect_len = sum(s['length'] for s in cleaned_segments if s['type'] == 'inspect')
    connect_len = sum(s['length'] for s in cleaned_segments if s['type'] == 'connect')
    total_len = inspect_len + connect_len

    data['statistics']['total_length'] = round(total_len, 2)
    data['statistics']['inspect_length'] = round(inspect_len, 2)
    data['statistics']['connect_length'] = round(connect_len, 2)
    data['statistics']['num_segments'] = len(cleaned_segments)

    # 更新 visit_order（可能因为删除 segment 而变化）
    inspect_edges = [s['edge_id'] for s in cleaned_segments if s['type'] == 'inspect' and s.get('edge_id')]
    data['visit_order']['edge_visit_order'] = inspect_edges

    # 保存
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"\n清洗完成:")
    print(f"  删除: {original_count} -> {len(cleaned_segments)} segments")
    print(f"  原始统计: Total={original_stats.get('total_length', 0):.1f}px, "
          f"Inspect={original_stats.get('inspect_length', 0):.1f}px, "
          f"Connect={original_stats.get('connect_length', 0):.1f}px")
    print(f"  清洗统计: Total={total_len:.1f}px, "
          f"Inspect={inspect_len:.1f}px, "
          f"Connect={connect_len:.1f}px")

    if removed_reasons:
        print(f"\n删除的 segments:")
        for reason in removed_reasons:
            print(f"  - {reason}")

    print(f"\n输出: {output_path}")

    return data


if __name__ == "__main__":
    import sys
    sys.path.insert(0, '.')

    input_file = "result/latest/mission_output.json"
    output_file = "result/latest/mission_output.json"

    # 备份原始文件
    import shutil
    backup_file = "result/latest/mission_output_before_clean.json"
    shutil.copy(input_file, backup_file)
    print(f"备份原始文件: {backup_file}")

    # 执行清洗
    clean_mission_json(input_file, output_file)

    print("\n" + "="*60)
    print("重新生成可视化页面（仅更新数据，不重新规划）...")
    print("="*60)

    # 直接调用 HTML 生成函数，不重新运行规划
    from demo.generate_interactive_main_view import create_interactive_main_view

    create_interactive_main_view(
        map_image_path="data/test.png",
        mission_json_path="result/latest/mission_output.json",
        output_html_path="result/latest/main_view_interactive.html"
    )

    print("\n完成！")
    print(f"输出文件: result/latest/main_view_interactive.html")
    print(f"备份数据: {backup_file}")
