"""
极简起点驱动重规划服务

功能：
- 提供 /replan 接口，接收起点坐标
- 在Python侧重新生成完整任务
- 返回新mission数据
- 自动与基线比较，防止退化

启动：
    python scripts/replan_service.py

使用：
    POST http://localhost:8000/replan
    {"x": 100, "y": 200}
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import json
import sys
import os
from pathlib import Path
import shutil
from typing import Optional, Dict, Any

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

app = FastAPI(title="UAV Replan Service", version="1.0.0")

# 允许跨域（方便前端调用）
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class ReplanRequest(BaseModel):
    """重规划请求"""
    x: float
    y: float


class ReplanResponse(BaseModel):
    """重规划响应"""
    success: bool
    message: str
    mission_data: Optional[Dict[str, Any]] = None
    comparison: Optional[Dict[str, Any]] = None
    is_adopted: bool = False  # 是否被采纳（优于基线）


# 基线数据缓存
BASELINE_DATA = None
BASELINE_PATH = Path("result/latest/mission_output.json")


def load_baseline():
    """加载基线数据"""
    global BASELINE_DATA
    if BASELINE_PATH.exists():
        with open(BASELINE_PATH, 'r', encoding='utf-8') as f:
            BASELINE_DATA = json.load(f)
        return BASELINE_DATA
    return None


def compare_with_baseline(new_mission_data: Dict) -> Dict:
    """
    与基线比较

    比较优先级：
    1. connect_length（一级，最重要）
    2. connect 段数（二级）
    3. total_length（三级）

    Returns:
        比较结果字典
    """
    if BASELINE_DATA is None:
        return {
            "has_baseline": False,
            "is_better": True,
            "reason": "无基线数据，采用新方案"
        }

    baseline_stats = BASELINE_DATA.get('statistics', {})
    new_stats = new_mission_data.get('statistics', {})

    baseline_connect = baseline_stats.get('connect_length', 0)
    new_connect = new_stats.get('connect_length', 0)

    baseline_total = baseline_stats.get('total_length', 0)
    new_total = new_stats.get('total_length', 0)

    baseline_segments = baseline_stats.get('num_segments', 0)
    new_segments = new_stats.get('num_segments', 0)

    baseline_connect_count = sum(1 for s in BASELINE_DATA.get('segments', [])
                                 if s.get('type') == 'connect')
    new_connect_count = sum(1 for s in new_mission_data.get('segments', [])
                            if s.get('type') == 'connect')

    # 一级：比较 connect_length
    # 允许 5% 的容差，避免微小差异导致频繁替换
    connect_diff = new_connect - baseline_connect
    connect_diff_ratio = connect_diff / baseline_connect if baseline_connect > 0 else 0

    if abs(connect_diff_ratio) < 0.05:  # 差异小于5%，认为相近
        # 二级：比较 connect 段数
        if new_connect_count < baseline_connect_count:
            return {
                "has_baseline": True,
                "is_better": True,
                "reason": "connect长度相近，但段数更少",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }
        elif new_connect_count > baseline_connect_count:
            return {
                "has_baseline": True,
                "is_better": False,
                "reason": "connect长度相近，但段数更多",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }

        # 三级：比较 total_length
        total_diff = new_total - baseline_total
        if total_diff < 0:
            return {
                "has_baseline": True,
                "is_better": True,
                "reason": "connect和段数相近，但总长度更短",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }
        else:
            return {
                "has_baseline": True,
                "is_better": False,
                "reason": "connect和段数相近，但总长度更长",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }

    elif connect_diff_ratio > 0.05:  # connect长度明显增加（>5%）
        return {
            "has_baseline": True,
            "is_better": False,
            "reason": f"connect长度明显增加 ({connect_diff_ratio*100:.1f}%)，拒绝采用",
            "baseline": {
                "connect_length": baseline_connect,
                "total_length": baseline_total,
                "connect_count": baseline_connect_count,
                "segment_count": baseline_segments
            },
            "new": {
                "connect_length": new_connect,
                "total_length": new_total,
                "connect_count": new_connect_count,
                "segment_count": new_segments
            }
        }

    else:  # connect长度明显减少（>5%）
        return {
            "has_baseline": True,
            "is_better": True,
            "reason": f"connect长度明显减少 ({abs(connect_diff_ratio)*100:.1f}%)",
            "baseline": {
                "connect_length": baseline_connect,
                "total_length": baseline_total,
                "connect_count": baseline_connect_count,
                "segment_count": baseline_segments
            },
            "new": {
                "connect_length": new_connect,
                "total_length": new_total,
                "connect_count": new_connect_count,
                "segment_count": new_segments
            }
        }


@app.on_event("startup")
async def startup_event():
    """启动时加载基线数据"""
    load_baseline()
    print("="*60)
    print("UAV 重规划服务已启动")
    print("="*60)
    if BASELINE_DATA:
        stats = BASELINE_DATA.get('statistics', {})
        print(f"基线数据已加载:")
        print(f"  Total: {stats.get('total_length', 0):.1f}px")
        print(f"  Inspect: {stats.get('inspect_length', 0):.1f}px")
        print(f"  Connect: {stats.get('connect_length', 0):.1f}px")
    else:
        print("警告：未找到基线数据")


@app.get("/")
async def root():
    """根路径"""
    return {
        "service": "UAV Replan Service",
        "version": "1.0.0",
        "endpoints": {
            "/replan": "POST - 起点驱动的任务重规划"
        }
    }


@app.get("/health")
async def health():
    """健康检查"""
    return {"status": "ok", "baseline_loaded": BASELINE_DATA is not None}


@app.post("/replan", response_model=ReplanResponse)
async def replan(request: ReplanRequest):
    """
    起点驱动的任务重规划

    Args:
        request: 包含起点坐标的请求

    Returns:
        重规划结果
    """
    try:
        print(f"\n[重规划请求] 起点: ({request.x}, {request.y})")

        # 导入重规划核心模块
        from core.start_driven_planner import plan_from_start_point

        # 执行重规划
        new_mission_data = plan_from_start_point(
            start_x=request.x,
            start_y=request.y,
            baseline_data=BASELINE_DATA
        )

        if new_mission_data is None:
            return ReplanResponse(
                success=False,
                message="重规划失败",
                is_adopted=False
            )

        # ========== 判断是否为用户指定起点的重规划 ==========
        # /replan 接口本身就是用户显式输入起点坐标触发的
        # 这种情况下应该直接采用新方案，因为约束条件已经变化（起点不同）
        is_user_specified_start = True  # /replan 接口总是用户指定起点

        if is_user_specified_start:
            # 用户指定起点：直接采用新方案，跳过基线劣化判断
            print(f"[接受] 用户指定起点 ({request.x}, {request.y})，采用真实起点驱动方案，跳过基线劣化判断")

            # 备份旧数据
            backup_path = Path("result/latest/mission_output_before_replan.json")
            if BASELINE_PATH.exists():
                shutil.copy(BASELINE_PATH, backup_path)
                print(f"  已备份旧基线: {backup_path}")

            # 保存新数据
            with open(BASELINE_PATH, 'w', encoding='utf-8') as f:
                json.dump(new_mission_data, f, indent=2, ensure_ascii=False)
            print(f"  已更新基线: {BASELINE_PATH}")

            # 重新生成HTML
            from demo.generate_interactive_main_view import create_interactive_main_view
            create_interactive_main_view(
                map_image_path="data/test.png",
                mission_json_path=str(BASELINE_PATH),
                output_html_path="result/latest/main_view_interactive.html"
            )
            print(f"  已更新可视化: result/latest/main_view_interactive.html")

            # 更新基线缓存
            load_baseline()

            # 生成统计摘要
            new_stats = new_mission_data.get('statistics', {})
            summary = (f"Total={new_stats.get('total_length', 0):.0f}px, "
                      f"Inspect={new_stats.get('inspect_length', 0):.0f}px, "
                      f"Connect={new_stats.get('connect_length', 0):.0f}px")

            message = f"重规划成功并已采纳（起点驱动）: {summary}"
            is_adopted = True
            comparison = None

        else:
            # 非用户指定起点：保留原有的基线比较逻辑
            comparison = compare_with_baseline(new_mission_data)
            is_adopted = comparison["is_better"]

            # 如果优于基线，保存并重新生成HTML
            if is_adopted:
                print(f"[采纳] 新方案优于基线: {comparison['reason']}")

                # 备份旧数据
                backup_path = Path("result/latest/mission_output_before_replan.json")
                if BASELINE_PATH.exists():
                    shutil.copy(BASELINE_PATH, backup_path)

                # 保存新数据
                with open(BASELINE_PATH, 'w', encoding='utf-8') as f:
                    json.dump(new_mission_data, f, indent=2, ensure_ascii=False)

                # 重新生成HTML
                from demo.generate_interactive_main_view import create_interactive_main_view
                create_interactive_main_view(
                    map_image_path="data/test.png",
                    mission_json_path=str(BASELINE_PATH),
                    output_html_path="result/latest/main_view_interactive.html"
                )

                # 更新基线缓存
                load_baseline()

                message = f"重规划成功并已采纳: {comparison['reason']}"
            else:
                print(f"[拒绝] 新方案劣于基线: {comparison['reason']}")
                message = f"重规划完成但未采纳: {comparison['reason']}"

        return ReplanResponse(
            success=True,
            message=message,
            mission_data=new_mission_data if is_adopted else None,
            comparison=comparison,
            is_adopted=is_adopted
        )

    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/baseline")
async def get_baseline():
    """获取当前基线数据"""
    if BASELINE_DATA is None:
        raise HTTPException(status_code=404, detail="基线数据未找到")
    return BASELINE_DATA


def main():
    """启动服务"""
    print("="*70)
    print("启动 UAV 重规划服务...")
    print("="*70)
    print("访问地址: http://localhost:8000")
    print("API文档: http://localhost:8000/docs")
    print("="*70)

    uvicorn.run(
        app,
        host="127.0.0.1",
        port=8000,
        log_level="info"
    )


if __name__ == "__main__":
    main()
