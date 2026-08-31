# UAV Powerline Inspection Mission Planning System

面向输电线路巡检的任务规划与感知集成研究原型。

本项目把拓扑感知任务规划、起终点重规划、浏览器 Dashboard、视频目标识别和图片缺陷检测串联为一个可演示的研究系统。主系统运行在 8001，并通过 HTTP 编排两个独立服务：8002 视频识别和 8003 缺陷检测。

> 本项目关注高层巡检任务规划和感知服务集成，不是生产飞控系统、实机自主控制系统、MPC 系统或安全关键工业平台。

## 1. Overview

当前原型提供：

- 基于图像和拓扑关系的输电线路 Mission 生成；
- inspect / connect 路径段及访问顺序展示；
- 用户指定起点、终点后的 Mission Replan；
- 普通 Mission A 与 Replan Mission B 的运行时身份绑定；
- 视频异步目标识别、关键帧下载和逐帧缺陷检测；
- 由 8001 提供的受控缺陷标注图代理；
- Mission 仿真回放与 Perception workflow 状态展示。

感知处理进度和飞行回放进度相互独立：感知可以达到 100%，同时航线回放仍处于较早位置。

## 2. Demo

仓库中的规划展示示例：

<p align="center">
  <img src="media/demo_main.png" width="850" alt="UAV powerline inspection mission planning example">
</p>

当前 Web Dashboard 还可以现场展示 Replan、视频识别、关键帧、缺陷结果和标注图片。视频输入需由使用者自行准备；仓库不提交大型 demo 视频、模型权重或用户媒体。

## 3. System Architecture

~~~mermaid
flowchart TD
    Browser[Browser Dashboard] --> Main[8001 Main System]
    Main --> Planning[Mission Planning]
    Main --> Replan[Start / End Replanning]
    Main --> Registry[RuntimeMissionRegistry]
    Main --> Orchestrator[Perception Orchestrator]
    Main --> Proxy[Defect Media Proxy]
    Orchestrator --> Video[8002 Video Recognition]
    Video -->|target frames| Orchestrator
    Orchestrator --> Defect[8003 Defect Detection]
    Defect -->|structured detections| Orchestrator
    Proxy -->|controlled image request| Defect
    Main --> Browser
~~~

8001 是唯一 orchestrator：

1. 接收浏览器上传的视频和当前 Mission identity；
2. 向 8002 创建异步视频识别任务并轮询；
3. 下载 8002 返回的目标关键帧；
4. 将关键帧和业务 identity 发送给 8003；
5. 汇总检测结果并通过 8001 返回浏览器；
6. 将 8003 的标注图片路径转换为受控的 8001 media proxy URL。

8002 不直接调用 8003。

## 4. Mission Flow

普通规划：

~~~text
Mission A
  → Dashboard A
  → RuntimeMissionRegistry(A)
  → MissionStore(A identity)
  → Perception workflow(A snapshot)
~~~

起终点重规划：

~~~text
Mission A
  → Start / End Replan
  → Mission B
  → Dashboard B
  → RuntimeMissionRegistry(B)
  → MissionStore(B identity)
  → Perception workflow(B snapshot)
~~~

Mission B 不通过覆盖 result/latest/mission_output.json 来冒充当前 Mission。浏览器把当前 Dashboard identity 随 perception 请求提交，后端从进程内 registry 精确读取相同 snapshot。

当前封版版本不持久化导出 Replan Mission B。Dashboard 显示 B 时，任务文件导出入口会被禁用，避免错误下载基线 Mission A。

## 5. Runtime Mission Identity

每个运行时 Mission snapshot 使用：

- mission_id：可关联的 Mission 标识；
- mission_sha256：canonical Mission JSON 的 SHA-256。

浏览器当前 Mission、Dashboard metadata、MissionStore 和 perception workflow 必须绑定相同的 mission_id / mission_sha256。巡检点 authority 来自当前 Mission 的 inspection_points[].point_id，前端不补零、不重编号，也不使用模糊匹配。

## 6. Repository Structure

~~~text
.
├── app.py                  # 8001 FastAPI 主入口
├── config/                 # 主系统和外部感知服务配置
├── core/                   # 拓扑、任务和 Mission 核心模型
├── planner/                # 规划、重规划和 Dashboard 构建
├── perception/             # HTTP clients、orchestrator、registry、API
├── web/                    # Dashboard HTML、CSS 和原生 JavaScript
├── tests/                  # unit、integration 和 frontend tests
├── demo/                   # 规划与可视化示例
├── scripts/                # 辅助脚本
├── data/                   # 示例输入和数据集配置
├── visualization/          # Dashboard/规划可视化工具
└── result/                 # 本地生成结果（Git ignored）
~~~

## 7. Prerequisites

- Python 3.9 或兼容版本；
- 主系统依赖：requirements.txt；
- 执行 Python 自动测试时额外需要 pytest（当前 requirements.txt 不包含开发测试依赖）；
- 三个服务分别使用独立 Python/Conda 环境；
- 一个现代浏览器。

| 模块 | 环境示例 | 监听地址 |
|---|---|---|
| 主系统 | uav_planning | 127.0.0.1:8001 |
| 视频识别服务 | yolo | 127.0.0.1:8002 |
| 缺陷检测服务 | power | 127.0.0.1:8003 |

三个项目依赖不同，不建议强行合并 Conda environment。requirements.txt 只描述本主仓库的依赖，不替代两个外部服务各自的环境配置。

## 8. Configuration

config/perception.py 支持以下进程环境变量：

| 环境变量 | 默认值 / 作用 |
|---|---|
| VIDEO_RECOGNITION_BASE_URL | http://127.0.0.1:8002 |
| DEFECT_DETECTION_BASE_URL | http://127.0.0.1:8003 |
| PERCEPTION_HTTP_TIMEOUT_SECONDS | 上游 HTTP timeout，默认 30 秒 |
| VIDEO_POLL_INTERVAL_SECONDS | 视频任务轮询间隔，默认 1 秒 |
| VIDEO_POLL_DEADLINE_SECONDS | 视频任务总等待期限，默认 600 秒 |
| MAX_FRAME_DOWNLOAD_BYTES | 单张关键帧下载上限，默认 20 MiB |

Base URL 由进程配置，不允许普通 API 请求动态覆盖。

## 9. Quick Start

安装主系统依赖：

~~~bash
conda activate uav_planning
python -m pip install -r requirements.txt
~~~

建议按依赖服务到主系统的顺序启动。

Terminal 1，在 defect detection service 项目目录运行：

~~~bash
conda activate power
python -m uvicorn server:app --host 127.0.0.1 --port 8003
~~~

Terminal 2，在 video recognition service 项目目录运行：

~~~bash
conda activate yolo
python -m uvicorn app:app --host 127.0.0.1 --port 8002
~~~

Terminal 3，在本主仓库根目录运行：

~~~bash
conda activate uav_planning
python app.py
~~~

健康检查：

~~~text
8003: http://127.0.0.1:8003/health
8002: http://127.0.0.1:8002/api/v1/health
8001: http://127.0.0.1:8001/debug/ping
~~~

然后打开 http://127.0.0.1:8001。

为保持固定演示地址，应先确认 8001 未被其他程序占用。直接运行 app.py 时，如果端口被占用，程序会寻找后续可用端口并在终端打印实际 URL。

## 10. Web Demo

一套约 3～5 分钟的展示流程：

1. 启动并检查 8003、8002、8001；
2. 打开 Dashboard，点击“生成任务”得到 Mission A；
3. 展示巡检点、巡检路径、连接路径和访问顺序；
4. 短暂启动 playback，展示 Mission 仿真回放；
5. 修改 start/end，执行 Replan 得到 Mission B；
6. 展示起终点和路径变化；
7. 打开“目标识别”Tab；
8. 从下拉框选择当前 Mission 的 inspection point；
9. 选择兼容的视频文件并点击“开始识别”；
10. 展示排队、视频识别和缺陷检测状态；
11. 打开“识别与缺陷结果”；
12. 展示目标关键帧、缺陷类别、置信度和检测数量；
13. 通过 8001 media proxy 查看标注图片。

感知工作流进度与仿真航线回放进度独立。Replan B 当前不提供 Mission 文件导出。

## 11. API Summary

### 8001 Main System

~~~text
POST /api/plan
POST /api/v1/perception/jobs
GET  /api/v1/perception/jobs/{workflow_job_id}
GET  /api/v1/perception/jobs/{workflow_job_id}/result
GET  /api/v1/perception/media/defect/{resource_id}
GET  /debug/ping
~~~

### 8002 Video Recognition

~~~text
GET  /api/v1/health
POST /api/v1/video-recognition/jobs
GET  /api/v1/video-recognition/jobs/{job_id}
GET  /api/v1/video-recognition/media/{resource_id}
~~~

### 8003 Defect Detection

~~~text
GET  /health
POST /api/defect-detection
~~~

## 12. Tests

若当前环境尚未安装 pytest，先执行：

~~~bash
python -m pip install pytest
~~~

这只是测试环境依赖，不要求为了运行主系统而修改 requirements.txt。

可靠的 perception / identity release tests：

~~~bash
python -B -m pytest -p no:cacheprovider -q tests/unit/test_perception_clients.py tests/unit/test_perception_orchestrator.py tests/unit/test_runtime_mission_registry.py tests/unit/test_stable_inspection_point_identity.py tests/integration/test_perception_api.py
~~~

Frontend behavior tests：

~~~bash
node tests/frontend/test_download_links.js
node tests/frontend/test_finalization_presentation.js
node tests/frontend/test_perception_ui.js
node tests/frontend/test_stable_point_identity.js
~~~

JavaScript syntax checks：

~~~bash
node --check web/static/app.js
node --check web/static/inspection_evidence.js
node --check web/static/mission_store.js
node --check web/static/playback.js
~~~

最终还应执行普通 Mission A 和 Replan Mission B 的真实浏览器 E2E，并确认标注图可以通过 8001 打开、Console 无新增关键错误。

本仓库存在历史 planning characterization failures，因此不宣称完整 Python suite 全部通过。封版判断以当前 perception、runtime Mission identity、前端行为和浏览器 E2E 为主。

## 13. Known Limitations

- RuntimeMissionRegistry 和 perception Job Store 是进程内存结构；
- 当前推荐以单 worker 运行 8001；
- 服务重启后旧页面的 Mission binding 会失效，需要重新加载/生成 Mission；
- A→B→C 连续重规划的 baseline 语义仍是后续工作，当前 Demo 聚焦 A→B；
- Replan Mission B 当前不提供 Mission 文件导出；
- playback 存在一个既有的 route-coordinate legacy issue；
- 部分旧 evidence image mapping 仍有限制；
- 历史 planning characterization tests 不作为当前 perception release gate；
- workflow/job 没有数据库持久化；
- 当前系统是研究原型和仿真展示，不执行真实无人机飞行控制。

## 14. Future Work

- 持久化 Runtime Mission registry 和 workflow result；
- 多 worker 共享运行时状态；
- 完整的 A→B→C replanning provenance；
- 更细粒度的阶段进度和可观测性；
- 更可靠的 evidence/media 生命周期管理；
- 真实 UAV telemetry 和低层轨迹执行系统的受控接口。

## 15. License

No explicit open-source license has been declared yet. Unless a license is added by the project owner, the repository should not be assumed to grant open-source reuse rights.
