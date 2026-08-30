"""Live Phase 2 smoke test for the 8002 -> 8003 perception pipeline."""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from perception.clients.defect_detection import DefectDetectionClient
from perception.clients.video_recognition import VideoRecognitionClient
from perception.mission_identity import MissionSnapshot
from perception.orchestrator import PerceptionOrchestrator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one live video-recognition and defect-detection workflow."
    )
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--inspection-point-id", required=True)
    parser.add_argument("--video-id")
    parser.add_argument("--runtime-mission-id")
    parser.add_argument(
        "--mission-file",
        type=Path,
        default=PROJECT_ROOT / "result" / "latest" / "mission_output.json",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        help="Override the total video polling deadline in seconds.",
    )
    return parser.parse_args()


def print_event(event: str, payload: Mapping[str, Any]) -> None:
    if event == "video_job_created":
        print(f"video job_id: {payload.get('job_id')}")
    elif event == "video_status":
        print(f"video status: {payload.get('status')}")
    elif event == "target_frames":
        print(f"target frame count: {payload.get('count')}")
    elif event == "frame_completed":
        defect = payload.get("defect_detection") or {}
        print(
            f"frame {payload.get('frame_id')}: "
            f"detection_count={defect.get('detection_count')}"
        )
    elif event == "frame_failed":
        error = payload.get("error") or {}
        print(
            f"frame {payload.get('frame_id')}: failed "
            f"({error.get('code')}: {error.get('message')})"
        )


async def run_pipeline(args: argparse.Namespace) -> int:
    if not args.video.is_file():
        print(f"ERROR: video file does not exist: {args.video}", file=sys.stderr)
        return 1
    try:
        snapshot = MissionSnapshot.from_file(
            args.mission_file, runtime_mission_id=args.runtime_mission_id
        )
        snapshot.require_inspection_point(args.inspection_point_id)
    except Exception as exc:
        print(f"ERROR: Mission identity validation failed: {exc}", file=sys.stderr)
        return 1

    async with VideoRecognitionClient() as video_client, DefectDetectionClient() as defect_client:
        try:
            print("8002 health:", json.dumps(await video_client.health(), ensure_ascii=False))
            print("8003 health:", json.dumps(await defect_client.health(), ensure_ascii=False))
        except Exception as exc:
            print(f"ERROR: upstream health check failed: {exc}", file=sys.stderr)
            return 1

        kwargs: dict[str, Any] = {
            "video_client": video_client,
            "defect_client": defect_client,
            "event_callback": print_event,
        }
        if args.timeout is not None:
            if args.timeout <= 0:
                print("ERROR: --timeout must be greater than zero", file=sys.stderr)
                return 1
            kwargs["poll_deadline_seconds"] = args.timeout
        orchestrator = PerceptionOrchestrator(**kwargs)
        result = await orchestrator.run(
            mission_snapshot=snapshot,
            video_path=args.video,
            inspection_point_id=args.inspection_point_id,
            video_id=args.video_id,
        )

    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    if result.status == "completed":
        print("FULL PIPELINE OK")
        return 0
    if result.status == "completed_with_errors":
        print("FULL PIPELINE COMPLETED WITH ERRORS")
        return 0
    print("FULL PIPELINE FAILED", file=sys.stderr)
    return 1


def main() -> int:
    return asyncio.run(run_pipeline(parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
