import copy
import json
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = PROJECT_ROOT / "result" / "latest" / "mission_output.json"

@pytest.fixture(scope="session")
def baseline_mission() -> dict:
    return json.loads(BASELINE_PATH.read_text(encoding="utf-8"))

@pytest.fixture
def baseline_copy(baseline_mission: dict) -> dict:
    return copy.deepcopy(baseline_mission)
