/**
 * Replan UI — start/end replan panel and /api/replan
 */
function updateReplanPanel() {
  const panel = $("replanPanel");
  if (!panel) return;
  if (isImagePipeline()) {
    panel.classList.remove("hidden");
  } else {
    panel.classList.add("hidden");
    state.pickPhase = null;
    updatePickHint();
  }
}

function updatePickHint() {
  const el = $("pickHint");
  if (!el) return;
  if (!state.pickPhase) {
    el.classList.add("hidden");
    el.textContent = "";
    return;
  }
  el.classList.remove("hidden");
  el.textContent =
    state.pickPhase === "start"
      ? "请在地图上点击设置起点"
      : "请在地图上点击设置终点";
}
function updateReplanInputLimits() {
  if (!isImagePipeline()) return;
  const { width, height } = getImageSize(state.lastResult || { map_background: state.mapConfig });
  const fields = [
    ["replanStartX", 0, width],
    ["replanEndX", 0, width],
    ["replanStartY", 0, height],
    ["replanEndY", 0, height],
  ];
  fields.forEach(([id, min, max]) => {
    const el = $(id);
    if (!el) return;
    el.min = String(min);
    el.max = String(max);
  });
}

function validateReplanCoords() {
  const sx = parseFloat($("replanStartX").value);
  const sy = parseFloat($("replanStartY").value);
  const ex = parseFloat($("replanEndX").value);
  const ey = parseFloat($("replanEndY").value);
  if ([sx, sy, ex, ey].some((v) => Number.isNaN(v))) {
    return { ok: false, message: "请填写完整的起点/终点坐标" };
  }
  const { width, height } = getImageSize(state.lastResult || { map_background: state.mapConfig });
  if (sx < 0 || sx > width || ex < 0 || ex > width || sy < 0 || sy > height || ey < 0 || ey > height) {
    return {
      ok: false,
      message: `起点/终点坐标超出图像范围：x 应在 [0,${width}]，y 应在 [0,${height}]`,
    };
  }
  return { ok: true, start: [sx, sy], end: [ex, ey] };
}
function buildReplanPayload(check) {
  const base = {
    pipeline: "image",
    start: check.start,
    end: check.end,
    planning_spacing: parseFloat($("planningSpacingInput")?.value) || 70,
    weather_aware: $("weatherAwareToggle")?.checked === true,
    weather_weight: Math.max(0, parseFloat($("weatherWeightInput")?.value || "1") || 1),
    dataset: $("datasetSelect")?.value,
    spacing: parseFloat($("spacingInput")?.value) || 50,
  };
  if (window.MissionStore) {
    return MissionStore.buildReplanPayload(base);
  }
  const meta = getCurrentMission()?.metadata || {};
  return {
    ...base,
    inspection_point_source: normalizeInspectionPointSource?.(getInspectionPointSource()) || getInspectionPointSource(),
    image_path: meta.map_image || getImagePathForSource(getInspectionPointSource()),
    inspection_points: getCurrentMission()?.inspection_points || [],
  };
}

async function runReplan() {
  if (!isImagePipeline()) {
    setStatus("重规划仅支持图像管线", "err");
    return;
  }
  readUiToState();
  const check = validateReplanCoords();
  if (!check.ok) {
    setStatus(check.message, "err");
    return;
  }

  const wasPlaying = typeof window.getPlaybackVisualState === "function"
    && window.getPlaybackVisualState()?.status === "playing";

  $("runReplanBtn").disabled = true;
  setStatus("起终点重规划中…", "running");
  if (AppPhaseManager) AppPhaseManager.beginReplanning();

  try {
    const res = await fetch("/api/replan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(buildReplanPayload(check)),
    });
    const data = await res.json();
    if (!res.ok || !data.ok) {
      throw new Error(data.message || data.detail || res.statusText);
    }

    const dashboard = normalizeMissionResult(data.dashboard || data);
    if (window.MissionStore) {
      MissionStore.applyServerReplan(dashboard, {
        mission_json_path: dashboard.output_files?.mission_snapshot || "latest_replan_mission.json",
      });
    } else {
      state.lastResult = dashboard;
    }
    state.experiment.active = false;
    if (window.LayerManager) LayerManager.clearLayer(window.LayerIds?.T3_AB_EXPERIMENT);
    renderExperimentResult();
    const mission = getCurrentMission();
    renderStats(mission?.statistics);
    renderVisitOrder(mission?.visit_order);
    renderMission(mission);
    renderMeta(mission?.metadata || {});
    updateDownloadLinks(dashboard.output_files || {}, "image");
    updateSpacingControlsVisibility();
    syncPlaybackAfterMissionChange({ restart: wasPlaying });

    const connected = mission?.metadata?.end_connected ?? dashboard.metadata?.end_connected;
    setStatus(
      `重规划完成 · ${mission?.statistics?.num_segments ?? 0} 段` +
        (connected ? " · 已连接终点" : " · 终点未连接"),
      connected ? "ok" : "err"
    );
  } catch (err) {
    setStatus(`重规划失败: ${err.message}`, "err");
    console.error(err);
    if (AppPhaseManager && MissionStore.getMission()) {
      AppPhaseManager.setPhase(AppPhase.MISSION_READY, { reason: "replan_failed" });
    }
  } finally {
    $("runReplanBtn").disabled = false;
  }
}

document.addEventListener("DOMContentLoaded", () => {
  $("clearReplanBtn")?.addEventListener("click", () => {
    ["replanStartX", "replanStartY", "replanEndX", "replanEndY"].forEach((id) => {
      const el = $(id);
      if (el) el.value = "";
    });
    state.pickPhase = null;
    updatePickHint();
    if (getCurrentMission() && window.MissionStore) {
      MissionStore.updateMarkers({});
    }
    refreshMapView?.();
  });
});
