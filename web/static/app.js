/**
 * UAV Mission Dashboard — dual pipeline + auto image generation
 */

const $ = (id) => document.getElementById(id);

const MAP_STYLES = {
  inspectColor: "#1f77b4",
  connectColor: "#ff7f0e",
  pointColor: "#2563eb",
  pointLine: "#e2e8f0",
};

const IMAGE_DEFAULT_SIZE = { width: 916, height: 960 };
const KEY_POINT_TYPES = new Set([
  "endpoint",
  "end",
  "start",
  "junction",
  "key",
  "corner",
  "tower",
  "pole",
  "terminal",
  "image_detected",
]);
const KEY_POINT_MAX = 30;

const state = {
  datasets: [],
  /** @deprecated 请使用 MissionStore.getCurrentMission()；保留为 getter/setter 别名 */
  lastResult: null,
  mapConfig: null,
  imageMissionAvailable: false,
  showBackground: false,
  showInspect: true,
  showConnect: true,
  showWeatherLayer: true,
  pointMode: "key",
  weatherAware: false,
  weatherWeight: 1.0,
  experiment: {
    active: false,
    overlay: true,
    runA: null,
    runB: null,
    metrics: null,
    reasoning: "",
  },
  dynamicWeather: {
    enabled: true,
    zones: [],
    baseZones: [],
    elapsed: 0,
    timer: null,
    lastTick: 0,
    riskThreshold: 0.72,
    predictionWindow: 10,
    autoPredictiveReplan: true,
    predictiveReplanCount: 0,
    predictedRisk: 0,
    predictedAffectedSegments: 0,
    timeToRisk: null,
    replanCooldownSec: 10,
    lastReplanAt: 0,
    lastWarningAt: 0,
    status: "正常巡航",
    events: [],
    replanTriggered: false,
    movementLogged: false,
    adaptivePath: null,
    adaptiveFlash: null,
    adaptiveFlashTimer: null,
    predictiveHud: null,
  },
  pickPhase: null,
  plotReady: { mapPlot: false, mapPlotFs: false },
};
window.state = state;

if (window.MissionStore) {
  // state.lastResult 仅为 MissionStore.getMission() 的兼容别名（deprecated）
  Object.defineProperty(state, "lastResult", {
    configurable: true,
    enumerable: true,
    get() {
      return MissionStore.getMission();
    },
    set(value) {
      if (value == null) {
        MissionStore.clear();
      } else {
        MissionStore.loadFromDashboard(value, { kind: "legacy_set" });
      }
    },
  });
}

function getCurrentMission() {
  return window.MissionStore ? MissionStore.getMission() : state.lastResult;
}

function syncPlaybackAfterMissionChange(options = {}) {
  const mission = getCurrentMission();
  if (!mission) return;
  if (typeof window.reloadPlaybackTimeline === "function") {
    window.reloadPlaybackTimeline(mission, options);
  } else if (typeof window.resetPlayback === "function") {
    window.resetPlayback(mission);
  }
  if (options.restart && typeof window.startInspectionPlayback === "function") {
    window.startInspectionPlayback({ preservePhase: true });
  }
}

function getPipeline() {
  return $("pipelineSelect")?.value || "image";
}

function isImagePipeline() {
  return getPipeline() === "image";
}

function getMapMode() {
  const sel = $("mapModeSelect");
  if (!sel || sel.disabled) {
    return isImagePipeline() ? "image_overlay" : "topology_only";
  }
  return sel.value;
}

async function loadMapConfig(imagePath) {
  try {
    const path = imagePath || "data/test.png";
    const res = await fetch(`/api/map/config?path=${encodeURIComponent(path)}`);
    if (!res.ok) throw new Error(res.statusText);
    state.mapConfig = await res.json();
  } catch (e) {
    state.mapConfig = { available: false };
  }
}

async function checkImageMission() {
  try {
    const res = await fetch("/api/image-mission/status");
    return await res.json();
  } catch (e) {
    return { available: false, message: e.message };
  }
}

async function loadDatasets() {
  const res = await fetch("/api/datasets");
  const data = await res.json();
  state.datasets = data.datasets || [];
}

function getImagePathForSource(source) {
  return source === "image" ? "data/test_point.png" : "data/test.png";
}

function getInspectionPointSource() {
  const ui = $("inspectionPointSourceSelect")?.value || "spacing";
  return window.normalizeInspectionPointSource
    ? normalizeInspectionPointSource(ui)
    : ui;
}

function updateSpacingControlsVisibility() {
  const imageMode = isImagePipeline() && getInspectionPointSource() === "image";
  const storeImage = window.MissionStore?.isImageSource?.() === true;
  const hideSpacing = imageMode || storeImage;

  const spacingField = $("spacingFieldWrap") || $("spacingInput")?.closest(".field");
  const planningField = $("planningSpacingFieldWrap") || $("planningSpacingInput")?.closest(".field");
  if (spacingField) spacingField.classList.toggle("hidden", hideSpacing);
  if (planningField) planningField.classList.toggle("hidden", hideSpacing);

  const replanHint = $("replanHint");
  if (replanHint && hideSpacing) {
    replanHint.textContent = "仅重建 connect 段；保留图像巡检点与 ID。";
  } else if (replanHint) {
    replanHint.textContent = "调整起终点后重规划；spacing 模式下可配合「重规划采样间距」。";
  }
}

function applyPipelineUi() {
  const image = isImagePipeline();
  const pointSource = getInspectionPointSource();
  if ($("pipelineBadge")) {
    $("pipelineBadge").textContent = image ? "图像任务" : "统一任务";
  }

  const ds = $("datasetSelect");
  const pl = $("plannerSelect");
  const sp = $("spacingInput");
  const mm = $("mapModeSelect");
  const forceBtn = $("forceRegenBtn");
  const pointSourceSel = $("inspectionPointSourceSelect");

  if (image) {
    const imagePath = getImagePathForSource(pointSource);
    if (ds) {
      ds.innerHTML = `<option value="${imagePath}">${imagePath}</option>`;
      ds.disabled = true;
    }
    if (pointSourceSel) pointSourceSel.disabled = false;
    if (pl) {
      pl.innerHTML = '<option value="legacy" selected>legacy</option>';
      pl.disabled = true;
    }
    const hideSpacing = pointSource === "image";
    if (sp) sp.disabled = hideSpacing;
    if (mm) {
      mm.innerHTML = `
      <option value="image_overlay" selected>图像底图</option>
      <option value="topology_only">深色拓扑</option>`;
      mm.disabled = false;
    }
    if ($("runBtn")) $("runBtn").textContent = "生成任务";
    forceBtn?.classList.remove("hidden");
  } else {
    if (pointSourceSel) pointSourceSel.disabled = true;
    forceBtn?.classList.add("hidden");
    $("dlLegacyHtml")?.classList.add("hidden");
    if (ds) {
      ds.disabled = false;
      ds.innerHTML = "";
      state.datasets.forEach((d) => {
        const opt = document.createElement("option");
        opt.value = d.path;
        opt.textContent = d.label;
        ds.appendChild(opt);
      });
      const topo = state.datasets.find((d) => d.id.includes("topo"));
      if (topo) ds.value = topo.path;
    }
    if (pl) {
      pl.disabled = false;
      pl.innerHTML = `
      <option value="baseline">baseline</option>
      <option value="optimized">optimized</option>
      <option value="dijkstra">dijkstra</option>`;
    }
    if (sp) sp.disabled = false;
    if (mm) {
      mm.disabled = false;
      mm.innerHTML = `
      <option value="topology_only" selected>深色拓扑</option>
      <option value="image_overlay">实验视图</option>`;
    }
    if ($("runBtn")) $("runBtn").textContent = "生成任务";
  }
  applyLayerDefaultsForPipeline();
  updateMapModeHint();
  updateReplanPanel();
  updateReplanInputLimits();
  updateSpacingControlsVisibility();
}


function readUiToState() {
  state.showBackground = $("toggleBaseMap")?.checked !== false;
  state.showInspect = $("toggleInspect")?.checked !== false;
  state.showConnect = $("toggleConnect")?.checked !== false;
  state.showWeatherLayer = $("toggleWeatherLayer")?.checked === true;
  if ($("togglePointsLayer")) {
    state.pointMode = $("togglePointsLayer").checked ? "key" : "hidden";
  } else {
    state.pointMode = $("pointModeSelect")?.value || "key";
  }
  state.weatherAware = $("weatherAwareToggle")?.checked === true;
  state.weatherWeight = Math.max(0, parseFloat($("weatherWeightInput")?.value || "1") || 1);
  state.experiment.overlay = $("experimentOverlayToggle")?.checked !== false;
  state.dynamicWeather.enabled = $("dynamicWeatherToggle")?.checked !== false;
  state.dynamicWeather.riskThreshold = Math.max(
    0.1,
    parseFloat($("adaptiveRiskThresholdInput")?.value || "0.72") || 0.72
  );
  state.dynamicWeather.predictionWindow = Math.max(
    1,
    parseInt($("predictionWindowSelect")?.value || "10", 10) || 10
  );
  state.dynamicWeather.autoPredictiveReplan = $("autoPredictiveReplanToggle")?.checked !== false;
  state.dynamicWeather.replanCooldownSec = Math.max(
    3,
    parseInt($("replanCooldownSelect")?.value || "10", 10) || 10
  );
}

function syncStateToUi() {
  if ($("toggleBaseMap")) $("toggleBaseMap").checked = state.showBackground;
  if ($("toggleInspect")) $("toggleInspect").checked = state.showInspect;
  if ($("toggleConnect")) $("toggleConnect").checked = state.showConnect;
  if ($("toggleWeatherLayer")) $("toggleWeatherLayer").checked = state.showWeatherLayer;
  if ($("togglePointsLayer")) $("togglePointsLayer").checked = state.pointMode !== "hidden";
  if ($("pointModeSelect")) $("pointModeSelect").value = state.pointMode;
  if ($("weatherAwareToggle")) $("weatherAwareToggle").checked = state.weatherAware;
  if ($("weatherWeightInput")) $("weatherWeightInput").value = String(state.weatherWeight);
  if ($("experimentOverlayToggle")) $("experimentOverlayToggle").checked = state.experiment.overlay;
  if ($("dynamicWeatherToggle")) $("dynamicWeatherToggle").checked = state.dynamicWeather.enabled;
  if ($("adaptiveRiskThresholdInput")) $("adaptiveRiskThresholdInput").value = String(state.dynamicWeather.riskThreshold);
  if ($("predictionWindowSelect")) $("predictionWindowSelect").value = String(state.dynamicWeather.predictionWindow);
  if ($("autoPredictiveReplanToggle")) $("autoPredictiveReplanToggle").checked = state.dynamicWeather.autoPredictiveReplan;
  if ($("replanCooldownSelect")) $("replanCooldownSelect").value = String(state.dynamicWeather.replanCooldownSec || 10);
}

function applyLayerDefaultsForPipeline() {
  if (isImagePipeline()) {
    state.showBackground = getMapMode() === "image_overlay";
    state.pointMode = "key";
  } else {
    state.showBackground = false;
    state.pointMode = "all";
  }
  if (state.showWeatherLayer == null) state.showWeatherLayer = false;
  syncStateToUi();
}

function getImageSize(result) {
  const bg = result?.map_background || state.mapConfig || {};
  return {
    width: bg.width || IMAGE_DEFAULT_SIZE.width,
    height: bg.height || IMAGE_DEFAULT_SIZE.height,
  };
}


function normalizeMissionResult(result) {
  const pipeline = result?.metadata?.pipeline || getPipeline();
  const out = { ...result };
  out.metadata = { ...(result.metadata || {}), pipeline };
  if (window.normalizeInspectionPointSource) {
    out.metadata.inspection_point_source = normalizeInspectionPointSource(
      out.metadata.inspection_point_source
    );
  }
  out.statistics = { ...(result.statistics || {}) };
  if (out.metadata.weather_mode != null) {
    out.statistics.weather_mode = out.metadata.weather_mode;
  }

  if (pipeline === "image") {
    const mapBg =
      result.map_background?.available !== false
        ? result.map_background || state.mapConfig
        : state.mapConfig;
    out.map_background = mapBg;
    if (out.metadata?.clean_map_image && mapBg?.path !== out.metadata.clean_map_image) {
      out.map_background = {
        ...mapBg,
        path: out.metadata.clean_map_image,
        url: `/api/map/background?path=${encodeURIComponent(out.metadata.clean_map_image)}`,
        layout_image: mapBg.layout_image
          ? {
              ...mapBg.layout_image,
              source: `/api/map/background?path=${encodeURIComponent(out.metadata.clean_map_image)}`,
            }
          : null,
      };
    }
    const { width, height } = getImageSize(out);
    out.bounds = {
      x_range: [0, width],
      y_range: [height, 0],
      width,
      height,
    };
  }
  return out;
}

function samplePointsEvenly(points, maxCount) {
  if (points.length <= maxCount) return points;
  const step = points.length / maxCount;
  const sampled = [];
  for (let i = 0; i < maxCount; i += 1) {
    sampled.push(points[Math.min(points.length - 1, Math.floor(i * step))]);
  }
  return sampled;
}

function selectInspectionPoints(points, mode) {
  if (!points?.length || mode === "hidden") return [];

  if (mode === "all") return points;

  let keyPts = points.filter((p) => {
    const t = String(p.point_type || p.type || "").toLowerCase();
    if (KEY_POINT_TYPES.has(t)) return true;
    const pid = String(p.point_id || "");
    return /start|end|key|junction|tower|pole/i.test(pid);
  });

  if (keyPts.length < 8) {
    const seen = new Set(keyPts.map((p) => `${p.x},${p.y}`));
    points.forEach((p, idx) => {
      if (keyPts.length >= KEY_POINT_MAX) return;
      const k = `${p.x},${p.y}`;
      if (idx % Math.max(1, Math.floor(points.length / KEY_POINT_MAX)) === 0 && !seen.has(k)) {
        keyPts.push(p);
        seen.add(k);
      }
    });
  }

  return samplePointsEvenly(keyPts, KEY_POINT_MAX);
}

function refreshMapView() {
  if (!state.lastResult) return;
  renderMission(state.lastResult, "mapPlot");
  const fullscreenOpen =
    $("mapFullscreenModal") && !$("mapFullscreenModal").classList.contains("hidden");
  if (fullscreenOpen) {
    renderMission(state.lastResult, "mapPlotFs", { fullscreen: true });
  }
}

function setStatus(text, cls = "") {
  const box = $("statusBox");
  if (!box) return;
  box.textContent = text;
  box.className = `status status-compact${cls ? ` ${cls}` : ""}`;
}

function updateMapModeHint() {
  const hint = $("mapModeHint");
  if (!hint) return;
  if (isImagePipeline()) {
    hint.textContent = state.imageMissionAvailable ? "任务已就绪" : "请先加载任务";
    $("mapPlot")?.classList.add("overlay-mode");
  } else {
    hint.textContent =
      getMapMode() === "topology_only"
        ? "深色拓扑视图"
        : "实验视图";
    $("mapPlot")?.classList.remove("overlay-mode");
  }
}

function updateDownloadLinks(outputFiles, pipeline) {
  const base = "/api/output/";
  const legacy = $("dlLegacyHtml");

  if (pipeline === "image") {
    const snap = outputFiles?.mission_snapshot || "latest_image_mission_snapshot.json";
    $("headerDownload").href = base + snap;
    $("dlMission").href = base + snap;
    $("dlAnalysis").classList.add("hidden");
    $("dlCompare").classList.add("hidden");
    if (outputFiles?.legacy_html || outputFiles?.legacy_html_path) {
      legacy.href = outputFiles.legacy_html || "/legacy/html";
      legacy.classList.remove("hidden");
    }
    return;
  }

  legacy.classList.add("hidden");
  const mission = outputFiles?.mission || "latest_mission.json";
  $("headerDownload").href = base + mission;
  $("dlMission").href = base + mission;
  $("dlAnalysis").href = base + (outputFiles?.analysis || "latest_analysis.json");
  $("dlAnalysis").classList.remove("hidden");
  const cmp = $("dlCompare");
  if (outputFiles?.compare) {
    cmp.href = base + outputFiles.compare;
    cmp.classList.remove("hidden");
  } else {
    cmp.classList.add("hidden");
  }
}

function renderMeta(metadata) {
  const box = $("metaBox");
  if (!box) return;
  const lines = [
    metadata?.weather_mode === "on" ? "天气感知已开启" : "天气感知关闭",
    state.dynamicWeather.predictiveReplanCount
      ? `预测重规划 ${state.dynamicWeather.predictiveReplanCount} 次`
      : null,
  ].filter(Boolean);
  box.innerHTML = lines.map((l) => `<div>${escapeHtml(l)}</div>`).join("");
  renderSystemStatus();
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;");
}

const PHASE_LABELS = {
  idle: "待命",
  planning: "规划中",
  mission_ready: "任务就绪",
  playing: "巡检中",
  paused: "已暂停",
  replanning: "重规划中",
  adaptive_warning: "风险预警",
  finished: "已完成",
};

function updateMissionStatusHud() {
  const phase = window.AppPhaseManager?.getPhase?.() || window.AppPhase?.IDLE || "idle";
  const flash = state.dynamicWeather.adaptiveFlash;
  const risk = Number(state.dynamicWeather.predictedRisk || 0);
  const th = Number(state.dynamicWeather.riskThreshold || 0.72);

  let label = "NORMAL";
  let mod = "normal";
  if (phase === "replanning") {
    label = "REPLANNING";
    mod = "replan";
  } else if (flash) {
    label = "REROUTING";
    mod = "reroute";
  } else if (phase === "adaptive_warning" || risk > th) {
    label = "WEATHER ALERT";
    mod = "alert";
  } else if (risk > th * 0.55) {
    label = "WARNING";
    mod = "warn";
  }

  const applyHud = (rootId, textId) => {
    const el = $(rootId);
    const tEl = $(textId);
    if (!el || !tEl) return;
    el.classList.remove(
      "mission-status-hud--normal",
      "mission-status-hud--warn",
      "mission-status-hud--alert",
      "mission-status-hud--reroute",
      "mission-status-hud--replan"
    );
    el.classList.add(`mission-status-hud--${mod}`);
    tEl.textContent = label;
  };
  applyHud("missionStatusHud", "missionStatusHudText");
  applyHud("missionStatusHudFs", "missionStatusHudTextFs");
}

function renderSystemStatus() {
  const phase = window.AppPhaseManager?.getPhase?.() || window.AppPhase?.IDLE || "idle";
  const phaseText = PHASE_LABELS[phase] || "待命";
  if ($("sysPhase")) $("sysPhase").textContent = phaseText;
  if ($("missionPhaseBadge")) $("missionPhaseBadge").textContent = phaseText;

  const weatherText = state.dynamicWeather.status || "正常";
  if ($("sysWeather")) $("sysWeather").textContent = weatherText;

  let replanText = "未触发";
  if (state.dynamicWeather.replanTriggered) replanText = "已触发绕行";
  else if (MissionStore?.current?.last_update_kind === "server_replan") replanText = "已完成重规划";
  if ($("sysReplan")) $("sysReplan").textContent = replanText;

  const visual = typeof window.getPlaybackVisualState === "function"
    ? window.getPlaybackVisualState()
    : null;
  const mission = getCurrentMission();
  let pointText = "—";
  let segmentText = "—";
  if (visual?.currentPointId && mission?.inspection_points?.length) {
    const pt = mission.inspection_points.find(
      (p) => (p.id || p.point_id) === visual.currentPointId
    );
    if (pt) {
      pointText = String(pt.point_id || pt.id || "—").replace(/^point_/, "巡检点 ");
      if (pt.segment_id) {
        segmentText = String(pt.segment_id).replace(/^seg_/, "区段 ");
      }
    }
  }
  const segLabel = $("timelineSegment")?.textContent;
  if (segmentText === "—" && segLabel && !segLabel.includes("--")) {
    segmentText = segLabel.replace(/^区段\s*/, "区段 ");
  }
  if ($("sysCurrentPoint")) $("sysCurrentPoint").textContent = pointText;
  if ($("sysCurrentSegment")) $("sysCurrentSegment").textContent = segmentText;
  if ($("playbackConsoleState")) $("playbackConsoleState").textContent = phaseText;
  const rk = formatRiskTier(state.dynamicWeather.predictedRisk);
  if ($("inspectCardRisk")) $("inspectCardRisk").textContent = rk;
  if ($("inspectCardRiskFs")) $("inspectCardRiskFs").textContent = rk;
  updateMissionStatusHud();
}

window.renderSystemStatus = renderSystemStatus;

function renderAdvancedStats(statistics) {
  const host = $("advancedStatCards");
  if (!host) return;
  const advancedOpen = document.getElementById("advancedExperimentPanel")?.open;
  if (!advancedOpen && !state.experiment.active) {
    host.classList.add("hidden");
    host.innerHTML = "";
    return;
  }
  const s = statistics || {};
  const items = [
    ["connect_ratio", "连接占比", ((s.connect_ratio || 0) * 100).toFixed(1), "%"],
    ["total_cost", "实际总代价", s.total_cost ?? s.total_length ?? 0, ""],
    ["predicted_affected_segments", "预测影响采样点", state.dynamicWeather.predictedAffectedSegments ?? 0, ""],
    ["time_to_risk", "预计进入风险", state.dynamicWeather.timeToRisk != null ? `${state.dynamicWeather.timeToRisk}s` : "—", ""],
    ["risky_distance", "高风险穿越", s.risky_distance ?? 0, " px"],
    ["predictive_replan_count", "预测重规划", state.dynamicWeather.predictiveReplanCount ?? 0, ""],
  ];
  host.classList.remove("hidden");
  host.innerHTML = items
    .map(
      ([, label, val, unit]) =>
        `<div class="card"><div class="label">${label}</div><div class="value">${val ?? "—"}${unit}</div></div>`
    )
    .join("");
}

function formatMetricNumber(value) {
  if (value == null || value === "" || value === "—") return "—";
  return String(value).replace(/m\/s|m|%|px/gi, "").trim();
}

function formatRiskTier(riskVal) {
  const r = Math.max(0, Number(riskVal || 0));
  if (r >= 0.85) return "极高";
  if (r >= 0.65) return "高";
  if (r >= 0.45) return "中";
  if (r >= 0.2) return "低";
  return "正常";
}

function renderStats(statistics) {
  const s = statistics || {};
  const phase = window.AppPhaseManager?.getPhase?.() || window.AppPhase?.IDLE || "idle";
  const phaseText = (PHASE_LABELS && PHASE_LABELS[phase]) || phase;
  const riskVal = Number(state.dynamicWeather.predictedRisk ?? 0);
  const wp = s.weather_penalty_total;
  const wpStr = wp != null && wp !== "" && Number.isFinite(Number(wp)) ? Number(wp).toFixed(2) : "—";

  const primary = $("statCardsPrimary");
  const secondary = $("statCardsSecondary");
  const itemsPrimary = [
    ["total_length", "总长度", s.total_length, " px"],
    ["num_inspection_points", "巡检点数", s.num_inspection_points, ""],
    ["phase", "任务状态", phaseText, ""],
    ["predicted_risk", "天气风险", riskVal.toFixed(2), ""],
  ];
  if (primary) {
    primary.innerHTML = itemsPrimary
      .map(
        ([, label, val, unit]) => `
    <div class="card"><div class="label">${label}</div>
    <div class="value">${val ?? "—"}${unit}</div></div>`
      )
      .join("");
  }

  const cr = s.connect_ratio != null ? Number(s.connect_ratio) : null;
  const connectPct = cr != null && Number.isFinite(cr) ? (cr <= 1 ? cr * 100 : cr) : null;
  const itemsSecondary = [
    ["connect_ratio", "连接占比", connectPct != null ? connectPct.toFixed(1) : "—", connectPct != null ? "%" : ""],
    ["risky_distance", "高风险穿越", s.risky_distance ?? "—", s.risky_distance != null ? " px" : ""],
    ["pred_samples", "预测采样点", state.dynamicWeather.predictedAffectedSegments ?? 0, ""],
    ["weather_penalty", "天气惩罚", wpStr, ""],
    ["reroute", "预测重规划", state.dynamicWeather.predictiveReplanCount ?? 0, " 次"],
  ];
  if (secondary) {
    secondary.innerHTML = itemsSecondary
      .map(
        ([, label, val, unit]) => `
    <div class="card"><div class="label">${label}</div>
    <div class="value">${val ?? "—"}${unit}</div></div>`
      )
      .join("");
  }

  const legacyHost = $("statCards");
  if (legacyHost) legacyHost.innerHTML = "";

  renderAdvancedStats(s);
  renderSystemStatus();
}

function renderVisitOrder(visitOrder) {
  const chips = $("visitChips");
  if (!chips) return;
  if (!visitOrder?.length) {
    chips.innerHTML = '<span class="chip">（无）</span>';
    return;
  }
  chips.innerHTML = visitOrder
    .map(
      (_eid, i) =>
        `<span class="chip"><span class="idx">${i + 1}</span>${escapeHtml(`巡检区段 ${i + 1}`)}</span>`
    )
    .join("");
}


function flattenMissionPathPoints(result) {
  const out = [];
  const segs = result?.segments || [];
  segs.forEach((s) => {
    const geom = s.geometry_2d || [];
    geom.forEach((p, idx) => {
      if (!p || p.length < 2) return;
      if (out.length && idx === 0) {
        const last = out[out.length - 1];
        if (Math.abs(last[0] - p[0]) < 1e-6 && Math.abs(last[1] - p[1]) < 1e-6) return;
      }
      out.push([Number(p[0]), Number(p[1])]);
    });
  });
  return out;
}






function buildMissionTraces(result) {
  const mission = getCurrentMission() || result;
  const segments = mission?.segments || [];
  const inspectX = [],
    inspectY = [],
    inspectCustom = [];
  const connectX = [],
    connectY = [],
    connectCustom = [];

  segments.forEach((seg) => {
    const geom = seg.geometry_2d || [];
    if (geom.length < 2) return;
    const segLabel = seg.segment_id
      ? `巡检区段 ${String(seg.segment_id).replace(/^seg_/, "")}`
      : "巡检区段";
    const typeLabel = seg.type === "inspect" ? "巡检段" : "连接段";
    const hoverLine = `${segLabel}<br>${typeLabel} · ${Math.round(seg.length)}px`;

    if (seg.type === "inspect") {
      inspectX.push(...geom.map((p) => p[0]), null);
      inspectY.push(...geom.map((p) => p[1]), null);
      inspectCustom.push(...geom.map(() => hoverLine), null);
    } else {
      connectX.push(...geom.map((p) => p[0]), null);
      connectY.push(...geom.map((p) => p[1]), null);
      connectCustom.push(...geom.map(() => hoverLine), null);
    }
  });

  [inspectX, inspectY, inspectCustom, connectX, connectY, connectCustom].forEach((arr) => {
    if (arr.length && arr[arr.length - 1] === null) arr.pop();
  });

  const traces = [];
  if (state.showWeatherLayer) {
    traces.push(...(window.LayerManager?.getLayer(window.LayerIds?.L3_WEATHER) || buildWeatherZoneTraces(mission)));
  }
  if (state.experiment.active && state.experiment.overlay) {
    traces.push(...(window.LayerManager?.getLayer(window.LayerIds?.T3_AB_EXPERIMENT) || buildExperimentOverlayTraces()));
  }
  if (state.dynamicWeather.adaptiveFlash) {
    traces.push(...(window.LayerManager?.getLayer(window.LayerIds?.T1_ADAPTIVE_FLASH) || buildAdaptiveReplanTraces()));
  }

  if (state.showInspect && inspectX.length) {
    traces.push({
      x: inspectX,
      y: inspectY,
      mode: "lines",
      name: "巡检路径",
      line: { color: MAP_STYLES.inspectColor, width: 4 },
      hoverinfo: "text",
      hovertext: inspectCustom,
      showlegend: true,
      legendgroup: "inspect",
    });
  }

  if (state.showConnect && connectX.length) {
    traces.push({
      x: connectX,
      y: connectY,
      mode: "lines",
      name: "连接路径",
      line: { color: MAP_STYLES.connectColor, width: 3, dash: "dash" },
      hoverinfo: "text",
      hovertext: connectCustom,
      showlegend: true,
      legendgroup: "connect",
    });
  }

  const visiblePts = selectInspectionPoints(
    mission.inspection_points || [],
    state.pointMode
  );
  if (visiblePts.length) {
    const isAll = state.pointMode === "all";
    const playbackVisual = typeof window.getPlaybackVisualState === "function"
      ? window.getPlaybackVisualState()
      : null;
    const visited = playbackVisual?.visitedIds || new Set();
    const currentId = playbackVisual?.currentPointId || null;
    const totalPoints = (mission.inspection_points || []).length || visiblePts.length;
    const unvisited = [];
    const visitedPts = [];
    let currentPt = null;

    visiblePts.forEach((p, idx) => {
      const pid = p.id || p.point_id || `point_${idx}`;
      const pointLabel = `巡检点 ${String(p.point_id || p.id || idx + 1).replace(/^point_/, "")}`;
      const segmentLabel = p.segment_id
        ? `巡检区段 ${String(p.segment_id).replace(/^seg_/, "")}`
        : "—";
      const isCurrent = currentId && pid === currentId;
      const isVisited = visited.has(pid);
      const cd = [
        segmentLabel,
        p.image_available ? "真实图片" : "占位图",
        `${p.progress_index || idx + 1}/${totalPoints}`,
        pointLabel,
      ];
      const item = { p, cd };
      if (isCurrent) currentPt = item;
      else if (isVisited) visitedPts.push(item);
      else unvisited.push(item);
    });

    if (unvisited.length) {
      traces.push({
        x: unvisited.map((v) => v.p.x),
        y: unvisited.map((v) => v.p.y),
        mode: "markers",
        name: "巡检点光晕",
        marker: {
          size: isAll ? 12 : 16,
          color: "rgba(34, 211, 238, 0.18)",
          line: { width: 0, color: "rgba(0,0,0,0)" },
        },
        hoverinfo: "skip",
        showlegend: false,
        legendgroup: "points",
      });
      traces.push({
        x: unvisited.map((v) => v.p.x),
        y: unvisited.map((v) => v.p.y),
        mode: "markers",
        name: "巡检点",
        marker: {
          size: isAll ? 6 : 8,
          color: "#22d3ee",
          opacity: isAll ? 0.72 : 0.95,
          line: { width: isAll ? 1.4 : 2, color: "#ffffff" },
        },
        text: unvisited.map((v) => v.cd[3]),
        customdata: unvisited.map((v) => v.cd),
        hovertemplate:
          "<b>%{text}</b><br>" +
          "区段: %{customdata[0]}<br>" +
          "图片: %{customdata[1]}<br>" +
          "进度: %{customdata[2]}<extra></extra>",
        showlegend: true,
        legendgroup: "points",
      });
    }

    if (visitedPts.length) {
      traces.push({
        x: visitedPts.map((v) => v.p.x),
        y: visitedPts.map((v) => v.p.y),
        mode: "markers",
        name: "已巡检点",
        marker: {
          size: isAll ? 6 : 8,
          color: "#22c55e",
          opacity: 0.88,
          line: { width: 2, color: "#ffffff" },
        },
        text: visitedPts.map((v) => v.cd[3]),
        customdata: visitedPts.map((v) => v.cd),
        hovertemplate:
          "<b>%{text}</b><br>" +
          "区段: %{customdata[0]}<br>" +
          "图片: %{customdata[1]}<br>" +
          "进度: %{customdata[2]}<extra></extra>",
        showlegend: true,
        legendgroup: "points",
      });
    }

    if (currentPt) {
      traces.push({
        x: [currentPt.p.x],
        y: [currentPt.p.y],
        mode: "markers",
        name: "当前巡检点光晕",
        marker: {
          size: 28,
          color: "rgba(250, 204, 21, 0.24)",
          line: { width: 0, color: "rgba(0,0,0,0)" },
        },
        hoverinfo: "skip",
        showlegend: false,
        legendgroup: "points",
      });
      traces.push({
        x: [currentPt.p.x],
        y: [currentPt.p.y],
        mode: "markers",
        name: "当前巡检点",
        marker: {
          size: 13,
          color: "#facc15",
          opacity: 1,
          line: { width: 2.4, color: "#ffffff" },
        },
        text: [currentPt.cd[3]],
        customdata: [currentPt.cd],
        hovertemplate:
          "<b>%{text}</b><br>" +
          "区段: %{customdata[0]}<br>" +
          "图片: %{customdata[1]}<br>" +
          "进度: %{customdata[2]}<extra></extra>",
        showlegend: true,
        legendgroup: "points",
      });
    }
  }

  const markers = result.markers || {};
  if (markers.start) {
    traces.push({
      x: [markers.start.x],
      y: [markers.start.y],
      mode: "markers+text",
      name: "起点",
      marker: {
        size: 16,
        color: "#22c55e",
        symbol: "star",
        line: { width: 2, color: "#14532d" },
      },
      text: ["起点"],
      textfont: { size: 10, color: "#bbf7d0" },
      textposition: "top center",
      hovertemplate: "起点 (%{x:.0f}, %{y:.0f})<extra></extra>",
      showlegend: true,
      legendgroup: "start",
    });
  }
  if (markers.end) {
    traces.push({
      x: [markers.end.x],
      y: [markers.end.y],
      mode: "markers+text",
      name: "终点",
      marker: {
        size: 16,
        color: "#ef4444",
        symbol: "diamond",
        line: { width: 2, color: "#7f1d1d" },
      },
      text: ["终点"],
      textfont: { size: 10, color: "#fecaca" },
      textposition: "bottom center",
      hovertemplate: "终点 (%{x:.0f}, %{y:.0f})<extra></extra>",
      showlegend: true,
      legendgroup: "end",
    });
  }

  return traces;
}

function buildLegendLayout(fullscreen = false) {
  // Legend inside the plot area at bottom-left — zero bottom margin waste
  return {
    orientation: "h",
    x: 0.01,
    y: 0.003,
    xanchor: "left",
    yanchor: "bottom",
    bgcolor: "rgba(10, 16, 28, 0.72)",
    bordercolor: "rgba(100,160,200,0.18)",
    borderwidth: 1,
    font: { size: fullscreen ? 11 : 10, color: "#94b8cc" },
  };
}

function getFullscreenPlotSize() {
  const panel = document.querySelector(".modal-panel");
  const plotEl = $("mapPlotFs");
  if (panel && plotEl) {
    const stage = plotEl.closest(".plot-stage-fs") || plotEl.parentElement;
    const pr = (stage || panel).getBoundingClientRect();
    const header = panel.querySelector(".modal-header");
    const headerH = header ? header.offsetHeight + 4 : 36;
    return {
      width: Math.max(380, Math.floor(pr.width - 2)),
      height: Math.max(360, Math.floor((stage ? pr.height : panel.getBoundingClientRect().height - headerH) - 2)),
    };
  }
  return {
    width: Math.floor(window.innerWidth * 0.74),
    height: Math.floor(window.innerHeight) - 72,
  };
}

function buildBackgroundImages(result) {
  const pipeline = result.metadata?.pipeline || getPipeline();
  if (pipeline !== "image") return [];
  if (getMapMode() === "topology_only") return [];

  const mapBg = result.map_background || state.mapConfig;
  if (!mapBg?.available) return [];

  const { width, height } = getImageSize(result);
  const baseImg = mapBg.layout_image || {
    source: mapBg.url || "/api/map/background",
    xref: "x",
    yref: "y",
    x: 0,
    y: height,
    sizex: width,
    sizey: height,
    sizing: "stretch",
    layer: "below",
    opacity: 0.92,
  };
  return [{ ...baseImg, opacity: baseImg.opacity ?? 0.92 }];
}

function buildMissionLayout(result, options = {}) {
  const { fullscreen = false } = options;
  const pipeline = result.metadata?.pipeline || getPipeline();
  const isImage = pipeline === "image";

  const layout = {
    paper_bgcolor: "#1a2332",
    plot_bgcolor: isImage ? "rgba(255,255,255,0.06)" : "#243044",
    font: { color: "#e6edf3", size: 11 },
    margin: fullscreen
      ? { l: 6, r: 6, t: 2, b: 4 }
      : { l: 14, r: 8, t: 4, b: 6 },
    legend: buildLegendLayout(fullscreen),
    hovermode: "closest",
    images: buildBackgroundImages(result),
    uirevision: "mission-map-v3",
    autosize: true,
  };

  if (isImage) {
    const darkTopology = getMapMode() === "topology_only";
    const { width, height } = getImageSize(result);
    layout.plot_bgcolor = darkTopology ? "#243044" : "rgba(255,255,255,0.06)";
    layout.xaxis = {
      title: "",
      range: [0, width],
      domain: [0.0, 1.0],
      autorange: false,
      fixedrange: true,
      constrain: "domain",
      showgrid: true,
      gridcolor: "rgba(255,255,255,0.10)",
      tickfont: { size: 9, color: "#5a7a94" },
      zeroline: false,
    };
    layout.yaxis = {
      title: "",
      range: [height, 0],
      domain: [0.0, 1.0],
      autorange: false,
      fixedrange: true,
      scaleanchor: "x",
      scaleratio: 1,
      showgrid: true,
      gridcolor: "rgba(255,255,255,0.10)",
      tickfont: { size: 9, color: "#5a7a94" },
      zeroline: false,
    };
  } else {
    const b = result.bounds || { x_range: [0, 1000], y_range: [1000, 0] };
    layout.xaxis = {
      title: "",
      range: b.x_range,
      domain: [0.0, 1.0],
      autorange: false,
      gridcolor: "#2d3a4f",
      tickfont: { size: 9, color: "#5a7a94" },
      zeroline: false,
    };
    layout.yaxis = {
      title: "",
      range: b.y_range,
      domain: [0.0, 1.0],
      scaleanchor: "x",
      scaleratio: 1,
      gridcolor: "#2d3a4f",
      tickfont: { size: 9, color: "#5a7a94" },
      zeroline: false,
    };
  }

  if (fullscreen) {
    const sz = getFullscreenPlotSize();
    layout.autosize = false;
    layout.width = sz.width;
    layout.height = sz.height;
  }

  return layout;
}

function buildPlotFromMission(result, options = {}) {
  let traces = buildMissionTraces(result);
  if (typeof window.appendPlaybackTraces === "function") {
    traces = window.appendPlaybackTraces(traces, result);
  }
  const layout = buildMissionLayout(result, options);
  return {
    traces,
    layout,
    config: {
      responsive: !options.fullscreen,
      displayModeBar: true,
      scrollZoom: true,
    },
  };
}

function onMissionLoaded(result, options = {}) {
  const normalized = normalizeMissionResult(result);
  if (window.MissionStore) {
    if (!options.skipStoreLoad) {
      MissionStore.loadFromDashboard(normalized, { kind: options.kind || "initial" });
    }
  } else {
    state.lastResult = normalized;
  }
  resetDynamicWeather(getCurrentMission() || normalized);
  updateSpacingControlsVisibility();
  if (typeof window.onMissionLoadedForPlayback === "function") {
    window.onMissionLoadedForPlayback(getCurrentMission() || normalized);
  }
}

function bindMapClick(plotId) {
  const el = document.getElementById(plotId);
  if (!el || el.__dashMapClick) return;
  el.__dashMapClick = true;
  el.on("plotly_click", (ev) => {
    if (!state.pickPhase || !ev?.points?.length) return;
    const p = ev.points[0];
    const x = Math.round(p.x);
    const y = Math.round(p.y);
    const { width, height } = getImageSize(state.lastResult || { map_background: state.mapConfig });
    if (x < 0 || x > width || y < 0 || y > height) return;

    if (state.pickPhase === "start") {
      $("replanStartX").value = x;
      $("replanStartY").value = y;
      state.pickPhase = null;
    } else if (state.pickPhase === "end") {
      $("replanEndX").value = x;
      $("replanEndY").value = y;
      state.pickPhase = null;
    }
    updatePickHint();
    if (getCurrentMission()) {
      const markers = { ...(getCurrentMission().markers || {}) };
      if ($("replanStartX").value !== "" && $("replanStartY").value !== "") {
        markers.start = {
          x: parseFloat($("replanStartX").value),
          y: parseFloat($("replanStartY").value),
        };
      }
      if ($("replanEndX").value !== "" && $("replanEndY").value !== "") {
        markers.end = {
          x: parseFloat($("replanEndX").value),
          y: parseFloat($("replanEndY").value),
        };
      }
      if (window.MissionStore) {
        MissionStore.updateMarkers(markers);
      } else {
        state.lastResult = { ...getCurrentMission(), markers };
      }
      refreshMapView();
    }
  });
}

function renderMission(result, targetId = "mapPlot", options = {}) {
  if (!result) return;
  readUiToState();
  updateMapModeHint();

  const normalized = normalizeMissionResult(result);
  const plot = buildPlotFromMission(normalized, options);
  const el = document.getElementById(targetId);
  if (!el) return;

  const draw = () => {
    bindMapClick(targetId);
    if (options.fullscreen) Plotly.Plots.resize(targetId);
  };

  const afterDraw = () => {
    draw();
    if (typeof window.resetPlayback === "function" && state.lastResult && targetId === "mapPlot") {
      window.resetPlayback(state.lastResult);
    }
  };

  if (state.plotReady[targetId] && el.data) {
    Plotly.react(targetId, plot.traces, plot.layout, plot.config).then(afterDraw);
  } else {
    Plotly.newPlot(targetId, plot.traces, plot.layout, plot.config).then(() => {
      state.plotReady[targetId] = true;
      afterDraw();
    });
  }
}

function openFullscreen() {
  if (!state.lastResult) return;
  const modal = $("mapFullscreenModal");
  document.body.classList.add("fullscreen-active");
  modal.classList.remove("hidden");
  modal.setAttribute("aria-hidden", "false");
  // Double rAF: first lets CSS layout settle, second measures correct dimensions
  requestAnimationFrame(() => requestAnimationFrame(() => {
    renderMission(state.lastResult, "mapPlotFs", { fullscreen: true });
  }));
}

function closeFullscreen() {
  const modal = $("mapFullscreenModal");
  document.body.classList.remove("fullscreen-active");
  modal.classList.add("hidden");
  modal.setAttribute("aria-hidden", "true");
  try {
    const fsEl = $("mapPlotFs");
    if (fsEl) fsEl.__dashMapClick = false;
    Plotly.purge("mapPlotFs");
    state.plotReady.mapPlotFs = false;
  } catch (_) {}
  // Restore main map to fill the now-wider space
  requestAnimationFrame(() => {
    try { Plotly.Plots.resize("mapPlot"); } catch (_) {}
  });
}


function buildCurrentPlanRequest() {
  const pipeline = getPipeline();
  const body = {
    pipeline,
    input_file: "",
    planner: "legacy",
    spacing: 50,
    map_mode: "image_overlay",
    weather_aware: $("weatherAwareToggle")?.checked === true,
    weather_weight: Math.max(0, parseFloat($("weatherWeightInput")?.value || "1") || 1),
  };
  if (pipeline === "image") {
    body.planner = "legacy";
    body.map_mode = "image_overlay";
    body.inspection_point_source = getInspectionPointSource();
    body.image_path = getImagePathForSource(body.inspection_point_source);
  } else {
    body.input_file = $("datasetSelect").value;
    body.planner = $("plannerSelect").value;
    body.spacing = parseFloat($("spacingInput").value) || 50;
    body.map_mode = getMapMode();
  }
  return body;
}


async function runPlanning() {
  const body = buildCurrentPlanRequest();
  const pipeline = body.pipeline;

  $("runBtn").disabled = true;
  setStatus(
    pipeline === "image" ? "正在生成/加载图像主线…" : "统一管线规划中…",
    "running"
  );
  if (AppPhaseManager) AppPhaseManager.beginPlanning();

  try {
    const res = await fetch("/api/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await res.json();
    if (!res.ok) {
      const detail =
        typeof data.detail === "string" ? data.detail : JSON.stringify(data.detail);
      throw new Error(detail || res.statusText);
    }

    const normalized = normalizeMissionResult(data);
    onMissionLoaded(normalized, { kind: "plan" });
    state.experiment.active = false;
    if (window.LayerManager) LayerManager.clearAllTransient();
    renderExperimentResult();
    if (pipeline === "image") {
      state.imageMissionAvailable = true;
      const st = await checkImageMission();
      state.imageMissionAvailable = st.available;
      updateReplanInputLimits();
    }

    const mission = getCurrentMission();
    renderStats(mission?.statistics);
    renderVisitOrder(mission?.visit_order);
    renderMission(mission);
    renderMeta(mission?.metadata || data.metadata || {});
    updateDownloadLinks(data.output_files || {}, pipeline);

    const gen = data.metadata?.image_generation;
    const msg = gen?.generated
      ? "已自动生成并加载图像主线"
      : pipeline === "image"
        ? "已加载 result/latest/mission_output.json"
        : `完成 · ${$("plannerSelect")?.selectedOptions?.[0]?.textContent || body.planner}`;
    setStatus(
      `${msg} · ${data.statistics?.num_segments ?? 0} 段`,
      "ok"
    );
  } catch (err) {
    setStatus(`错误: ${err.message}`, "err");
    console.error(err);
  } finally {
    $("runBtn").disabled = false;
  }
}

async function forceRegenerateImage() {
  $("forceRegenBtn").disabled = true;
  setStatus("正在重新生成任务（约 1–2 分钟）…", "running");
  try {
    const res = await fetch("/api/image-mission/generate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        force: true,
        inspection_point_source: getInspectionPointSource(),
        image_path: getImagePathForSource(getInspectionPointSource()),
      }),
    });
    const data = await res.json();
    if (!res.ok || !data.ok) {
      throw new Error(data.message || data.detail || "生成失败");
    }
    state.imageMissionAvailable = true;
    await runPlanning();
  } catch (err) {
    setStatus(`重新生成失败: ${err.message}`, "err");
  } finally {
    $("forceRegenBtn").disabled = false;
  }
}

document.addEventListener("DOMContentLoaded", async () => {
  try {
    await Promise.all([loadDatasets(), loadMapConfig()]);
    const st = await checkImageMission();
    state.imageMissionAvailable = st.available;
    applyLayerDefaultsForPipeline();
    applyPipelineUi();
    updateReplanInputLimits();
    renderExperimentResult();
    renderAdaptiveEvents();
    setAdaptiveStatus("正常巡航");
    renderSystemStatus();
    if (window.MissionStore?.subscribePhase) {
      MissionStore.subscribePhase(() => renderSystemStatus());
    }
    setStatus("就绪 · 点击「生成任务」开始", "");
  } catch (e) {
    setStatus(`初始化失败: ${e.message}`, "err");
  }

  $("pipelineSelect").addEventListener("change", () => {
    applyPipelineUi();
    if (window.MissionStore) {
      MissionStore.clear();
    } else {
      state.lastResult = null;
    }
    state.experiment.active = false;
    state.experiment.runA = null;
    state.experiment.runB = null;
    state.experiment.metrics = null;
    state.experiment.reasoning = "";
    renderExperimentResult();
    state.plotReady.mapPlot = false;
    if (typeof window.resetInspectionPlayback === "function") window.resetInspectionPlayback();
    setStatus("就绪", "");
  });

  $("inspectionPointSourceSelect")?.addEventListener("change", async () => {
    applyPipelineUi();
    updateSpacingControlsVisibility();
    const imagePath = getImagePathForSource(getInspectionPointSource());
    await loadMapConfig(imagePath);
    if (getCurrentMission()) refreshMapView();
  });

  $("runBtn").addEventListener("click", runPlanning);
  $("runWeatherExperimentBtn")?.addEventListener("click", runWeatherExperiment);
  $("forceRegenBtn").addEventListener("click", forceRegenerateImage);
  $("fullscreenBtn").addEventListener("click", openFullscreen);
  $("closeFullscreenBtn").addEventListener("click", closeFullscreen);
  $("modalBackdrop")?.addEventListener("click", closeFullscreen);
  $("runReplanBtn")?.addEventListener("click", runReplan);

  ["toggleBaseMap", "toggleInspect", "toggleConnect", "toggleWeatherLayer", "togglePointsLayer"].forEach((id) => {
    $(id)?.addEventListener("change", () => refreshMapView());
  });
  $("fullscreenBtnHeader")?.addEventListener("click", openFullscreen);
  $("toolsFullscreenBtn")?.addEventListener("click", openFullscreen);
  $("pointModeSelect")?.addEventListener("change", () => refreshMapView());
  $("weatherAwareToggle")?.addEventListener("change", () => {
    readUiToState();
    if (state.lastResult) renderMeta(state.lastResult.metadata || {});
  });
  $("weatherWeightInput")?.addEventListener("change", () => {
    readUiToState();
    if (state.lastResult) renderMeta(state.lastResult.metadata || {});
  });
  $("experimentOverlayToggle")?.addEventListener("change", () => {
    state.experiment.overlay = $("experimentOverlayToggle")?.checked !== false;
    refreshMapView();
  });
  $("dynamicWeatherToggle")?.addEventListener("change", () => {
    readUiToState();
    if (state.dynamicWeather.enabled) {
      state.dynamicWeather.lastTick = performance.now();
      startDynamicWeatherLoop();
      pushAdaptiveEvent("动态天气模拟已开启");
    } else {
      stopDynamicWeatherLoop();
      setAdaptiveStatus("天气监测暂停");
      pushAdaptiveEvent("动态天气模拟已暂停");
    }
  });
  $("adaptiveRiskThresholdInput")?.addEventListener("change", () => {
    readUiToState();
    pushAdaptiveEvent(`风险阈值更新为 ${state.dynamicWeather.riskThreshold.toFixed(2)}`);
  });
  $("predictionWindowSelect")?.addEventListener("change", () => {
    readUiToState();
    pushAdaptiveEvent(`预测窗口更新为 ${state.dynamicWeather.predictionWindow}s`);
    updatePredictiveMetrics(predictFutureWeatherRisk());
  });
  $("autoPredictiveReplanToggle")?.addEventListener("change", () => {
    readUiToState();
    pushAdaptiveEvent(
      state.dynamicWeather.autoPredictiveReplan ? "预测式自动重规划已开启" : "预测式自动重规划已关闭"
    );
  });
  $("replanCooldownSelect")?.addEventListener("change", () => {
    readUiToState();
    if (typeof pushAdaptiveEvent === "function") {
      pushAdaptiveEvent(`重规划冷却更新为 ${state.dynamicWeather.replanCooldownSec}s`);
    }
  });

  $("pickStartBtn")?.addEventListener("click", () => {
    state.pickPhase = "start";
    updatePickHint();
  });
  $("pickEndBtn")?.addEventListener("click", () => {
    state.pickPhase = "end";
    updatePickHint();
  });

  $("mapModeSelect")?.addEventListener("change", () => {
    if (state.lastResult && !isImagePipeline()) runPlanning();
    else {
      updateMapModeHint();
      refreshMapView();
    }
  });

  window.addEventListener("resize", () => {
    if ($("mapFullscreenModal") && !$("mapFullscreenModal").classList.contains("hidden")) {
      if (state.lastResult) {
        renderMission(state.lastResult, "mapPlotFs", { fullscreen: true });
      }
    } else {
      try { Plotly.Plots.resize("mapPlot"); } catch (_) {}
    }
  });

  // Resize main Plotly when the plot-stage changes width (e.g. inspection card shows/hides)
  const plotStageEl = document.querySelector(".plot-stage");
  if (plotStageEl && typeof ResizeObserver !== "undefined") {
    let _resizeRaf = null;
    new ResizeObserver(() => {
      cancelAnimationFrame(_resizeRaf);
      _resizeRaf = requestAnimationFrame(() => {
        try { Plotly.Plots.resize("mapPlot"); } catch (_) {}
      });
    }).observe(plotStageEl);
  }
});
