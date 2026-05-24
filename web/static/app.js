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
]);
const KEY_POINT_MAX = 30;

const state = {
  datasets: [],
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
    status: "正常巡航",
    events: [],
    replanTriggered: false,
    movementLogged: false,
    adaptivePath: null,
  },
  pickPhase: null,
  plotReady: { mapPlot: false, mapPlotFs: false },
};
window.state = state;

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

async function loadMapConfig() {
  try {
    const res = await fetch("/api/map/config");
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

function applyPipelineUi() {
  const image = isImagePipeline();
  $("pipelineBadge").textContent = image ? "图像 PNG" : "统一 JSON";

  const ds = $("datasetSelect");
  const pl = $("plannerSelect");
  const sp = $("spacingInput");
  const mm = $("mapModeSelect");
  const forceBtn = $("forceRegenBtn");

  if (image) {
    ds.innerHTML = '<option value="data/test.png">data/test.png</option>';
    ds.disabled = true;
    pl.innerHTML = '<option value="legacy">传统任务模式</option>';
    pl.disabled = true;
    sp.disabled = true;
    mm.innerHTML = '<option value="image_overlay" selected>图像叠加</option>';
    mm.disabled = true;
    $("runBtn").textContent = "生成/加载图像主线任务";
    forceBtn.classList.remove("hidden");
  } else {
    forceBtn.classList.add("hidden");
    $("dlLegacyHtml").classList.add("hidden");
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
    pl.disabled = false;
    pl.innerHTML = `
      <option value="baseline">基线方案</option>
      <option value="optimized">优化方案</option>
      <option value="dijkstra">Dijkstra 最短路</option>`;
    sp.disabled = false;
    mm.disabled = false;
    mm.innerHTML = `
      <option value="topology_only" selected>仅拓扑</option>
      <option value="image_overlay">样式化（无 test.png）</option>`;
    $("runBtn").textContent = "运行统一管线规划";
  }
  applyLayerDefaultsForPipeline();
  updateMapModeHint();
  updateReplanPanel();
  updateReplanInputLimits();
}

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

function readUiToState() {
  state.showBackground = $("toggleBaseMap")?.checked !== false;
  state.showInspect = $("toggleInspect")?.checked !== false;
  state.showConnect = $("toggleConnect")?.checked !== false;
  state.showWeatherLayer = $("toggleWeatherLayer")?.checked === true;
  state.pointMode = $("pointModeSelect")?.value || "key";
  state.weatherAware = $("weatherAwareToggle")?.checked === true;
  state.weatherWeight = Math.max(0, parseFloat($("weatherWeightInput")?.value || "1") || 1);
  state.experiment.overlay = $("experimentOverlayToggle")?.checked !== false;
  state.dynamicWeather.enabled = $("dynamicWeatherToggle")?.checked !== false;
  state.dynamicWeather.riskThreshold = Math.max(
    0.1,
    parseFloat($("adaptiveRiskThresholdInput")?.value || "0.72") || 0.72
  );
}

function syncStateToUi() {
  if ($("toggleBaseMap")) $("toggleBaseMap").checked = state.showBackground;
  if ($("toggleInspect")) $("toggleInspect").checked = state.showInspect;
  if ($("toggleConnect")) $("toggleConnect").checked = state.showConnect;
  if ($("toggleWeatherLayer")) $("toggleWeatherLayer").checked = state.showWeatherLayer;
  if ($("pointModeSelect")) $("pointModeSelect").value = state.pointMode;
  if ($("weatherAwareToggle")) $("weatherAwareToggle").checked = state.weatherAware;
  if ($("weatherWeightInput")) $("weatherWeightInput").value = String(state.weatherWeight);
  if ($("experimentOverlayToggle")) $("experimentOverlayToggle").checked = state.experiment.overlay;
  if ($("dynamicWeatherToggle")) $("dynamicWeatherToggle").checked = state.dynamicWeather.enabled;
  if ($("adaptiveRiskThresholdInput")) $("adaptiveRiskThresholdInput").value = String(state.dynamicWeather.riskThreshold);
}

function applyLayerDefaultsForPipeline() {
  if (isImagePipeline()) {
    state.showBackground = false;
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

function normalizeMissionResult(result) {
  const pipeline = result?.metadata?.pipeline || getPipeline();
  const out = { ...result };
  out.metadata = { ...(result.metadata || {}), pipeline };
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
  $("statusBox").textContent = text;
  $("statusBox").className = "status " + cls;
}

function updateMapModeHint() {
  const hint = $("mapModeHint");
  if (!hint) return;
  if (isImagePipeline()) {
    const avail = state.imageMissionAvailable ? "任务已就绪" : "将自动生成任务";
    hint.textContent = `图像主线 · ${avail} · 默认无底图`;
    $("mapPlot")?.classList.add("overlay-mode");
  } else {
    hint.textContent =
      getMapMode() === "topology_only"
        ? "统一管线 · 拓扑自适应"
        : "统一管线 · 无底图";
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
  const pipelineMap = {
    image: "图像主线",
    unified: "统一管线",
  };
  const plannerMap = {
    baseline: "基线规划",
    optimized: "全局优化规划",
    dijkstra: "全局优化规划",
    legacy: "全局优化规划",
    start_end_replan: "全局优化规划",
  };
  const pipelineText = pipelineMap[metadata.pipeline] || "—";
  const plannerText = plannerMap[metadata.planner] || "全局优化规划";
  const lines = [
    `管线: ${pipelineText}`,
    `规划器: ${plannerText}`,
    metadata.planning_spacing != null ? `巡检点间距: ${metadata.planning_spacing}px` : null,
    `天气感知: ${metadata.weather_mode === "on" ? "开启" : "关闭"}`,
    metadata.weather_mode === "on" ? `天气权重: ${metadata.weather_weight ?? 1.0}` : null,
  ].filter(Boolean);
  $("metaBox").innerHTML = lines.map((l) => `<div>${escapeHtml(l)}</div>`).join("");
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;");
}

function renderStats(statistics) {
  const s = statistics || {};
  const items = [
    ["total_length", "总长度", s.total_length, " px"],
    ["inspect_length", "巡检长度", s.inspect_length, " px"],
    ["connect_length", "连接长度", s.connect_length, " px"],
    ["connect_ratio", "连接占比", ((s.connect_ratio || 0) * 100).toFixed(1), "%"],
    ["num_segments", "区段数", s.num_segments, ""],
    ["num_inspection_points", "巡检点数", s.num_inspection_points, ""],
    ["weather_penalty_total", "天气惩罚", s.weather_penalty_total ?? 0, ""],
    ["total_cost", "实际总代价", s.total_cost ?? s.total_length ?? 0, ""],
    ["weather_affected_edges", "受天气影响区段", s.weather_affected_edges ?? 0, ""],
    ["risky_distance", "高风险穿越长度", s.risky_distance ?? 0, " px"],
    ["weather_mode", "天气感知", s.weather_mode === "on" ? "开启" : "关闭", ""],
  ];
  $("statCards").innerHTML = items
    .map(
      ([, label, val, unit]) => `
    <div class="card"><div class="label">${label}</div>
    <div class="value">${val ?? "—"}${unit}</div></div>`
    )
    .join("");
}

function renderVisitOrder(visitOrder) {
  const chips = $("visitChips");
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

function getWeatherZones(result) {
  const zones = state.dynamicWeather.zones?.length
    ? state.dynamicWeather.zones
    : result?.weather_zones || result?.metadata?.weather_zones || [];
  return Array.isArray(zones) ? zones : [];
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

function pathAsHashSet(points, stride = 3, grid = 10) {
  const st = new Set();
  for (let i = 0; i < points.length; i += stride) {
    const x = Math.round(points[i][0] / grid);
    const y = Math.round(points[i][1] / grid);
    st.add(`${x},${y}`);
  }
  return st;
}

function computeRerouteRatio(resultA, resultB) {
  const aPts = flattenMissionPathPoints(resultA);
  const bPts = flattenMissionPathPoints(resultB);
  if (!aPts.length || !bPts.length) return 0;
  const setA = pathAsHashSet(aPts);
  const setB = pathAsHashSet(bPts);
  const union = new Set([...setA, ...setB]);
  let inter = 0;
  setA.forEach((v) => {
    if (setB.has(v)) inter += 1;
  });
  const sim = union.size > 0 ? inter / union.size : 1;
  return Math.max(0, Math.min(100, (1 - sim) * 100));
}

function calcRouteTraceArrays(result) {
  const x = [];
  const y = [];
  (result?.segments || []).forEach((seg) => {
    const geom = seg.geometry_2d || [];
    if (geom.length < 2) return;
    x.push(...geom.map((p) => p[0]), null);
    y.push(...geom.map((p) => p[1]), null);
  });
  if (x.length && x[x.length - 1] === null) x.pop();
  if (y.length && y[y.length - 1] === null) y.pop();
  return { x, y };
}

function buildExperimentMetrics(resultA, resultB) {
  const a = resultA?.statistics || {};
  const b = resultB?.statistics || {};
  const totalA = Number(a.total_length || 0);
  const totalB = Number(b.total_length || 0);
  const penaltyB = Number(b.weather_penalty_total || 0);
  const costA = Number(a.total_cost != null ? a.total_cost : totalA);
  const costB = Number(b.total_cost != null ? b.total_cost : totalB + penaltyB);
  const riskyA = Number(a.risky_distance || 0);
  const riskyB = Number(b.risky_distance || 0);
  return {
    total_distance_a: totalA,
    total_distance_b: totalB,
    weather_penalty_total: penaltyB,
    total_cost_a: costA,
    total_cost_b: costB,
    weather_affected_edges_a: Number(a.weather_affected_edges || 0),
    weather_affected_edges_b: Number(b.weather_affected_edges || 0),
    risky_distance_a: riskyA,
    risky_distance_b: riskyB,
    reroute_ratio: computeRerouteRatio(resultA, resultB),
    high_risk_bypass: riskyB + 1e-6 < riskyA,
  };
}

function fmtNum(v, unit = "") {
  const n = Number(v || 0);
  return `${n.toFixed(2)}${unit}`;
}

function buildPlannerReasoning(resultA, resultB, metrics) {
  const zones = resultB?.metadata?.weather_summary?.top_zones || [];
  const segs = resultB?.metadata?.weather_summary?.top_segments || [];
  if (!zones.length) {
    return `系统未检测到显著天气惩罚，路径变化约 ${metrics.reroute_ratio.toFixed(1)}%。`;
  }
  const z = zones[0];
  const labelMap = { wind: "强风区", rain: "降雨区", visibility: "低能见度", risk: "风险区域" };
  const zLabel = labelMap[String(z.type || "").toLowerCase()] || "天气区域";
  const segText = segs.length ? `优先规避区段：${segs.slice(0, 2).map((s) => s.segment_id).filter(Boolean).join("、") || "关键连接段"}` : "规避高风险连接段";
  return [
    `系统检测到主要影响区域：${zLabel}（强度 ${Number(z.severity || 0).toFixed(2)}）。`,
    `该区域累计惩罚约 ${fmtNum(z.penalty)}，触发绕行策略。`,
    `${segText}。`,
    `对比结果：路径变化 ${metrics.reroute_ratio.toFixed(1)}%，总距离增量 ${fmtNum(metrics.total_distance_b - metrics.total_distance_a, " px")}。`,
  ].join(" ");
}

function renderExperimentResult() {
  const host = $("experimentCards");
  const reason = $("plannerReasoningBox");
  if (!host || !reason) return;
  if (!state.experiment.active || !state.experiment.metrics) {
    host.innerHTML = "";
    reason.textContent = "尚未运行实验";
    return;
  }
  const m = state.experiment.metrics;
  host.innerHTML = [
    `<div class="card"><div class="label">天气关闭总距离</div><div class="value">${fmtNum(m.total_distance_a, " px")}</div></div>`,
    `<div class="card"><div class="label">天气开启总距离</div><div class="value">${fmtNum(m.total_distance_b, " px")}</div></div>`,
    `<div class="card"><div class="label">天气惩罚</div><div class="value">${fmtNum(m.weather_penalty_total)}</div></div>`,
    `<div class="card"><div class="label">天气关闭总代价</div><div class="value">${fmtNum(m.total_cost_a)}</div></div>`,
    `<div class="card"><div class="label">天气开启总代价</div><div class="value">${fmtNum(m.total_cost_b)}</div></div>`,
    `<div class="card"><div class="label">受天气影响区段</div><div class="value">${m.weather_affected_edges_b}</div></div>`,
    `<div class="card"><div class="label">高风险穿越长度</div><div class="value">${fmtNum(m.risky_distance_b, " px")}</div></div>`,
    `<div class="card"><div class="label">路径变化比例</div><div class="value">${m.reroute_ratio.toFixed(1)}%</div></div>`,
  ].join("");
  reason.textContent = state.experiment.reasoning || "实验已完成。";
}

function weatherColor(type, severity = 0.5) {
  const t = String(type || "").toLowerCase();
  const sev = Math.max(0, Math.min(1, Number(severity || 0)));
  const alpha = (0.12 + sev * 0.30).toFixed(3);
  const glowAlpha = (0.18 + sev * 0.55).toFixed(3);
  if (t === "wind") return { fill: `rgba(34, 211, 238, ${alpha})`, line: `rgba(34, 211, 238, ${glowAlpha})`, label: "强风区" };
  if (t === "rain") return { fill: `rgba(59, 130, 246, ${alpha})`, line: `rgba(59, 130, 246, ${glowAlpha})`, label: "降雨区" };
  if (t === "visibility") return { fill: `rgba(168, 85, 247, ${alpha})`, line: `rgba(168, 85, 247, ${glowAlpha})`, label: "低能见度" };
  return { fill: `rgba(239, 68, 68, ${alpha})`, line: `rgba(239, 68, 68, ${glowAlpha})`, label: "风险区域" };
}

function buildWeatherZoneTraces(result) {
  if (!state.showWeatherLayer) return [];
  const zones = getWeatherZones(result);
  if (!zones.length) return [];
  const traces = [];
  const lx = [];
  const ly = [];
  const lt = [];
  zones.forEach((z, idx) => {
    const center = z.center || [];
    const radius = Number(z.radius || 0);
    if (center.length < 2 || !Number.isFinite(radius) || radius <= 0) return;
    const cx = Number(center[0]);
    const cy = Number(center[1]);
    const sev = Number(z.severity || 0);
    const c = weatherColor(z.type, sev);
    const n = 56;
    const xs = [];
    const ys = [];
    for (let i = 0; i <= n; i += 1) {
      const a = (i / n) * Math.PI * 2;
      xs.push(cx + radius * Math.cos(a));
      ys.push(cy + radius * Math.sin(a));
    }
    const lineW = 1.2 + Math.max(0, Math.min(1, sev)) * 2.4;
    traces.push({
      x: xs,
      y: ys,
      mode: "lines",
      fill: "toself",
      fillcolor: c.fill,
      line: { color: c.line, width: lineW },
      name: `天气区 ${idx + 1}`,
      hovertemplate: `${c.label}<br>强度: ${sev.toFixed(2)}<br>半径: ${radius.toFixed(0)}px<extra></extra>`,
      showlegend: false,
    });
    lx.push(cx);
    ly.push(cy);
    lt.push(c.label);
  });
  if (lx.length) {
    traces.push({
      x: lx,
      y: ly,
      mode: "text",
      name: "天气标签",
      text: lt,
      textfont: { size: 10, color: "#e2e8f0" },
      textposition: "middle center",
      hoverinfo: "skip",
      showlegend: false,
    });
  }
  return traces;
}

function buildExperimentOverlayTraces() {
  if (!state.experiment.active || !state.experiment.overlay) return [];
  const runA = state.experiment.runA;
  const runB = state.experiment.runB;
  if (!runA || !runB) return [];
  const a = calcRouteTraceArrays(runA);
  const b = calcRouteTraceArrays(runB);
  const traces = [];
  if (a.x.length) {
    traces.push({
      x: a.x,
      y: a.y,
      mode: "lines",
      name: "A 天气关闭",
      line: { color: "rgba(148,163,184,0.88)", width: 2.4, dash: "dot" },
      hoverinfo: "skip",
      showlegend: true,
      legendgroup: "expab",
    });
  }
  if (b.x.length) {
    traces.push({
      x: b.x,
      y: b.y,
      mode: "lines",
      name: "B 天气开启",
      line: { color: "rgba(37,99,235,0.95)", width: 2.8 },
      hoverinfo: "skip",
      showlegend: true,
      legendgroup: "expab",
    });
  }
  return traces;
}

function buildAdaptiveReplanTraces() {
  const path = state.dynamicWeather.adaptivePath;
  if (!path || path.length < 2) return [];
  return [{
    x: path.map((p) => p[0]),
    y: path.map((p) => p[1]),
    mode: "lines",
    name: "自适应重规划路径",
    line: { color: "rgba(34, 197, 94, 0.95)", width: 4, dash: "dashdot" },
    hovertemplate: "动态天气触发的新连接路径<extra></extra>",
    showlegend: true,
    legendgroup: "adaptive",
  }];
}

function cloneWeatherZones(zones) {
  return (zones || []).map((z) => ({
    ...z,
    center: [...(z.center || [0, 0])],
    velocity: [...(z.velocity || [0, 0])],
  }));
}

function resetDynamicWeather(result) {
  const zones = cloneWeatherZones(result?.weather_zones || []);
  state.dynamicWeather.baseZones = cloneWeatherZones(zones);
  state.dynamicWeather.zones = cloneWeatherZones(zones);
  state.dynamicWeather.elapsed = 0;
  state.dynamicWeather.lastTick = performance.now();
  state.dynamicWeather.replanTriggered = false;
  state.dynamicWeather.movementLogged = false;
  state.dynamicWeather.adaptivePath = null;
  state.dynamicWeather.events = [];
  setAdaptiveStatus("天气监测");
  pushAdaptiveEvent("动态天气监测初始化");
  startDynamicWeatherLoop();
}

function updateDynamicWeatherZones(dt) {
  const bounds = state.lastResult?.bounds || { x_range: [0, 1000], y_range: [1000, 0] };
  const xr = bounds.x_range || [0, 1000];
  const yr = bounds.y_range || [1000, 0];
  const xmin = Math.min(xr[0], xr[1]);
  const xmax = Math.max(xr[0], xr[1]);
  const ymin = Math.min(yr[0], yr[1]);
  const ymax = Math.max(yr[0], yr[1]);
  state.dynamicWeather.elapsed += dt;
  state.dynamicWeather.zones = cloneWeatherZones(state.dynamicWeather.zones).map((z) => {
    if (!z.dynamic) return z;
    const vel = z.velocity || [0, 0];
    z.center[0] += Number(vel[0] || 0) * dt;
    z.center[1] += Number(vel[1] || 0) * dt;
    z.radius = Math.max(10, Number(z.radius || 0) + Number(z.expand_rate || 0) * dt);
    z.severity = Math.max(0, Math.min(1, Number(z.severity || 0) + Number(z.severity_rate || 0) * dt));
    if (z.center[0] < xmin || z.center[0] > xmax) {
      z.velocity[0] = -Number(z.velocity[0] || 0);
      z.center[0] = Math.max(xmin, Math.min(xmax, z.center[0]));
    }
    if (z.center[1] < ymin || z.center[1] > ymax) {
      z.velocity[1] = -Number(z.velocity[1] || 0);
      z.center[1] = Math.max(ymin, Math.min(ymax, z.center[1]));
    }
    return z;
  });
  if (!state.dynamicWeather.movementLogged && state.dynamicWeather.elapsed >= 2) {
    const moving = state.dynamicWeather.zones.find((z) => z.dynamic && (Math.abs(Number(z.velocity?.[0] || 0)) + Math.abs(Number(z.velocity?.[1] || 0)) > 0));
    if (moving) {
      const label = weatherColor(moving.type, moving.severity).label;
      pushAdaptiveEvent(`检测到${label}移动`);
      state.dynamicWeather.movementLogged = true;
    }
  }
}

function pointZoneRisk(point, zone) {
  if (!point || !zone?.center) return 0;
  const dx = point[0] - Number(zone.center[0]);
  const dy = point[1] - Number(zone.center[1]);
  const r = Number(zone.radius || 0);
  if (r <= 0 || Math.hypot(dx, dy) > r) return 0;
  const typeBoost = String(zone.type || "").toLowerCase() === "risk" ? 1.25 : 1;
  return Number(zone.severity || 0) * typeBoost;
}

function getPlaybackUavPoint() {
  const visual = typeof window.getPlaybackVisualState === "function"
    ? window.getPlaybackVisualState()
    : null;
  const p = visual?.uavPos;
  if (!p) return null;
  return [Number(p.x), Number(p.y)];
}

function nearestPathIndex(path, point) {
  if (!path?.length || !point) return 0;
  let best = 0;
  let bestD = Infinity;
  for (let i = 0; i < path.length; i += 1) {
    const d = Math.hypot(path[i][0] - point[0], path[i][1] - point[1]);
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  }
  return best;
}

function analyzeFutureWeatherRisk() {
  const uav = getPlaybackUavPoint();
  const path = flattenMissionPathPoints(state.lastResult);
  if (!uav || path.length < 2 || !state.dynamicWeather.zones.length) return null;
  const start = nearestPathIndex(path, uav);
  const lookahead = path.slice(start, Math.min(path.length, start + 90));
  let maxRisk = 0;
  let hitZone = null;
  let hitPoint = null;
  for (const p of lookahead) {
    for (const z of state.dynamicWeather.zones) {
      const risk = pointZoneRisk(p, z);
      if (risk > maxRisk) {
        maxRisk = risk;
        hitZone = z;
        hitPoint = p;
      }
    }
  }
  return { maxRisk, hitZone, hitPoint, startIndex: start, uav, path };
}

function buildAdaptivePathAroundZone(uav, hitPoint, zone) {
  if (!uav || !hitPoint || !zone?.center) return null;
  const cx = Number(zone.center[0]);
  const cy = Number(zone.center[1]);
  const radius = Number(zone.radius || 0) + 65;
  const vx = hitPoint[0] - cx;
  const vy = hitPoint[1] - cy;
  const len = Math.hypot(vx, vy) || 1;
  const nx = -vy / len;
  const ny = vx / len;
  const side = ((uav[0] - cx) * nx + (uav[1] - cy) * ny) >= 0 ? 1 : -1;
  const wp1 = [cx + nx * radius * side, cy + ny * radius * side];
  const wp2 = [hitPoint[0] + nx * radius * 0.55 * side, hitPoint[1] + ny * radius * 0.55 * side];
  return [uav, wp1, wp2, hitPoint];
}

function adaptiveStatusText(status) {
  return status || "正常巡航";
}

function setAdaptiveStatus(status) {
  state.dynamicWeather.status = adaptiveStatusText(status);
  const el = $("adaptiveStatus");
  if (el) el.textContent = state.dynamicWeather.status;
}

function pushAdaptiveEvent(text) {
  const t = Math.max(0, Math.round(state.dynamicWeather.elapsed || 0));
  state.dynamicWeather.events.unshift({ t, text });
  state.dynamicWeather.events = state.dynamicWeather.events.slice(0, 8);
  renderAdaptiveEvents();
}

function renderAdaptiveEvents() {
  const el = $("adaptiveEventList");
  if (!el) return;
  const events = state.dynamicWeather.events;
  if (!events.length) {
    el.innerHTML = '<li><span>T+00:00</span><b>动态天气监测待命</b></li>';
    return;
  }
  el.innerHTML = events
    .map((e) => `<li><span>T+${String(e.t).padStart(2, "0")}s</span><b>${escapeHtml(e.text)}</b></li>`)
    .join("");
}

function maybeTriggerAdaptiveReplan() {
  const visual = typeof window.getPlaybackVisualState === "function" ? window.getPlaybackVisualState() : null;
  if (!visual || visual.status !== "playing") return;
  setAdaptiveStatus("风险分析");
  const risk = analyzeFutureWeatherRisk();
  if (!risk?.hitZone) {
    setAdaptiveStatus("正常巡航");
    return;
  }
  if (risk.maxRisk > state.dynamicWeather.riskThreshold) {
    if (!state.dynamicWeather.replanTriggered) {
      const label = weatherColor(risk.hitZone.type, risk.hitZone.severity).label;
      pushAdaptiveEvent(`检测到${label}进入前方航迹`);
      pushAdaptiveEvent(`风险 ${risk.maxRisk.toFixed(2)} 超过阈值`);
      setAdaptiveStatus("自动重规划");
      state.dynamicWeather.adaptivePath = buildAdaptivePathAroundZone(risk.uav, risk.hitPoint, risk.hitZone);
      state.dynamicWeather.replanTriggered = true;
      pushAdaptiveEvent("触发自动重规划");
      pushAdaptiveEvent("新路径生成完成");
      setAdaptiveStatus("路径更新完成");
      updateAdaptivePlotTraces();
    }
  } else {
    setAdaptiveStatus("天气监测");
  }
}

function startDynamicWeatherLoop() {
  stopDynamicWeatherLoop();
  state.dynamicWeather.timer = window.setInterval(() => {
    if (!state.lastResult || !state.dynamicWeather.enabled) return;
    const now = performance.now();
    const dt = Math.min(2.0, Math.max(0.2, (now - state.dynamicWeather.lastTick) / 1000));
    state.dynamicWeather.lastTick = now;
    updateDynamicWeatherZones(dt);
    maybeTriggerAdaptiveReplan();
    updateWeatherPlotTraces();
  }, 900);
}

function stopDynamicWeatherLoop() {
  if (state.dynamicWeather.timer) {
    clearInterval(state.dynamicWeather.timer);
    state.dynamicWeather.timer = null;
  }
}

function replaceTraceSet(plotId, predicate, traces) {
  const el = document.getElementById(plotId);
  if (!el?.data || !state.plotReady[plotId]) return;
  const keep = el.data.filter((t) => !predicate(t));
  Plotly.react(plotId, keep.concat(traces), el.layout, { responsive: plotId !== "mapPlotFs", displayModeBar: true, scrollZoom: true });
}

function updateWeatherPlotTraces() {
  const traces = buildWeatherZoneTraces(state.lastResult);
  ["mapPlot", "mapPlotFs"].forEach((plotId) => {
    replaceTraceSet(plotId, (t) => String(t.name || "").startsWith("天气区 ") || t.name === "天气标签", traces);
  });
}

function updateAdaptivePlotTraces() {
  const traces = buildAdaptiveReplanTraces();
  ["mapPlot", "mapPlotFs"].forEach((plotId) => {
    replaceTraceSet(plotId, (t) => t.name === "自适应重规划路径", traces);
  });
}

/** 统一 traces：原始 mission / Unified / Replan 共用 */
function buildMissionTraces(result) {
  const segments = result.segments || [];
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
  traces.push(...buildWeatherZoneTraces(result));
  traces.push(...buildExperimentOverlayTraces());
  traces.push(...buildAdaptiveReplanTraces());

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
    result.inspection_points || [],
    state.pointMode
  );
  if (visiblePts.length) {
    const isAll = state.pointMode === "all";
    const playbackVisual = typeof window.getPlaybackVisualState === "function"
      ? window.getPlaybackVisualState()
      : null;
    const visited = playbackVisual?.visitedIds || new Set();
    const currentId = playbackVisual?.currentPointId || null;
    const totalPoints = (result.inspection_points || []).length || visiblePts.length;
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
        name: "巡检点",
        marker: {
          size: isAll ? 5 : 6,
          color: "#38bdf8",
          opacity: isAll ? 0.5 : 0.82,
          line: { width: isAll ? 0.8 : 1.2, color: "#dbeafe" },
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
          size: isAll ? 5 : 7,
          color: "#22c55e",
          opacity: 0.62,
          line: { width: 1, color: "#14532d" },
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
        name: "当前巡检点",
        marker: {
          size: 22,
          color: "rgba(250, 204, 21, 0.26)",
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
          size: 12,
          color: "#facc15",
          opacity: 1,
          line: { width: 2.2, color: "#ffffff" },
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
  if (!state.showBackground) return [];
  const pipeline = result.metadata?.pipeline || getPipeline();
  if (pipeline !== "image") return [];

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
    const { width, height } = getImageSize(result);
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

function onMissionLoaded(result) {
  resetDynamicWeather(result);
  if (typeof window.onMissionLoadedForPlayback === "function") {
    window.onMissionLoadedForPlayback(result);
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
    if (state.lastResult) {
      const markers = { ...(state.lastResult.markers || {}) };
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
      state.lastResult = { ...state.lastResult, markers };
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

async function runReplan() {
  if (!isImagePipeline()) {
    setStatus("重规划仅支持 Image Pipeline", "err");
    return;
  }
  readUiToState();
  const check = validateReplanCoords();
  if (!check.ok) {
    setStatus(check.message, "err");
    return;
  }

  const planningSpacing = parseFloat($("planningSpacingInput")?.value) || 70;

  $("runReplanBtn").disabled = true;
  setStatus("起终点重规划中…", "running");

  try {
    const res = await fetch("/api/replan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        pipeline: "image",
        start: check.start,
        end: check.end,
        planning_spacing: planningSpacing,
        weather_aware: $("weatherAwareToggle")?.checked === true,
        weather_weight: Math.max(0, parseFloat($("weatherWeightInput")?.value || "1") || 1),
      }),
    });
    const data = await res.json();
    if (!res.ok || !data.ok) {
      throw new Error(data.message || data.detail || res.statusText);
    }

    const dashboard = normalizeMissionResult(data.dashboard || data);
    state.lastResult = dashboard;
    state.experiment.active = false;
    renderExperimentResult();
    renderStats(dashboard.statistics);
    renderVisitOrder(dashboard.visit_order);
    renderMission(dashboard);
    onMissionLoaded(dashboard);
    renderMeta(dashboard.metadata || {});
    updateDownloadLinks(dashboard.output_files || {}, "image");

    const connected = dashboard.metadata?.end_connected;
    setStatus(
      `重规划完成 · ${dashboard.statistics?.num_segments ?? 0} 段` +
        (connected ? " · 已连接终点" : " · 终点未连接"),
      connected ? "ok" : "err"
    );
  } catch (err) {
    setStatus(`重规划失败: ${err.message}`, "err");
    console.error(err);
  } finally {
    $("runReplanBtn").disabled = false;
  }
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
  } else {
    body.input_file = $("datasetSelect").value;
    body.planner = $("plannerSelect").value;
    body.spacing = parseFloat($("spacingInput").value) || 50;
    body.map_mode = getMapMode();
  }
  return body;
}

async function executeExperimentRun(baseBody, weatherAware) {
  const pipeline = baseBody.pipeline;
  if (pipeline === "image") {
    const check = validateReplanCoords();
    if (check.ok) {
      const planningSpacing = parseFloat($("planningSpacingInput")?.value) || 70;
      const rr = await fetch("/api/replan", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          pipeline: "image",
          start: check.start,
          end: check.end,
          planning_spacing: planningSpacing,
          weather_aware: weatherAware,
          weather_weight: baseBody.weather_weight,
        }),
      });
      const rj = await rr.json();
      if (!rr.ok || !rj.ok) throw new Error(rj.message || rj.detail || rr.statusText);
      return normalizeMissionResult(rj.dashboard || rj);
    }
  }
  const body = { ...baseBody, weather_aware: weatherAware };
  const res = await fetch("/api/plan", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await res.json();
  if (!res.ok) {
    const detail = typeof data.detail === "string" ? data.detail : JSON.stringify(data.detail);
    throw new Error(detail || res.statusText);
  }
  return normalizeMissionResult(data);
}

async function runWeatherExperiment() {
  const btn = $("runWeatherExperimentBtn");
  if (btn) btn.disabled = true;
  setStatus("天气实验运行中：A(关闭) vs B(开启)…", "running");
  try {
    readUiToState();
    const baseBody = buildCurrentPlanRequest();
    const runA = await executeExperimentRun(baseBody, false);
    const runB = await executeExperimentRun(baseBody, true);
    const metrics = buildExperimentMetrics(runA, runB);
    const reasoning = buildPlannerReasoning(runA, runB, metrics);

    state.experiment.active = true;
    state.experiment.runA = runA;
    state.experiment.runB = runB;
    state.experiment.metrics = metrics;
    state.experiment.reasoning = reasoning;
    state.experiment.overlay = $("experimentOverlayToggle")?.checked !== false;

    state.lastResult = runB;
    renderStats(runB.statistics);
    renderVisitOrder(runB.visit_order);
    renderMeta(runB.metadata || {});
    renderExperimentResult();
    refreshMapView();

    const bypassText = metrics.high_risk_bypass ? "已触发高风险绕行" : "未触发明显高风险绕行";
    setStatus(
      `实验完成 · 路径变化 ${metrics.reroute_ratio.toFixed(1)}% · ${bypassText}`,
      "ok"
    );
  } catch (err) {
    setStatus(`天气实验失败: ${err.message}`, "err");
  } finally {
    if (btn) btn.disabled = false;
  }
}

async function runPlanning() {
  const body = buildCurrentPlanRequest();
  const pipeline = body.pipeline;

  $("runBtn").disabled = true;
  setStatus(
    pipeline === "image" ? "正在生成/加载图像主线…" : "统一管线规划中…",
    "running"
  );

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

    state.lastResult = normalizeMissionResult(data);
    state.experiment.active = false;
    renderExperimentResult();
    if (pipeline === "image") {
      state.imageMissionAvailable = true;
      const st = await checkImageMission();
      state.imageMissionAvailable = st.available;
      updateReplanInputLimits();
    }

    renderStats(state.lastResult.statistics);
    renderVisitOrder(state.lastResult.visit_order);
    renderMission(state.lastResult);
    onMissionLoaded(state.lastResult);
    renderMeta(data.metadata || {});
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
  setStatus("强制重新生成图像主线（约 1–2 分钟）…", "running");
  try {
    const res = await fetch("/api/image-mission/generate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ force: true }),
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
    setStatus("就绪 · 运行 python app.py 后点击加载即可", "");
  } catch (e) {
    setStatus(`初始化失败: ${e.message}`, "err");
  }

  $("pipelineSelect").addEventListener("change", () => {
    applyPipelineUi();
    state.lastResult = null;
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

  $("runBtn").addEventListener("click", runPlanning);
  $("runWeatherExperimentBtn")?.addEventListener("click", runWeatherExperiment);
  $("forceRegenBtn").addEventListener("click", forceRegenerateImage);
  $("fullscreenBtn").addEventListener("click", openFullscreen);
  $("closeFullscreenBtn").addEventListener("click", closeFullscreen);
  $("modalBackdrop")?.addEventListener("click", closeFullscreen);
  $("runReplanBtn")?.addEventListener("click", runReplan);

  ["toggleBaseMap", "toggleInspect", "toggleConnect", "toggleWeatherLayer"].forEach((id) => {
    $(id)?.addEventListener("change", () => refreshMapView());
  });
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
    else updateMapModeHint();
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
