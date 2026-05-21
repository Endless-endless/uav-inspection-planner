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
  showBackground: true,
  showInspect: true,
  showConnect: true,
  pointMode: "key",
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
  $("pipelineBadge").textContent = image ? "Image PNG" : "Unified JSON";

  const ds = $("datasetSelect");
  const pl = $("plannerSelect");
  const sp = $("spacingInput");
  const mm = $("mapModeSelect");
  const forceBtn = $("forceRegenBtn");

  if (image) {
    ds.innerHTML = '<option value="data/test.png">data/test.png</option>';
    ds.disabled = true;
    pl.innerHTML = '<option value="legacy">Legacy Mission</option>';
    pl.disabled = true;
    sp.disabled = true;
    mm.innerHTML = '<option value="image_overlay" selected>Image Overlay</option>';
    mm.disabled = true;
    $("runBtn").textContent = "生成/加载图像主线 Mission";
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
      <option value="baseline">Baseline</option>
      <option value="optimized">Optimized</option>
      <option value="dijkstra">Dijkstra</option>`;
    sp.disabled = false;
    mm.disabled = false;
    mm.innerHTML = `
      <option value="topology_only" selected>Topology Only</option>
      <option value="image_overlay">Styled（无 test.png）</option>`;
    $("runBtn").textContent = "运行 Unified 规划";
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
  state.pointMode = $("pointModeSelect")?.value || "key";
}

function syncStateToUi() {
  if ($("toggleBaseMap")) $("toggleBaseMap").checked = state.showBackground;
  if ($("toggleInspect")) $("toggleInspect").checked = state.showInspect;
  if ($("toggleConnect")) $("toggleConnect").checked = state.showConnect;
  if ($("pointModeSelect")) $("pointModeSelect").value = state.pointMode;
}

function applyLayerDefaultsForPipeline() {
  if (isImagePipeline()) {
    state.showBackground = true;
    state.pointMode = "key";
  } else {
    state.showBackground = false;
    state.pointMode = "all";
  }
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
    return { ok: false, message: "请填写完整的 Start/End 坐标" };
  }
  const { width, height } = getImageSize(state.lastResult || { map_background: state.mapConfig });
  if (sx < 0 || sx > width || ex < 0 || ex > width || sy < 0 || sy > height || ey < 0 || ey > height) {
    return {
      ok: false,
      message: `Start/End coordinate out of image bounds: x must be [0,${width}], y must be [0,${height}]`,
    };
  }
  return { ok: true, start: [sx, sy], end: [ex, ey] };
}

function normalizeMissionResult(result) {
  const pipeline = result?.metadata?.pipeline || getPipeline();
  const out = { ...result };
  out.metadata = { ...(result.metadata || {}), pipeline };

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
    const sz = state.mapConfig?.available
      ? `${state.mapConfig.width}×${state.mapConfig.height}`
      : "—";
    const avail = state.imageMissionAvailable ? "mission 已就绪" : "将自动生成 mission";
    hint.textContent = `图像主线 · ${avail} · 底图 ${sz}`;
    $("mapPlot")?.classList.add("overlay-mode");
  } else {
    hint.textContent =
      getMapMode() === "topology_only"
        ? "Unified · 拓扑自适应"
        : "Unified · 无底图";
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
  const gen = metadata.image_generation;
  const lines = [
    `pipeline: ${metadata.pipeline || "—"}`,
    `source: ${metadata.source || "—"}`,
    gen ? `generation: ${gen.generated ? "新生成" : "复用缓存"}` : null,
    gen?.message ? `gen_msg: ${gen.message}` : null,
    `planner: ${metadata.planner || "—"}`,
    metadata.connect_planner ? `connect: ${metadata.connect_planner}` : null,
    metadata.map_mode_note ? `map: ${metadata.map_mode_note}` : null,
    metadata.end_connected != null ? `end_connected: ${metadata.end_connected}` : null,
    metadata.planning_spacing != null ? `planning_spacing: ${metadata.planning_spacing}px` : null,
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
      (eid, i) =>
        `<span class="chip"><span class="idx">${i + 1}</span>${escapeHtml(eid)}</span>`
    )
    .join("");
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
    const edge = seg.edge_id || `${seg.from_edge_id || "?"}→${seg.to_edge_id || "?"}`;
    const hoverLine = `${seg.segment_id}<br>${seg.type} · ${Math.round(seg.length)}px<br>${edge}`;

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

  if (state.showInspect && inspectX.length) {
    traces.push({
      x: inspectX,
      y: inspectY,
      mode: "lines",
      name: "Inspect",
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
      name: "Connect",
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
    traces.push({
      x: visiblePts.map((p) => p.x),
      y: visiblePts.map((p) => p.y),
      mode: "markers",
      name: "Inspection Points",
      marker: {
        size: isAll ? 4 : 8,
        color: MAP_STYLES.pointColor,
        opacity: isAll ? 0.45 : 0.88,
        line: { width: isAll ? 0.5 : 1.5, color: MAP_STYLES.pointLine },
      },
      text: visiblePts.map((p) => p.point_id || p.edge_id || "point"),
      hovertemplate: "%{text}<extra></extra>",
      showlegend: true,
      legendgroup: "points",
    });
  }

  const markers = result.markers || {};
  if (markers.start) {
    traces.push({
      x: [markers.start.x],
      y: [markers.start.y],
      mode: "markers+text",
      name: "Start",
      marker: {
        size: 16,
        color: "#22c55e",
        symbol: "star",
        line: { width: 2, color: "#14532d" },
      },
      text: ["START"],
      textfont: { size: 10, color: "#bbf7d0" },
      textposition: "top center",
      hovertemplate: "Start (%{x:.0f}, %{y:.0f})<extra></extra>",
      showlegend: true,
      legendgroup: "start",
    });
  }
  if (markers.end) {
    traces.push({
      x: [markers.end.x],
      y: [markers.end.y],
      mode: "markers+text",
      name: "End",
      marker: {
        size: 16,
        color: "#ef4444",
        symbol: "diamond",
        line: { width: 2, color: "#7f1d1d" },
      },
      text: ["END"],
      textfont: { size: 10, color: "#fecaca" },
      textposition: "bottom center",
      hovertemplate: "End (%{x:.0f}, %{y:.0f})<extra></extra>",
      showlegend: true,
      legendgroup: "end",
    });
  }

  return traces;
}

function buildLegendLayout() {
  return {
    orientation: "h",
    y: -0.12,
    x: 0.5,
    xanchor: "center",
    yanchor: "top",
    bgcolor: "rgba(26, 35, 50, 0.75)",
    bordercolor: "#2d3a4f",
    font: { size: 10 },
  };
}

function getFullscreenPlotSize() {
  const panel = document.querySelector(".modal-panel");
  const plotEl = $("mapPlotFs");
  if (panel && plotEl) {
    const pr = panel.getBoundingClientRect();
    const header = panel.querySelector(".modal-header");
    const headerH = header ? header.offsetHeight + 12 : 40;
    return {
      width: Math.max(320, Math.floor(pr.width - 24)),
      height: Math.max(320, Math.floor(pr.height - headerH - 16)),
    };
  }
  return {
    width: Math.floor(window.innerWidth * 0.96),
    height: Math.floor(window.innerHeight * 0.9) - 64,
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
      ? { l: 56, r: 20, t: 40, b: 72 }
      : { l: 52, r: 12, t: 40, b: 72 },
    legend: buildLegendLayout(),
    hovermode: "closest",
    images: buildBackgroundImages(result),
    uirevision: "mission-map-v3",
  };

  if (isImage) {
    const { width, height } = getImageSize(result);
    layout.xaxis = {
      title: "X (px)",
      range: [0, width],
      autorange: false,
      fixedrange: true,
      constrain: "domain",
      showgrid: true,
      gridcolor: "rgba(255,255,255,0.12)",
      zeroline: false,
    };
    layout.yaxis = {
      title: "Y (px)",
      range: [height, 0],
      autorange: false,
      fixedrange: true,
      scaleanchor: "x",
      scaleratio: 1,
      showgrid: true,
      gridcolor: "rgba(255,255,255,0.12)",
      zeroline: false,
    };
  } else {
    const b = result.bounds || { x_range: [0, 1000], y_range: [1000, 0] };
    layout.xaxis = {
      title: "x (px)",
      range: b.x_range,
      autorange: false,
      gridcolor: "#2d3a4f",
      zeroline: false,
    };
    layout.yaxis = {
      title: "y (px)",
      range: b.y_range,
      scaleanchor: "x",
      scaleratio: 1,
      gridcolor: "#2d3a4f",
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
  modal.classList.remove("hidden");
  modal.setAttribute("aria-hidden", "false");
  requestAnimationFrame(() => {
    renderMission(state.lastResult, "mapPlotFs", { fullscreen: true });
  });
}

function closeFullscreen() {
  const modal = $("mapFullscreenModal");
  modal.classList.add("hidden");
  modal.setAttribute("aria-hidden", "true");
  try {
    const fsEl = $("mapPlotFs");
    if (fsEl) fsEl.__dashMapClick = false;
    Plotly.purge("mapPlotFs");
    state.plotReady.mapPlotFs = false;
  } catch (_) {}
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
      }),
    });
    const data = await res.json();
    if (!res.ok || !data.ok) {
      throw new Error(data.message || data.detail || res.statusText);
    }

    const dashboard = normalizeMissionResult(data.dashboard || data);
    state.lastResult = dashboard;
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

async function runPlanning() {
  const pipeline = getPipeline();
  const body = {
    pipeline,
    input_file: "",
    planner: "legacy",
    spacing: 50,
    map_mode: "image_overlay",
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

  $("runBtn").disabled = true;
  setStatus(
    pipeline === "image" ? "正在生成/加载图像主线…" : "Unified 规划中…",
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
        : `完成 · ${body.planner}`;
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
    setStatus("就绪 · 运行 python app.py 后点击加载即可", "");
  } catch (e) {
    setStatus(`初始化失败: ${e.message}`, "err");
  }

  $("pipelineSelect").addEventListener("change", () => {
    applyPipelineUi();
    state.lastResult = null;
    state.plotReady.mapPlot = false;
    if (typeof window.resetInspectionPlayback === "function") window.resetInspectionPlayback();
    setStatus("就绪", "");
  });

  $("runBtn").addEventListener("click", runPlanning);
  $("forceRegenBtn").addEventListener("click", forceRegenerateImage);
  $("fullscreenBtn").addEventListener("click", openFullscreen);
  $("closeFullscreenBtn").addEventListener("click", closeFullscreen);
  $("modalBackdrop")?.addEventListener("click", closeFullscreen);
  $("runReplanBtn")?.addEventListener("click", runReplan);

  ["toggleBaseMap", "toggleInspect", "toggleConnect"].forEach((id) => {
    $(id)?.addEventListener("change", () => refreshMapView());
  });
  $("pointModeSelect")?.addEventListener("change", () => refreshMapView());

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
    }
  });
});
