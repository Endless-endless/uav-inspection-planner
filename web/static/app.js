/**
 * UAV Mission Dashboard — dual pipeline + auto image generation
 */

const $ = (id) => document.getElementById(id);

const MAP_STYLES = {
  inspectColor: "#1f77b4",
  connectColor: "#ff7f0e",
  /** 未巡检线路：仅当 payload 显式提供 unvisited_geometry_2d / explicit_unvisited_lines 时使用；默认无几何 */
  pendingTopoLine: {
    color: "red",
    width: 3,
    dash: "solid",
    opacity: 1,
  },
  pointColor: "#2563eb",
  pointLine: "#e2e8f0",
};

const IMAGE_DEFAULT_SIZE = { width: 916, height: 960 };

/** Plotly 底图等：统一走后端 /api/map/background，避免直连 /data 静态路径不一致 */
function toStaticDataUrl(relPath) {
  const p = String(relPath || "").replace(/\\/g, "/");
  return `/api/map/background?path=${encodeURIComponent(p)}`;
}
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
  pointMode: "key",
  plotReady: { mapPlot: false, mapPlotFs: false },
  imageDatasetRegistry: null,
  imageDatasetProfile: null,
  /** 任务控制左上角状态：idle | planning | mission_ready | playing | paused | replanning | replan_failed */
  taskControlPhase: "idle",
};
window.state = state;

const IMAGE_DATASET_FALLBACK = {
  id: "chengdu_real",
  dataset: "data/chengdu_real_point.png",
  clean_map_image: "data/chengdu_real_point.png",
  line_image: "data/chengdu_real_line.png",
  point_image: "data/chengdu_real_point.png",
  dataset_type: "real_satellite",
};

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

function logBackgroundImageLoadError(url, reason) {
  console.error("[map background] image load failed:", url, reason || "");
}

/** 预加载底图；失败时在 console.error 输出 URL */
function preloadBackgroundImage(url) {
  if (!url) return Promise.resolve(false);
  return new Promise((resolve) => {
    const img = new Image();
    img.onload = () => resolve(true);
    img.onerror = () => {
      logBackgroundImageLoadError(url, "Image onerror");
      resolve(false);
    };
    img.src = url;
  });
}

async function loadMapConfig(imagePath) {
  try {
    const path = imagePath || "data/test.png";
    const res = await fetch(`/api/map/config?path=${encodeURIComponent(path)}`);
    if (!res.ok) throw new Error(res.statusText);
    state.mapConfig = await res.json();
    const bgUrl = toStaticDataUrl(path);
    await preloadBackgroundImage(bgUrl);
  } catch (e) {
    state.mapConfig = { available: false };
    logBackgroundImageLoadError(toStaticDataUrl(imagePath || "data/test.png"), e?.message || e);
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
  state.datasets = [];
}

async function loadImageDatasetProfiles() {
  try {
    const res = await fetch("/api/image-datasets");
    if (!res.ok) throw new Error(res.statusText);
    const data = await res.json();
    state.imageDatasetRegistry = data;
    const profiles = data.datasets || [];
    const defaultId = data.default_id || IMAGE_DATASET_FALLBACK.id;
    state.imageDatasetProfile =
      profiles.find((p) => p.id === defaultId) || profiles[0] || IMAGE_DATASET_FALLBACK;
  } catch (e) {
    console.warn("image-datasets load failed, using fallback", e);
    state.imageDatasetRegistry = { default_id: IMAGE_DATASET_FALLBACK.id, datasets: [IMAGE_DATASET_FALLBACK] };
    state.imageDatasetProfile = IMAGE_DATASET_FALLBACK;
  }
}

function getActiveImageProfile() {
  return state.imageDatasetProfile || IMAGE_DATASET_FALLBACK;
}

function isRealSatelliteDataset(profile = getActiveImageProfile()) {
  return profile?.dataset_type === "real_satellite";
}

function usesImagePixelCoords(result) {
  const m = result?.metadata || {};
  return (
    m.pixel_coordinate_mode === true ||
    m.coordinate_mode === "image_pixel_fixed" ||
    m.dataset_type === "real_satellite" ||
    isRealSatelliteDataset()
  );
}

/** 图像管线 Plotly 底图路径（真实地图 = 带点标注图，与 CV 输入一致） */
function getImagePipelineMapPath() {
  const profile = getActiveImageProfile();
  if (isRealSatelliteDataset(profile)) {
    return profile.point_image || profile.dataset || "data/chengdu_real_point.png";
  }
  const clean = profile.clean_map_image;
  if (clean && clean !== "auto") return clean;
  return profile.line_image || profile.dataset || "data/test.png";
}

function getDisplayMapPath() {
  return getImagePipelineMapPath();
}

function getDetectionImagePath(source) {
  const profile = getActiveImageProfile();
  const norm = window.normalizeInspectionPointSource
    ? normalizeInspectionPointSource(source)
    : source;
  if (norm === "image") {
    return profile.point_image || profile.dataset || "data/test_point.png";
  }
  return profile.line_image || profile.dataset || "data/test.png";
}

function getImagePathForSource(source) {
  return getDetectionImagePath(source);
}

/** 显示底图路径：与识别输入同源（真实地图用 chengdu_real_point.png） */
function resolveCleanMapImagePath(result) {
  const meta = result?.metadata || {};
  let path =
    meta.display_map_image ||
    meta.clean_map_image ||
    meta.point_image ||
    meta.map_image ||
    result?.map_background?.path ||
    null;
  if (path && /chengdu_real_satellite/i.test(String(path))) {
    path = meta.point_image || getImagePipelineMapPath();
  }
  if (!path && (meta.dataset_type === "real_satellite" || /chengdu_real/i.test(String(meta.dataset_type || meta.dataset || "")))) {
    path = getImagePipelineMapPath();
  }
  if (!path && isImagePipeline()) {
    path = getImagePipelineMapPath();
  }
  return path;
}

function getPipelineBackgroundSpec(result) {
  const relPath = resolveCleanMapImagePath(result) || getImagePipelineMapPath();
  const cfg =
    state.mapConfig?.path === relPath || String(state.mapConfig?.path || "").endsWith(relPath)
      ? state.mapConfig
      : null;
  const bg = result?.map_background || {};
  const bounds = result?.bounds || {};
  const width = bounds.width || bg.width || cfg?.width || IMAGE_DEFAULT_SIZE.width;
  const height = bounds.height || bg.height || cfg?.height || IMAGE_DEFAULT_SIZE.height;
  const source = toStaticDataUrl(relPath);
  if (isImagePipeline() && relPath && !getPipelineBackgroundSpec._preloadDone?.[source]) {
    getPipelineBackgroundSpec._preloadDone = getPipelineBackgroundSpec._preloadDone || {};
    getPipelineBackgroundSpec._preloadDone[source] = true;
    preloadBackgroundImage(source);
  }
  return { relPath, width, height, source };
}

/** 为 image 管线合成 map_background（尺寸与 URL 元数据；主图底图由 HTML img 叠层，不用 layout.images） */
function buildMapBackgroundForResult(result) {
  const pipeline = result?.metadata?.pipeline || (isImagePipeline() ? "image" : "unified");
  if (pipeline !== "image") return result?.map_background || null;

  const displayPath = resolveCleanMapImagePath(result);
  if (!displayPath) return result?.map_background || state.mapConfig || null;

  const existing = result?.map_background;
  const url = `/api/map/background?path=${encodeURIComponent(displayPath)}`;
  const cfg =
    state.mapConfig?.path === displayPath || state.mapConfig?.path?.endsWith(displayPath)
      ? state.mapConfig
      : null;
  const width =
    cfg?.width || existing?.width || result?.bounds?.width || IMAGE_DEFAULT_SIZE.width;
  const height =
    cfg?.height || existing?.height || result?.bounds?.height || IMAGE_DEFAULT_SIZE.height;

  if (
    existing?.layout_image?.source &&
    (existing.path === displayPath || String(existing.url || "").includes(encodeURIComponent(displayPath)))
  ) {
    return { ...existing, available: true, path: displayPath, url: existing.url || url, width, height };
  }

  const layoutImage = {
    source: toStaticDataUrl(displayPath),
    xref: "x",
    yref: "y",
    x: 0,
    y: 0,
    xanchor: "left",
    yanchor: "bottom",
    sizex: width,
    sizey: height,
    sizing: "stretch",
    layer: "below",
    opacity: 1,
  };

  return {
    available: true,
    path: displayPath,
    url,
    width,
    height,
    layout_image: layoutImage,
    axis: { x_range: [0, width], y_range: [height, 0] },
    coordinate_system: "image_upper_left_y_down",
  };
}

function getInspectionPointSource() {
  const ui = $("inspectionPointSourceSelect")?.value || "spacing";
  return window.normalizeInspectionPointSource
    ? normalizeInspectionPointSource(ui)
    : ui;
}

function updateSpacingControlsVisibility() {
  const src = getInspectionPointSource();
  const imageMode = isImagePipeline() && src === "image";
  const storeImage = window.MissionStore?.isImageSource?.() === true;
  const hideSpacing = imageMode || storeImage;

  const spacingField = $("spacingFieldWrap") || $("spacingInput")?.closest(".field");
  const planningField = $("planningSpacingFieldWrap") || $("planningSpacingInput")?.closest(".field");
  if (spacingField) spacingField.classList.toggle("hidden", hideSpacing);
  if (planningField) planningField.classList.toggle("hidden", hideSpacing);
}

function stripManualInspectionSourceOption() {
  const sel = $("inspectionPointSourceSelect");
  if (!sel) return;
  [...sel.options].forEach((opt) => {
    if (opt.value === "manual") opt.remove();
  });
  if (sel.value === "manual") sel.value = "image";
}

function applyPipelineUi() {
  stripManualInspectionSourceOption();
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
    const profile = getActiveImageProfile();
    const detectPath = getDetectionImagePath(pointSource);
    const displayPath = getDisplayMapPath();
    if (ds) {
      const label =
        profile.dataset_type === "real_satellite"
          ? `真实地图: ${profile.point_image || detectPath}`
          : detectPath;
      ds.innerHTML = `<option value="${detectPath}">${label}</option>`;
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
  updateSpacingControlsVisibility();
}


function readUiToState() {
  state.showBackground = $("toggleBaseMap")?.checked !== false;
  state.showInspect = $("toggleInspect")?.checked !== false;
  state.showConnect = $("toggleConnect")?.checked !== false;
  if ($("togglePointsLayer")) {
    state.pointMode = $("togglePointsLayer").checked ? "key" : "hidden";
  } else {
    state.pointMode = $("pointModeSelect")?.value || "key";
  }
}

function syncStateToUi() {
  if ($("toggleBaseMap")) $("toggleBaseMap").checked = state.showBackground;
  if ($("toggleInspect")) $("toggleInspect").checked = state.showInspect;
  if ($("toggleConnect")) $("toggleConnect").checked = state.showConnect;
  if ($("togglePointsLayer")) $("togglePointsLayer").checked = state.pointMode !== "hidden";
  if ($("pointModeSelect")) $("pointModeSelect").value = state.pointMode;
}

function applyLayerDefaultsForPipeline() {
  if (isImagePipeline()) {
    state.showBackground = getMapMode() === "image_overlay";
    state.pointMode = "key";
  } else {
    state.showBackground = false;
    state.pointMode = "all";
  }
  syncStateToUi();
}

function syncHtmlBasemapImages(mission) {
  const norm = normalizeMissionResult(mission || buildIdleImageMapPayload());
  const pairs = [
    { stack: "mapPlotStack", img: "mapBasemapImg" },
    { stack: "mapPlotStackFs", img: "mapBasemapImgFs" },
  ];
  if (!isImagePipeline()) {
    pairs.forEach(({ stack: sid, img: iid }) => {
      const s = $(sid);
      const im = $(iid);
      if (s) s.style.aspectRatio = "";
      im?.classList.add("hidden");
    });
    return;
  }
  const rel = resolveCleanMapImagePath(norm) || getImagePipelineMapPath();
  const src = toStaticDataUrl(rel);
  const show = state.showBackground !== false && Boolean(rel);
  const spec = getPipelineBackgroundSpec(norm);
  const w = spec.width || 1415;
  const h = spec.height || 1258;
  pairs.forEach(({ stack: sid, img: iid }) => {
    const stack = $(sid);
    const img = $(iid);
    if (stack && usesImagePixelCoords(norm)) {
      stack.style.aspectRatio = `${w} / ${h}`;
    } else if (stack) {
      stack.style.aspectRatio = "";
    }
    if (!img) return;
    if (show) {
      img.classList.remove("hidden");
      if (img.dataset.loadedSrc !== src) {
        img.src = src;
        img.dataset.loadedSrc = src;
      }
    } else {
      img.classList.add("hidden");
    }
  });
}

function getImageSize(result) {
  const spec = getPipelineBackgroundSpec(result || (typeof getCurrentMission === "function" ? getCurrentMission() : null));
  return { width: spec.width, height: spec.height };
}

/** 底图改为 HTML img 叠层，Plotly 不再使用 layout.images */
function getFixedSatelliteLayoutImages(_result) {
  return [];
}

/** 透明 Plotly + 空 images；底图由 #mapBasemapImg / #mapBasemapImgFs 提供 */
function applyFixedSatelliteToLayout(layout, result) {
  const { width, height } = getPipelineBackgroundSpec(
    result || (typeof getCurrentMission === "function" ? getCurrentMission() : null)
  );
  const base = layout && typeof layout === "object" ? layout : {};
  const transparent =
    isImagePipeline() && usesImagePixelCoords(result || (typeof getCurrentMission === "function" ? getCurrentMission() : null));
  const hiddenPixelAxes = transparent
    ? {
        showgrid: false,
        zeroline: false,
        showline: false,
        showticklabels: false,
        ticks: "",
      }
    : null;
  return {
    ...base,
    paper_bgcolor: transparent ? "rgba(0,0,0,0)" : base.paper_bgcolor || "#1a2332",
    plot_bgcolor: transparent ? "rgba(0,0,0,0)" : base.plot_bgcolor || "rgba(255,255,255,0.06)",
    images: [],
    xaxis: {
      ...(base.xaxis || {}),
      range: [0, width],
      autorange: false,
      ...(hiddenPixelAxes || {}),
    },
    yaxis: {
      ...(base.yaxis || {}),
      range: [height, 0],
      autorange: false,
      scaleanchor: "x",
      scaleratio: 1,
      constrain: "domain",
      ...(hiddenPixelAxes || {}),
    },
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

  if (pipeline === "image" || isImagePipeline()) {
    if (!out.metadata.pipeline) out.metadata.pipeline = "image";
    const displayRel = resolveCleanMapImagePath(out);
    if (displayRel) {
      out.metadata.display_map_image = displayRel;
      out.metadata.clean_map_image = displayRel;
    }
    out.map_background = buildMapBackgroundForResult(out);
    const spec = getPipelineBackgroundSpec(out);
    const width = spec.width;
    const height = spec.height;
    if (usesImagePixelCoords(out)) {
      out.metadata.coordinate_mode = "image_pixel_fixed";
      out.metadata.pixel_coordinate_mode = true;
      out.metadata.image_width = width;
      out.metadata.image_height = height;
    }
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

function buildIdleImageMapPayload() {
  return {
    segments: [],
    inspection_points: [],
    visit_order: [],
    metadata: { pipeline: "image" },
  };
}

function renderInitialMapPreview(targetId = "mapPlot") {
  if (!isImagePipeline()) return;
  const plot = buildPlotFromMission(buildIdleImageMapPayload());
  const el = document.getElementById(targetId);
  if (!el) return;
  const layout = applyFixedSatelliteToLayout(plot.layout, buildIdleImageMapPayload());
  const tracesFresh = Array.isArray(plot.traces) ? plot.traces.slice() : [];
  const draw = () => {
    bindMapClick(targetId);
    syncHtmlBasemapImages(getCurrentMission() || buildIdleImageMapPayload());
  };
  if (state.plotReady[targetId] && el.data) {
    Plotly.newPlot(targetId, tracesFresh, layout, plot.config).then(draw);
  } else {
    Plotly.newPlot(targetId, tracesFresh, layout, plot.config).then(() => {
      state.plotReady[targetId] = true;
      draw();
    });
  }
}

/** 清空地图 Plotly 与易失图层，避免旧红线 / 旧 trace 残留（生成、重规划、重置前调用） */
function purgeDashboardMapPlots() {
  if (!window.Plotly) return;
  ["mapPlot", "mapPlotFs"].forEach((plotId) => {
    const el = document.getElementById(plotId);
    if (!el) return;
    try {
      Plotly.purge(plotId);
    } catch (_) {}
    state.plotReady[plotId] = false;
  });
}
window.purgeDashboardMapPlots = purgeDashboardMapPlots;

/** 服务端重规划写入前：清掉可能残留的拓扑/实验图层与 Plotly trace */
window.prepareMissionPlotForServerReplan = function prepareMissionPlotForServerReplan() {
  purgeDashboardMapPlots();
};

function purgeSingleMissionPlot(plotId) {
  if (!window.Plotly) return;
  const el = document.getElementById(plotId);
  if (!el) return;
  try {
    Plotly.purge(plotId);
  } catch (_) {}
  state.plotReady[plotId] = false;
}

function refreshMissionMap(targetId = "mapPlot", options = {}) {
  const mission = getCurrentMission();
  if (!mission) {
    if (isImagePipeline()) renderInitialMapPreview(targetId);
    return;
  }
  renderMission(mission, targetId, options);
}

function refreshMapView() {
  const mission = getCurrentMission();
  if (!mission) {
    if (isImagePipeline()) renderInitialMapPreview("mapPlot");
    return;
  }
  refreshMissionMap("mapPlot");
  const fullscreenOpen =
    $("mapFullscreenModal") && !$("mapFullscreenModal").classList.contains("hidden");
  if (fullscreenOpen) {
    refreshMissionMap("mapPlotFs", { fullscreen: true });
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
    $("mapPlotStack")?.classList.add("overlay-mode");
    $("mapPlotStackFs")?.classList.add("overlay-mode");
  } else {
    hint.textContent =
      getMapMode() === "topology_only"
        ? "深色拓扑视图"
        : "实验视图";
    $("mapPlotStack")?.classList.remove("overlay-mode");
    $("mapPlotStackFs")?.classList.remove("overlay-mode");
  }
}

function updateDownloadLinks(outputFiles, pipeline) {
  const base = "/api/output/";

  if (pipeline === "image") {
    const snap = outputFiles?.mission_snapshot || "latest_image_mission_snapshot.json";
    $("headerDownload").href = base + snap;
    $("dlMission").href = base + snap;
    $("dlAnalysis").classList.add("hidden");
    $("dlCompare").classList.add("hidden");
    $("dlLegacyHtml")?.classList.add("hidden");
    return;
  }

  $("dlLegacyHtml")?.classList.add("hidden");
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
  const lines = [metadata?.pipeline ? `管线: ${metadata.pipeline}` : null].filter(Boolean);
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
  planning: "任务生成中",
  mission_ready: "任务已就绪",
  playing: "巡检中",
  paused: "已暂停",
  replanning: "重规划中",
  replan_failed: "重规划失败",
  adaptive_warning: "风险预警",
  finished: "已完成",
};

function resolveTaskControlText() {
  const visual = typeof window.getPlaybackVisualState === "function"
    ? window.getPlaybackVisualState()
    : null;
  if (visual?.status === "playing") return "巡检中";
  if (visual?.status === "paused") return "已暂停";
  const phase = state.taskControlPhase || "idle";
  return PHASE_LABELS[phase] || "待命";
}

/** 更新左上角「任务控制」状态框（#playbackConsoleState） */
function setTaskControlStatus(text) {
  const t = text || resolveTaskControlText();
  if ($("playbackConsoleState")) $("playbackConsoleState").textContent = t;
  if ($("missionPhaseBadge")) $("missionPhaseBadge").textContent = t;
}

function setTaskControlPhase(phase) {
  state.taskControlPhase = phase || "idle";
  setTaskControlStatus();
}

window.setTaskControlPhase = setTaskControlPhase;
window.setTaskControlStatus = setTaskControlStatus;

function updateMissionStatusHud() {
  const phase = state.taskControlPhase || "idle";
  const visual = typeof window.getPlaybackVisualState === "function"
    ? window.getPlaybackVisualState()
    : null;
  const playbackPhase = visual?.status === "playing"
    ? "playing"
    : visual?.status === "paused"
      ? "paused"
      : phase;

  let label = "NORMAL";
  let mod = "normal";
  if (playbackPhase === "replanning" || phase === "replanning") {
    label = "REPLANNING";
    mod = "replan";
  } else if (playbackPhase === "playing" || phase === "playing") {
    label = "INSPECTING";
    mod = "normal";
  } else if (playbackPhase === "paused" || phase === "paused") {
    label = "PAUSED";
    mod = "warn";
  } else if (phase === "finished") {
    label = "FINISHED";
    mod = "normal";
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
  const phaseText = resolveTaskControlText();
  if ($("sysPhase")) $("sysPhase").textContent = phaseText;
  setTaskControlStatus(phaseText);

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
  updateMissionStatusHud();
}

window.renderSystemStatus = renderSystemStatus;

function renderAdvancedStats(statistics) {
  const host = $("advancedStatCards");
  if (!host) return;
  const advancedOpen = document.getElementById("advancedExperimentPanel")?.open;
  if (!advancedOpen) {
    host.classList.add("hidden");
    host.innerHTML = "";
    return;
  }
  const s = statistics || {};
  const items = [
    ["connect_ratio", "连接占比", ((s.connect_ratio || 0) * 100).toFixed(1), "%"],
    ["total_cost", "实际总代价", s.total_cost ?? s.total_length ?? 0, ""],
    ["risky_distance", "高风险穿越", s.risky_distance ?? 0, " px"],
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

function renderStats(statistics) {
  const s = statistics || {};
  const mission = typeof getCurrentMission === "function" ? getCurrentMission() : null;
  const dashLen = mission != null ? (mission.inspection_points ?? []).length : null;
  const inspectionPointKpi =
    mission != null
      ? (mission.inspection_points ?? []).length
      : s.dashboard_inspection_points ?? s.inspection_points_count ?? s.num_inspection_points;
  const phase = window.AppPhaseManager?.getPhase?.() || window.AppPhase?.IDLE || "idle";
  const phaseText = (PHASE_LABELS && PHASE_LABELS[phase]) || phase;

  const primary = $("statCardsPrimary");
  const secondary = $("statCardsSecondary");
  const itemsPrimary = [
    ["total_length", "总长度", s.total_length, " px"],
    ["num_inspection_points", "巡检点数", inspectionPointKpi, ""],
    ["phase", "任务状态", phaseText, ""],
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
    ["total_cost", "实际总代价", s.total_cost ?? s.total_length ?? "—", ""],
    ["num_segments", "区段数", s.num_segments ?? "—", ""],
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
  const meta = mission?.metadata || {};
  const idst = meta.image_detection_stats || {};
  console.log(
    "[point-flow] cv_after_merge=",
    idst.after_merge ?? idst.merged_points ?? "—",
    "dashboard_inspection_points=",
    dashLen ?? "—",
    "ui_kpi_inspection_points=",
    inspectionPointKpi
  );
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






function collectInspectEdgeIdsFromSegments(mission) {
  const segs = mission?.segments || [];
  const out = [];
  segs.forEach((seg) => {
    if (!seg || typeof seg !== "object") return;
    const typ = String(seg.type || "").toLowerCase();
    const role = String(seg.role || "").toLowerCase();
    const skipTypes = new Set([
      "debug",
      "overlay",
      "redline",
      "topo",
      "raw",
      "skeleton",
      "hsv",
      "legacy",
      "image_trace",
      "mask",
    ]);
    if (skipTypes.has(typ)) return;
    if (/debug|overlay|redline|topo|raw|skeleton|hsv|legacy|mask|dummy|placeholder/.test(role)) return;
    if (typ !== "inspect") return;
    const eid = seg.edge_id != null ? String(seg.edge_id).replace(/[+-]$/, "") : "";
    if (eid) out.push(eid);
  });
  return out;
}

/** Mission / 点上的 edge_id 与 segment 对齐（去掉 +/- 方向后缀） */
function normalizeMissionEdgeId(raw) {
  if (raw == null || raw === "") return "";
  return String(raw).replace(/[+-]$/, "").trim();
}

/** 从 L_001 / L_001_edge_0 等串中提取 L_ 数字前缀，供与 segment / chain 上的 line 字段对齐 */
function extractTopoLineKeyFromEdgeString(norm) {
  if (!norm) return null;
  const m = String(norm).match(/^(L_\d+)/);
  return m ? m[1] : null;
}

/** 巡检点到折线的最短距离（像素），与后端 _point_segment_distance 一致 */
function missionPointToPolylineDistancePx(x, y, geometry) {
  if (!geometry || geometry.length < 2) return Infinity;
  let best = Infinity;
  for (let i = 0; i < geometry.length - 1; i += 1) {
    const ax = Number(geometry[i][0]);
    const ay = Number(geometry[i][1]);
    const bx = Number(geometry[i + 1][0]);
    const by = Number(geometry[i + 1][1]);
    const dx = bx - ax;
    const dy = by - ay;
    const denom = dx * dx + dy * dy;
    let d;
    if (denom <= 1e-9) {
      d = Math.hypot(x - ax, y - ay);
    } else {
      const t = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / denom));
      const px = ax + t * dx;
      const py = ay + t * dy;
      d = Math.hypot(x - px, y - py);
    }
    if (d < best) best = d;
  }
  return best;
}

function collectSegmentInspectMatchTokens(seg) {
  const tokens = new Set();
  const add = (v) => {
    if (v == null || v === "") return;
    if (Array.isArray(v)) {
      v.forEach((x) => add(x));
      return;
    }
    const n = normalizeMissionEdgeId(String(v));
    if (!n) return;
    tokens.add(n);
    const lk = extractTopoLineKeyFromEdgeString(n);
    if (lk) tokens.add(lk);
  };
  if (!seg || typeof seg !== "object") return tokens;
  add(seg.line_id);
  add(seg.original_line_id);
  add(seg.source_line_id);
  add(seg.chain_id);
  add(seg.physical_line_id);
  add(seg.edge_id);
  add(seg.member_chain_ids);
  return tokens;
}

/** 仅 edge_id / line_id / original_line_id：用于「字段命中」判定，命中则不走几何兜底 */
function collectPointTopoLineKeysPrimary(point) {
  const keys = new Set();
  [point?.edge_id, point?.line_id, point?.original_line_id].forEach((f) => {
    if (f == null || f === "") return;
    const n = normalizeMissionEdgeId(String(f));
    if (!n) return;
    const lk = extractTopoLineKeyFromEdgeString(n);
    if (lk) keys.add(lk);
    keys.add(n);
  });
  return keys;
}

function getPhysicalLineChainsArray(mission) {
  return (
    mission?.physical_line_chains ||
    mission?.metadata?.physical_line_chains ||
    mission?.metadata?.mission_metadata?.physical_line_chains ||
    []
  );
}

/** 从 L_001 得到 PL_001（数字与 PL 后缀对齐；用于 segment 未带 line_id 时的兜底） */
function plEdgeIdFromTopoLineKey(lineKey) {
  const m = lineKey && String(lineKey).match(/^L_(\d+)$/);
  if (!m) return "";
  const n = parseInt(m[1], 10);
  if (!Number.isFinite(n) || n < 0) return "";
  return `PL_${String(n).padStart(3, "0")}`;
}

/** 按 edge_id → line_id → original_line_id 顺序收集 L_xxx，用于 L→PL 推断 */
function collectOrderedTopoLineKeysFromPoint(point) {
  const keys = [];
  const seen = new Set();
  const pushLk = (f) => {
    const lk = extractTopoLineKeyFromEdgeString(normalizeMissionEdgeId(f));
    if (lk && !seen.has(lk)) {
      seen.add(lk);
      keys.push(lk);
    }
  };
  pushLk(point?.edge_id);
  pushLk(point?.line_id);
  pushLk(point?.original_line_id);
  return keys;
}

function getMissionInspectPlEdgeIdSet(mission) {
  return new Set(
    collectInspectEdgeIdsFromSegments(mission).map((e) => normalizeMissionEdgeId(e))
  );
}

/** 点到指定 PL inspect 段折线的最短距离（px），供日志与非几何路径展示 */
function missionDistanceToInspectPlEdge(mission, plEdgeId, px, py) {
  const pl = normalizeMissionEdgeId(plEdgeId);
  if (!pl || !pl.startsWith("PL_")) return null;
  if (!Number.isFinite(px) || !Number.isFinite(py)) return null;
  let best = Infinity;
  (mission?.segments || []).forEach((seg) => {
    if (!seg || String(seg.type || "").toLowerCase() !== "inspect") return;
    if (normalizeMissionEdgeId(seg.edge_id) !== pl) return;
    const d = missionPointToPolylineDistancePx(px, py, seg.geometry_2d || []);
    if (d < best) best = d;
  });
  if (best === Infinity) return null;
  return Math.round(best * 100) / 100;
}

/**
 * 将巡检点上的 L_ 系 edge / line 标识解析为 Mission inspect 段上的 PL_ edge_id，
 * 供与 collectInspectEdgeIdsFromSegments 得到的集合对齐（纯前端展示归属）。
 * @returns {{ resolved_edge_id: string, nearest_dist: number|null, fallback_reason: string }}
 */
function resolvePointInspectEdgeId(point, mission) {
  const pack = (resolved_edge_id, nearest_dist, fallback_reason) => ({
    resolved_edge_id: resolved_edge_id || "",
    nearest_dist: nearest_dist == null || !Number.isFinite(nearest_dist) ? null : nearest_dist,
    fallback_reason: fallback_reason || "",
  });

  const raw_edge_id = point?.edge_id;
  const raw_line_id = point?.line_id != null ? point.line_id : point?.metadata?.line_id;
  const ne = normalizeMissionEdgeId(raw_edge_id);
  const px = Number(point?.x);
  const py = Number(point?.y);

  if (ne && ne.startsWith("PL_")) {
    return pack(ne, missionDistanceToInspectPlEdge(mission, ne, px, py), "direct_pl_on_point");
  }

  const primaryKeys = collectPointTopoLineKeysPrimary(point);
  const segs = mission?.segments || [];

  for (let i = 0; i < segs.length; i += 1) {
    const seg = segs[i];
    if (!seg || String(seg.type || "").toLowerCase() !== "inspect") continue;
    const segPl = normalizeMissionEdgeId(seg.edge_id);
    if (!segPl || !segPl.startsWith("PL_")) continue;
    const segTok = collectSegmentInspectMatchTokens(seg);
    for (const pk of primaryKeys) {
      if (pk && segTok.has(pk)) {
        return pack(
          segPl,
          missionDistanceToInspectPlEdge(mission, segPl, px, py),
          "primary_field_inspect_segment"
        );
      }
    }
    if (ne && segTok.has(ne)) {
      return pack(
        segPl,
        missionDistanceToInspectPlEdge(mission, segPl, px, py),
        "primary_field_inspect_segment"
      );
    }
  }

  const chains = getPhysicalLineChainsArray(mission);
  if (Array.isArray(chains)) {
    for (let c = 0; c < chains.length; c += 1) {
      const ch = chains[c];
      if (!ch || typeof ch !== "object") continue;
      const pid = normalizeMissionEdgeId(ch.id != null ? ch.id : ch.edge_id);
      if (!pid || !pid.startsWith("PL_")) continue;
      const nested = ch.meta && typeof ch.meta === "object" ? ch.meta : {};
      const parts = [
        ...(Array.isArray(ch.line_ids) ? ch.line_ids : []),
        ...(Array.isArray(ch.chain_ids) ? ch.chain_ids : []),
        ...(Array.isArray(nested.member_chain_ids) ? nested.member_chain_ids : []),
        ...(Array.isArray(nested.member_line_ids) ? nested.member_line_ids : []),
      ];
      const chTok = new Set();
      parts.forEach((v) => {
        const n = normalizeMissionEdgeId(v);
        if (n) chTok.add(n);
        const lk = extractTopoLineKeyFromEdgeString(n);
        if (lk) chTok.add(lk);
      });
      for (const pk of primaryKeys) {
        if (pk && chTok.has(pk)) {
          return pack(
            pid,
            missionDistanceToInspectPlEdge(mission, pid, px, py),
            "primary_field_physical_chain"
          );
        }
      }
      if (ne && chTok.has(ne)) {
        return pack(
          pid,
          missionDistanceToInspectPlEdge(mission, pid, px, py),
          "primary_field_physical_chain"
        );
      }
    }
  }

  const inspectPlSet = getMissionInspectPlEdgeIdSet(mission);
  const orderedLineKeys = collectOrderedTopoLineKeysFromPoint(point);
  if (!orderedLineKeys.length) {
    return pack("", null, "unresolved_no_l_key_in_point_fields");
  }
  for (let k = 0; k < orderedLineKeys.length; k += 1) {
    const inferred = plEdgeIdFromTopoLineKey(orderedLineKeys[k]);
    if (inferred && inspectPlSet.has(inferred)) {
      return pack(
        inferred,
        missionDistanceToInspectPlEdge(mission, inferred, px, py),
        "line_suffix_pl_inference"
      );
    }
  }

  return pack("", null, "unresolved_no_matching_pl");
}

/**
 * 仅从显式字段读取红线几何：unvisited_geometry_2d 或 explicit_unvisited_lines。
 * 不接受 topo_edges_pixel / metadata 拓扑参考自动推导。
 */
function readExplicitRedUnvisitedGeometry(mission) {
  const keys = ["unvisited_geometry_2d", "explicit_unvisited_lines"];
  const meta = mission?.metadata || {};
  const nested = meta.mission_metadata || {};
  for (const key of keys) {
    for (const b of [mission, meta, nested]) {
      const v = b?.[key];
      if (Array.isArray(v) && v.length) return { key, value: v };
    }
  }
  return null;
}

function appendPolylineToRedTrace(traceX, traceY, poly2d) {
  if (!poly2d || poly2d.length < 2) return 0;
  let n = 0;
  poly2d.forEach((pt) => {
    if (!pt || pt.length < 2) return;
    traceX.push(Number(pt[0]));
    traceY.push(Number(pt[1]));
    n += 1;
  });
  if (n >= 2) {
    traceX.push(null);
    traceY.push(null);
    return 1;
  }
  while (n > 0) {
    traceX.pop();
    traceY.pop();
    n -= 1;
  }
  return 0;
}

function flattenExplicitRedGeometry(mission) {
  const found = readExplicitRedUnvisitedGeometry(mission);
  if (!found) {
    return { x: [], y: [], polylineCount: 0, sourceKey: null };
  }
  const xs = [];
  const ys = [];
  let polylineCount = 0;
  const { value, key } = found;
  const first = value[0];
  const firstIsPointPair =
    Array.isArray(first) &&
    first.length >= 2 &&
    (Number.isFinite(Number(first[0])) || typeof first[0] === "string");

  if (firstIsPointPair) {
    polylineCount += appendPolylineToRedTrace(xs, ys, value);
  } else {
    value.forEach((block) => {
      if (!block) return;
      if (Array.isArray(block) && block.length >= 2 && Array.isArray(block[0])) {
        polylineCount += appendPolylineToRedTrace(xs, ys, block);
      } else if (typeof block === "object" && Array.isArray(block.geometry_2d)) {
        polylineCount += appendPolylineToRedTrace(xs, ys, block.geometry_2d);
      }
    });
  }
  if (xs.length && xs[xs.length - 1] === null) {
    xs.pop();
    ys.pop();
  }
  return { x: xs, y: ys, polylineCount, sourceKey: key };
}

/**
 * 红色「未巡检线路」：
 * - 默认 x/y 为空；绝不读取 topo_edges_pixel / metadata.topo_edges_pixel / mission_metadata.topo_edges_pixel。
 * - 仅当存在非空的 unvisited_geometry_2d 或 explicit_unvisited_lines 时写入折线几何。
 */
function buildUnvisitedRedLinesTrace(mission) {
  const sty = MAP_STYLES.pendingTopoLine;
  const trace = {
    x: [],
    y: [],
    mode: "lines",
    name: "未巡检线路",
    opacity: sty.opacity != null ? sty.opacity : 1,
    line: {
      color: sty.color,
      width: sty.width,
      dash: sty.dash,
    },
    hoverinfo: "skip",
    showlegend: true,
    legendgroup: "pending_topo",
    legendrank: 3,
  };

  const inspectEdges = collectInspectEdgeIdsFromSegments(mission);
  const flat = flattenExplicitRedGeometry(mission);
  trace.x = flat.x;
  trace.y = flat.y;

  const diag = {
    inspectEdges,
    explicitRedSource: flat.sourceKey,
    redPolylineCount: flat.polylineCount,
    redPointCount: flat.x.filter((v) => v != null && Number.isFinite(Number(v))).length,
    topo_edges_pixel_used_as_reference: false,
  };
  return { trace, diag };
}

function isSkippableMissionDebugSegment(seg) {
  if (!seg || typeof seg !== "object") return true;
  const typ = String(seg.type || "").toLowerCase();
  const role = String(seg.role || "").toLowerCase();
  const skipTypes = new Set([
    "debug",
    "overlay",
    "redline",
    "topo",
    "raw",
    "skeleton",
    "hsv",
    "legacy",
    "image_trace",
    "mask",
  ]);
  if (skipTypes.has(typ)) return true;
  if (/debug|overlay|redline|topo|raw|skeleton|hsv|legacy|mask|dummy|placeholder/.test(role)) return true;
  return false;
}

function inspectionPointHoverTextFromCd(cd) {
  const name = (cd && cd[3]) || "巡检点";
  return `<b>${name}</b><br>区段: ${cd[0]}<br>图片: ${cd[1]}<br>进度: ${cd[2]}`;
}

/** 将 inspect 段列表展开为 Plotly 折线（段间 null 断开） */
function flattenInspectSegsToPlotly(inspectSegs) {
  const inspectX = [];
  const inspectY = [];
  const inspectCustom = [];
  (inspectSegs || []).forEach((seg) => {
    const g = seg.geom || [];
    const hoverLine = seg.hoverLine || "";
    g.forEach((pt) => {
      inspectX.push(pt[0]);
      inspectY.push(pt[1]);
      inspectCustom.push(hoverLine);
    });
    inspectX.push(null);
    inspectY.push(null);
    inspectCustom.push(null);
  });
  while (inspectX.length && inspectX[inspectX.length - 1] === null) {
    inspectX.pop();
    inspectY.pop();
    inspectCustom.pop();
  }
  return { xs: inspectX, ys: inspectY, customs: inspectCustom };
}

const _BLUE_ROUTE_MIN_SEG_LEN2 = 1e-18;
const _BLUE_ROUTE_INSERT_MAX_DIST_PX = 500;

function logRoutePointAudit(total, inserted, missingRows) {
  const missing = missingRows.length;
  console.log(`[route-point-audit] total=${total} inserted=${inserted} missing=${missing}`);
  missingRows.forEach((r) => {
    const dist =
      r.nearest_dist == null || Number.isNaN(r.nearest_dist) ? "—" : Number(r.nearest_dist).toFixed(1);
    console.log(
      `[route-point-missing] point_id=${r.point_id} x=${r.x} y=${r.y} nearest_dist=${dist}`
    );
  });
}

/** 点到线段的最近投影（确定性 tie-break：弧长 s → segIdx → localJ → edgeIdx → pointIdx） */
function nearestBlueEdgeProjection(px, py, edges) {
  let best = null;
  for (let ei = 0; ei < edges.length; ei += 1) {
    const e = edges[ei];
    const denom = e.dx * e.dx + e.dy * e.dy;
    let t = 0;
    let d = 0;
    if (denom <= _BLUE_ROUTE_MIN_SEG_LEN2) {
      d = Math.hypot(px - e.ax, py - e.ay);
    } else {
      t = Math.max(0, Math.min(1, ((px - e.ax) * e.dx + (py - e.ay) * e.dy) / denom));
      const qx = e.ax + t * e.dx;
      const qy = e.ay + t * e.dy;
      d = Math.hypot(px - qx, py - qy);
    }
    const s = e.arc0 + t * e.len;
    if (
      !best ||
      d < best.d - 1e-9 ||
      (Math.abs(d - best.d) <= 1e-9 &&
        (s < best.s - 1e-9 ||
          (Math.abs(s - best.s) <= 1e-9 &&
            (e.segIdx < best.segIdx ||
              (e.segIdx === best.segIdx &&
                (e.localJ < best.localJ ||
                  (e.localJ === best.localJ && ei < best.ei)))))))
    ) {
      best = { s, d, t, segIdx: e.segIdx, localJ: e.localJ, ei };
    }
  }
  return best;
}
const BAD_SNAPPED_DIST_PX = 40;
const _badSnappedLogged = new Set();

function inspectionPointCoordPair(a, b) {
  const x = Number(a);
  const y = Number(b);
  return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
}

function inspectionPointRawXY(p) {
  if (!p || typeof p !== "object") return null;
  return (
    inspectionPointCoordPair(p.raw_x, p.raw_y) ||
    inspectionPointCoordPair(p.original_pixel_x, p.original_pixel_y) ||
    inspectionPointCoordPair(p.x, p.y)
  );
}

function isSnappedCoordUsable(p) {
  if (!p || typeof p !== "object") return false;
  const snapped = inspectionPointCoordPair(p.snapped_x, p.snapped_y);
  if (!snapped) return false;
  const raw = inspectionPointRawXY(p);
  if (!raw) return true;
  const dist = Math.hypot(raw.x - snapped.x, raw.y - snapped.y);
  if (dist <= BAD_SNAPPED_DIST_PX) return true;
  const pid = String(p.point_id || p.id || "").trim();
  if (!_badSnappedLogged.has(pid)) {
    _badSnappedLogged.add(pid);
    console.warn(
      `[bad-snapped] ${pid} raw=(${raw.x},${raw.y}) snapped=(${snapped.x},${snapped.y}) dist=${dist.toFixed(1)} edge=${p.edge_id || ""}`
    );
  }
  return false;
}

/** Dashboard 主图坐标：snapped 仅在 raw 偏移 ≤40px 时采用，否则 fallback */
function inspectionPointDisplayXY(p) {
  if (!p || typeof p !== "object") return null;
  if (isSnappedCoordUsable(p)) {
    const snapped = inspectionPointCoordPair(p.snapped_x, p.snapped_y);
    if (snapped) return snapped;
  }
  let r = inspectionPointCoordPair(p.display_x, p.display_y);
  if (r) return r;
  if (p.metadata && typeof p.metadata === "object") {
    r = inspectionPointCoordPair(p.metadata.display_x, p.metadata.display_y);
    if (r) return r;
  }
  r = inspectionPointRawXY(p);
  if (r) return r;
  const dr = p.detection_result || {};
  const rc = dr.raw_coord || p.raw_coord;
  if (Array.isArray(rc) && rc.length >= 2) {
    r = inspectionPointCoordPair(rc[0], rc[1]);
    if (r) return r;
  }
  const pos = p.pixel_position || p.position_2d || p.position || p.pos2d;
  if (Array.isArray(pos) && pos.length >= 2) {
    r = inspectionPointCoordPair(pos[0], pos[1]);
    if (r) return r;
  }
  return inspectionPointCoordPair(p.x, p.y);
}

function formatInspectionCoordPair(xy) {
  if (!xy || !Number.isFinite(xy.x) || !Number.isFinite(xy.y)) return "—";
  return `(${xy.x},${xy.y})`;
}

function logInspectionPointCoordAudit(p) {
  const pid = String(p.point_id || p.id || "").trim();
  if (!["IP_0005", "IP_0007", "IP_0012"].includes(pid)) return;
  const raw = inspectionPointRawXY(p);
  const snapped = inspectionPointCoordPair(p.snapped_x, p.snapped_y);
  const used = inspectionPointDisplayXY(p);
  console.log(
    `[point-coord] ${pid} raw=${formatInspectionCoordPair(raw)} snapped=${formatInspectionCoordPair(snapped)} used=${formatInspectionCoordPair(used)}`
  );
}

function logInspectionRouteBindingSummary(mission, inspectPoints, pointFrameIndex) {
  const allPts = Array.isArray(mission?.inspection_points)
    ? mission.inspection_points
    : [];
  const total = allPts.length;
  const bound = Object.keys(pointFrameIndex).sort();
  const missing = inspectPoints
    .map((pt) => pt.point_id)
    .filter((id) => pointFrameIndex[id] == null || pointFrameIndex[id] < 0);

  const snappedGroups = new Map();
  allPts.forEach((p) => {
    const xy = inspectionPointDisplayXY(p);
    if (!xy) return;
    const key = `${xy.x},${xy.y}`;
    if (!snappedGroups.has(key)) snappedGroups.set(key, []);
    snappedGroups.get(key).push(String(p.point_id || p.id || "").trim());
  });
  const duplicateSnapped = [];
  snappedGroups.forEach((ids, key) => {
    if (ids.length > 1) duplicateSnapped.push(`${key}:[${ids.join(",")}]`);
  });

  console.log(`[inspection points total] ${total}`);
  console.log(`[route bound points] [${bound.join(",")}]`);
  console.log(`[missing points] [${missing.join(",")}]`);
  console.log(
    `[duplicate display coord] ${duplicateSnapped.length ? duplicateSnapped.join("; ") : "none"}`
  );
}

/**
 * 用 mission.inspection_points 的 display x/y，在全部蓝色 inspect 段折线边上找最近投影；
 * 确定性插入：每点投影到最近边，同边上按 t 排序；不创建绿色连线 trace。
 * @param {Array<{ geom: number[][], hoverLine: string }>} inspectSegs
 * @param {object} mission
 */
function augmentInspectSegmentsWithInspectionPoints(inspectSegs, mission) {
  const drawPts = Array.isArray(mission?.inspection_points) ? mission.inspection_points : [];
  const m = drawPts.length;
  const inspectionRows = [];
  const missingRows = [];

  drawPts.forEach((p, idx) => {
    const xy = inspectionPointDisplayXY(p);
    const pointId = String(p.point_id || p.id || idx + 1).replace(/^point_/, "");
    if (!xy) {
      missingRows.push({
        point_id: pointId,
        x: p.x ?? "—",
        y: p.y ?? "—",
        nearest_dist: null,
      });
      return;
    }
    const totalPoints = m;
    const pointLabel = `巡检点 ${pointId}`;
    const segmentLabel = p.segment_id
      ? `巡检区段 ${String(p.segment_id).replace(/^seg_/, "")}`
      : "—";
    const cd = [
      segmentLabel,
      p.image_available ? "真实图片" : "占位图",
      `${p.progress_index || idx + 1}/${totalPoints}`,
      pointLabel,
    ];
    inspectionRows.push({ x: xy.x, y: xy.y, cd, idx, point_id: pointId });
  });

  const auditAndReturn = (inspectX, inspectY, inspectCustom, inserted) => {
    logRoutePointAudit(m, inserted, missingRows);
    return { inspectX, inspectY, inspectCustom };
  };

  if (!inspectSegs?.length) {
    drawPts.forEach((p, idx) => {
      if (missingRows.some((r) => r.point_id === String(p.point_id || p.id || idx + 1).replace(/^point_/, ""))) {
        return;
      }
      const xy = inspectionPointDisplayXY(p);
      if (xy) {
        missingRows.push({
          point_id: String(p.point_id || p.id || idx + 1).replace(/^point_/, ""),
          x: xy.x,
          y: xy.y,
          nearest_dist: null,
        });
      }
    });
    const flat = flattenInspectSegsToPlotly(inspectSegs || []);
    return auditAndReturn(flat.xs, flat.ys, flat.customs, 0);
  }

  if (!inspectionRows.length) {
    const flat = flattenInspectSegsToPlotly(inspectSegs);
    return auditAndReturn(flat.xs, flat.ys, flat.customs, 0);
  }

  const edges = [];
  let arcBase = 0;
  inspectSegs.forEach((seg, segIdx) => {
    const g = seg.geom;
    for (let j = 0; j < g.length - 1; j += 1) {
      const ax = g[j][0];
      const ay = g[j][1];
      const bx = g[j + 1][0];
      const by = g[j + 1][1];
      const dx = bx - ax;
      const dy = by - ay;
      const len = Math.hypot(dx, dy);
      edges.push({ ax, ay, bx, by, dx, dy, len, arc0: arcBase, segIdx, localJ: j });
      arcBase += len;
    }
  });

  if (!edges.length) {
    inspectionRows.forEach((row) => {
      missingRows.push({
        point_id: row.point_id,
        x: row.x,
        y: row.y,
        nearest_dist: null,
      });
    });
    const flat = flattenInspectSegsToPlotly(inspectSegs);
    return auditAndReturn(flat.xs, flat.ys, flat.customs, 0);
  }

  const projections = [];
  inspectionRows.forEach((row) => {
    const best = nearestBlueEdgeProjection(row.x, row.y, edges);
    if (!best) {
      missingRows.push({
        point_id: row.point_id,
        x: row.x,
        y: row.y,
        nearest_dist: null,
      });
      return;
    }
    // 始终插入最近边（重规划后蓝线可能暂时偏离 >500px，仍须稳定接入）
    projections.push({
      row,
      s: best.s,
      t: best.t,
      segIdx: best.segIdx,
      localJ: best.localJ,
      idx: row.idx,
      d: best.d,
    });
  });

  projections.sort((a, b) => {
    if (a.s !== b.s) return a.s - b.s;
    if (a.segIdx !== b.segIdx) return a.segIdx - b.segIdx;
    if (a.localJ !== b.localJ) return a.localJ - b.localJ;
    if (a.t !== b.t) return a.t - b.t;
    return a.idx - b.idx;
  });

  const groupKey = (segIdx, localJ) => `${segIdx}\t${localJ}`;
  const groups = new Map();
  projections.forEach((p) => {
    const k = groupKey(p.segIdx, p.localJ);
    if (!groups.has(k)) groups.set(k, []);
    groups.get(k).push(p);
  });
  groups.forEach((arr) => {
    arr.sort((a, b) => (a.t !== b.t ? a.t - b.t : a.idx - b.idx));
  });

  const inspectX = [];
  const inspectY = [];
  const inspectCustom = [];
  inspectSegs.forEach((seg, segIdx) => {
    const g = seg.geom;
    const hoverLine = seg.hoverLine;
    for (let j = 0; j < g.length - 1; j += 1) {
      inspectX.push(g[j][0]);
      inspectY.push(g[j][1]);
      inspectCustom.push(hoverLine);
      const inserts = groups.get(groupKey(segIdx, j)) || [];
      inserts.forEach((ins) => {
        inspectX.push(ins.row.x);
        inspectY.push(ins.row.y);
        inspectCustom.push(inspectionPointHoverTextFromCd(ins.row.cd));
      });
    }
    const last = g[g.length - 1];
    inspectX.push(last[0]);
    inspectY.push(last[1]);
    inspectCustom.push(hoverLine);
    inspectX.push(null);
    inspectY.push(null);
    inspectCustom.push(null);
  });
  while (inspectX.length && inspectX[inspectX.length - 1] === null) {
    inspectX.pop();
    inspectY.pop();
    inspectCustom.pop();
  }

  return auditAndReturn(inspectX, inspectY, inspectCustom, projections.length);
}

const ROUTE_SEQ_EPS = 1e-3;
const ON_GEOM_EPS = 15.01;
const POINT_FRAME_EPS = 1;

function pointToPolylineDist(px, py, geom) {
  if (!geom || geom.length < 2) return Infinity;
  let best = Infinity;
  for (let i = 0; i < geom.length - 1; i += 1) {
    const ax = Number(geom[i][0]);
    const ay = Number(geom[i][1]);
    const bx = Number(geom[i + 1][0]);
    const by = Number(geom[i + 1][1]);
    const dx = bx - ax;
    const dy = by - ay;
    const denom = dx * dx + dy * dy;
    let t = 0;
    if (denom > 1e-18) {
      t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / denom));
    }
    const qx = ax + t * dx;
    const qy = ay + t * dy;
    const d = Math.hypot(px - qx, py - qy);
    if (d < best) best = d;
  }
  return best;
}

function projectPointOnPolyline(px, py, geom) {
  if (!geom || geom.length < 2) return null;
  let arcBase = 0;
  let best = null;
  for (let j = 0; j < geom.length - 1; j += 1) {
    const ax = Number(geom[j][0]);
    const ay = Number(geom[j][1]);
    const bx = Number(geom[j + 1][0]);
    const by = Number(geom[j + 1][1]);
    const dx = bx - ax;
    const dy = by - ay;
    const denom = dx * dx + dy * dy;
    let t = 0;
    let d = 0;
    if (denom <= 1e-18) {
      d = Math.hypot(px - ax, py - ay);
    } else {
      t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / denom));
      const qx = ax + t * dx;
      const qy = ay + t * dy;
      d = Math.hypot(px - qx, py - qy);
    }
    const s = arcBase + t * Math.hypot(dx, dy);
    if (!best || s < best.arcS - 1e-9 || (Math.abs(s - best.arcS) <= 1e-9 && j < best.edgeIndex)) {
      best = { edgeIndex: j, t, arcS: s, d };
    }
    arcBase += Math.hypot(dx, dy);
  }
  return best;
}

function findVertexMatchIndex(pt, geom, eps) {
  for (let i = 0; i < geom.length; i += 1) {
    const vx = Number(geom[i][0]);
    const vy = Number(geom[i][1]);
    if (Math.abs(vx - pt.x) <= eps && Math.abs(vy - pt.y) <= eps) return i;
  }
  return -1;
}

function polylineVertexArcs(geom) {
  const arcs = [0];
  for (let i = 1; i < geom.length; i += 1) {
    const ax = Number(geom[i - 1][0]);
    const ay = Number(geom[i - 1][1]);
    const bx = Number(geom[i][0]);
    const by = Number(geom[i][1]);
    arcs.push(arcs[i - 1] + Math.hypot(bx - ax, by - ay));
  }
  return arcs;
}

function normalizeRoutePointIds(ids) {
  const out = [];
  (ids || []).forEach((id) => {
    const pid = String(id || "").trim();
    if (pid && !out.includes(pid)) out.push(pid);
  });
  return out;
}

function mergeRoutePointIds(existing, incoming) {
  return normalizeRoutePointIds([...(existing || []), ...(incoming || [])]);
}

function frameRouteFields(routePointIds) {
  const routePointIdsNorm = normalizeRoutePointIds(routePointIds);
  return {
    routePointIds: routePointIdsNorm,
    routePointId: routePointIdsNorm.length === 1 ? routePointIdsNorm[0] : null,
  };
}

function appendPointIdToItem(item, pointId) {
  if (!pointId) return;
  if (!item.point_ids) item.point_ids = [];
  if (!item.point_ids.includes(pointId)) item.point_ids.push(pointId);
  item.kind = "inspection";
}

function expandInspectSegmentByArc(seg, pointsOnSeg, segmentId) {
  const geom = seg.geometry_2d || [];
  const sid = segmentId || String(seg.segment_id || "");
  const rawCount = geom.length;
  const failedPoints = [];
  const insertMeta = [];

  if (geom.length < 2) {
    (pointsOnSeg || []).forEach((pt) => {
      console.warn(`[point-insert-failed] ${pt.point_id} reason=empty_geometry`);
      failedPoints.push({ pt, reason: "empty_geometry" });
    });
    return { frames: [], insertMeta, failedPoints, rawCount, pointCount: (pointsOnSeg || []).length };
  }

  const vertexArcs = polylineVertexArcs(geom);
  const items = geom.map((p, i) => ({
    x: Number(p[0]),
    y: Number(p[1]),
    arc: vertexArcs[i],
    kind: "vertex",
    point_ids: [],
    orderIndex: -1,
  }));

  const insertedIds = new Set();

  (pointsOnSeg || []).forEach((pt) => {
    if (!Number.isFinite(pt.x) || !Number.isFinite(pt.y)) {
      console.warn(`[point-insert-failed] ${pt.point_id} reason=invalid_coords`);
      failedPoints.push({ pt, reason: "invalid_coords" });
      return;
    }

    const vi = findVertexMatchIndex(pt, geom, POINT_FRAME_EPS);
    if (vi >= 0) {
      appendPointIdToItem(items[vi], pt.point_id);
      insertedIds.add(pt.point_id);
      insertMeta.push({
        point_id: pt.point_id,
        arc: vertexArcs[vi],
        x: pt.x,
        y: pt.y,
      });
      return;
    }

    const proj = projectPointOnPolyline(pt.x, pt.y, geom);
    if (!proj || !Number.isFinite(proj.arcS)) {
      console.warn(`[point-insert-failed] ${pt.point_id} reason=no_projection`);
      failedPoints.push({ pt, reason: "no_projection" });
      return;
    }

    items.push({
      x: pt.x,
      y: pt.y,
      arc: proj.arcS,
      t: proj.t,
      kind: "inspection",
      point_ids: [pt.point_id],
      orderIndex: pt.orderIndex != null ? pt.orderIndex : 0,
    });
    insertedIds.add(pt.point_id);
    insertMeta.push({
      point_id: pt.point_id,
      arc: proj.arcS,
      x: pt.x,
      y: pt.y,
    });
  });

  (pointsOnSeg || []).forEach((pt) => {
    if (!insertedIds.has(pt.point_id)) {
      if (!failedPoints.some((f) => f.pt.point_id === pt.point_id)) {
        console.warn(`[point-insert-failed] ${pt.point_id} reason=not_inserted`);
        failedPoints.push({ pt, reason: "not_inserted" });
      }
    }
  });

  items.sort((a, b) => {
    if (Math.abs(a.arc - b.arc) > 1e-9) return a.arc - b.arc;
    if (a.kind !== b.kind) return a.kind === "vertex" ? -1 : 1;
    if (a.kind === "inspection" && b.kind === "inspection") {
      return a.orderIndex - b.orderIndex;
    }
    return 0;
  });

  const deduped = [];
  items.forEach((it) => {
    const prev = deduped[deduped.length - 1];
    if (
      prev &&
      Math.abs(prev.x - it.x) <= ROUTE_SEQ_EPS &&
      Math.abs(prev.y - it.y) <= ROUTE_SEQ_EPS
    ) {
      prev.point_ids = mergeRoutePointIds(prev.point_ids, it.point_ids);
      if (it.kind === "inspection") {
        prev.kind = "inspection";
      }
      return;
    }
    deduped.push({
      ...it,
      point_ids: normalizeRoutePointIds(it.point_ids),
    });
  });

  const frames = deduped.map((it) => ({
    x: it.x,
    y: it.y,
    ...frameRouteFields(it.kind === "inspection" ? it.point_ids : []),
  }));

  const pointCount = (pointsOnSeg || []).length;
  const finalCount = frames.length;
  if (finalCount) {
    console.log(
      `[segment-expanded] ${sid} raw=${rawCount} points=${pointCount} final=${finalCount} first=(${frames[0].x},${frames[0].y}) last=(${frames[finalCount - 1].x},${frames[finalCount - 1].y})`
    );
  }

  return {
    frames,
    insertMeta,
    failedPoints,
    rawCount,
    pointCount,
    finalCount,
  };
}

function buildMissionRouteSequence(mission, rowsWithXY) {
  const markers = mission?.markers || {};
  const meta = mission?.metadata || {};
  const sx = Number(markers.start?.x ?? meta.start?.x ?? meta.start_point?.x);
  const sy = Number(markers.start?.y ?? meta.start?.y ?? meta.start_point?.y);
  const segments = mission?.segments || [];
  const dist2 = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

  const inspectPoints = [];
  (rowsWithXY || []).forEach((r, i) => {
    const p = r.point || {};
    const pointId = String(p.point_id || p.id || i + 1).trim();
    let boundSeg = String(p.segment_id || "").trim();
    if (!boundSeg) {
      for (const seg of segments) {
        if (isSkippableMissionDebugSegment(seg) || seg.type !== "inspect") continue;
        const geom = seg.geometry_2d || [];
        if (geom.length < 2) continue;
        if (pointToPolylineDist(r.x, r.y, geom) <= ON_GEOM_EPS) {
          boundSeg = String(seg.segment_id || "");
          break;
        }
      }
    }
    if (!boundSeg) {
      let bestD = Infinity;
      for (const seg of segments) {
        if (isSkippableMissionDebugSegment(seg) || seg.type !== "inspect") continue;
        const d = pointToPolylineDist(r.x, r.y, seg.geometry_2d || []);
        if (d < bestD) {
          bestD = d;
          boundSeg = String(seg.segment_id || "");
        }
      }
    }
    inspectPoints.push({
      point_id: pointId,
      id: pointId,
      x: r.x,
      y: r.y,
      image_url:
        (typeof p.image_url === "string" && p.image_url.trim()) ||
        `/api/inspection-image/${pointId}.jpg`,
      segment_id: boundSeg,
      edge_id: p.edge_id || "",
      orderIndex: i,
    });
  });

  const pointsBySegment = new Map();
  inspectPoints.forEach((pt) => {
    const sid = pt.segment_id;
    if (!sid) return;
    if (!pointsBySegment.has(sid)) pointsBySegment.set(sid, []);
    pointsBySegment.get(sid).push(pt);
  });

  const sequence = [];
  let skippedStartDup = false;

  const pushFrame = (frame) => {
    const prev = sequence[sequence.length - 1];
    const incomingIds = frame.routePointIds || (frame.routePointId ? [frame.routePointId] : []);
    if (prev && dist2(prev, frame) <= ROUTE_SEQ_EPS) {
      const merged = mergeRoutePointIds(prev.routePointIds, incomingIds);
      prev.routePointIds = merged;
      prev.routePointId = merged.length === 1 ? merged[0] : null;
      return;
    }
    sequence.push({
      ...frame,
      ...frameRouteFields(incomingIds),
    });
  };

  if (Number.isFinite(sx) && Number.isFinite(sy)) {
    pushFrame({
      segment_id: "start",
      type: "start",
      segment_type: "start",
      x: sx,
      y: sy,
      edge_id: null,
      routePointIds: [],
      routePointId: null,
    });
  }

  segments.forEach((seg) => {
    if (isSkippableMissionDebugSegment(seg)) return;
    const geom = seg.geometry_2d || [];
    if (geom.length < 2) return;
    const sid = String(seg.segment_id || "");
    const stype =
      seg.type === "inspect"
        ? "inspect"
        : seg.type === "connect"
          ? "connect"
          : String(seg.type || "move");
    const edgeId = seg.edge_id != null ? String(seg.edge_id) : null;

    if (stype === "inspect") {
      const segStartIdx = sequence.length;
      const expanded = expandInspectSegmentByArc(
        seg,
        pointsBySegment.get(sid) || [],
        sid
      );
      expanded.frames.forEach((v, idx) => {
        if (!skippedStartDup && idx === 0 && sequence.length) {
          const startFrame = sequence[0];
          if (
            startFrame?.type === "start" &&
            dist2(startFrame, v) <= ROUTE_SEQ_EPS
          ) {
            skippedStartDup = true;
            return;
          }
        }
        pushFrame({
          segment_id: sid,
          type: stype,
          segment_type: stype,
          x: v.x,
          y: v.y,
          edge_id: edgeId,
          routePointIds: v.routePointIds || (v.routePointId ? [v.routePointId] : []),
          routePointId: v.routePointId || null,
        });
      });
      expanded.insertMeta.forEach((meta) => {
        let frameIdx = -1;
        for (let i = segStartIdx; i < sequence.length; i += 1) {
          const ids = sequence[i].routePointIds || [];
          if (ids.includes(meta.point_id)) {
            frameIdx = i;
            break;
          }
        }
        if (frameIdx < 0) return;
        console.log(
          `[point-insert] ${meta.point_id} segment=${sid} arc=${Number(meta.arc).toFixed(2)} frame=${frameIdx} x=${meta.x} y=${meta.y}`
        );
      });
      return;
    }

    geom.forEach((p, idx) => {
      const x = Number(p[0]);
      const y = Number(p[1]);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      if (!skippedStartDup && idx === 0 && sequence.length) {
        const startFrame = sequence[0];
        if (
          startFrame?.type === "start" &&
          dist2(startFrame, { x, y }) <= ROUTE_SEQ_EPS
        ) {
          skippedStartDup = true;
          return;
        }
      }
      pushFrame({
        segment_id: sid,
        type: stype,
        segment_type: stype,
        x,
        y,
        edge_id: edgeId,
        routePointIds: [],
        routePointId: null,
      });
    });
  });

  const pointFrameIndex = {};
  sequence.forEach((f, i) => {
    (f.routePointIds || []).forEach((pid) => {
      pointFrameIndex[pid] = i;
    });
  });
  inspectPoints.forEach((pt) => {
    pt.trigger_frame = pointFrameIndex[pt.point_id] ?? -1;
  });

  logInspectionRouteBindingSummary(mission, inspectPoints, pointFrameIndex);

  sequence.forEach((f, i) => {
    const ids = f.routePointIds || [];
    if (ids.length > 1) {
      console.log(
        `[multi-point-frame] frame=${i} ids=${ids.join(",")} coord=(${f.x},${f.y})`
      );
    }
  });

  window.__MISSION_ROUTE_SEQUENCE = sequence;
  window.__INSPECTION_POINT_BINDINGS = inspectPoints;

  const segSummary = [];
  let lastKey = null;
  sequence.forEach((f) => {
    const key = `${f.segment_id}:${f.segment_type || f.type}`;
    if (key !== lastKey) {
      segSummary.push(key);
      lastKey = key;
    }
  });
  const first = sequence[0];
  if (first) {
    console.log(
      `[mission-route] frames=${sequence.length} first=(${first.x},${first.y})`
    );
    console.log(
      `[mission-route] first_20_segments=${segSummary.slice(0, 20).join(", ")}`
    );
  } else {
    console.warn("[mission-route] empty sequence");
  }

  return sequence;
}

function cacheDrawnRouteForPlayback(mission, connectX, connectY, inspectX, inspectY, rowsWithXY) {
  const markers = mission?.markers || {};
  const sx = Number(markers.start?.x);
  const sy = Number(markers.start?.y);
  const start =
    Number.isFinite(sx) && Number.isFinite(sy) ? { x: sx, y: sy } : null;

  const inspectPoints = (rowsWithXY || []).map((r, i) => {
    const p = r.point || {};
    const pointId = String(p.point_id || p.id || i + 1).trim();
    return {
      point_id: pointId,
      id: pointId,
      x: r.x,
      y: r.y,
      image_url:
        (typeof p.image_url === "string" && p.image_url.trim()) ||
        `/api/inspection-image/${pointId}.jpg`,
      segment_id: p.segment_id || "",
      edge_id: p.edge_id || "",
      orderIndex: i,
    };
  });

  window.__DRAWN_ROUTE_DEBUG = {
    connect_x: connectX.slice(),
    connect_y: connectY.slice(),
    inspect_x: inspectX.slice(),
    inspect_y: inspectY.slice(),
    inspect_points: inspectPoints,
    start_x: start ? start.x : null,
    start_y: start ? start.y : null,
  };

  window.DRAWN_ROUTE_FOR_PLAYBACK = {
    start,
    orange: { x: connectX.slice(), y: connectY.slice() },
    blue: { x: inspectX.slice(), y: inspectY.slice() },
  };

  const countFinite = (xs) =>
    (xs || []).filter((v) => v != null && Number.isFinite(Number(v))).length;
  const orangeN = countFinite(connectX);
  const blueN = countFinite(inspectX);
  if (start) {
    console.log(
      `[drawn-route-cache] start=(${start.x},${start.y}) orange=${orangeN} blue=${blueN} inspect_points=${inspectPoints.length}`
    );
  } else {
    console.warn("[drawn-route-cache] missing start marker");
  }
}

function buildMissionTraces(missionPayload) {
  const mission = missionPayload || getCurrentMission();
  _badSnappedLogged.clear();
  const inspectionPointsLengthAtEntry = mission?.inspection_points?.length ?? 0;
  console.log(
    "[dashboard-render] inspection_points=",
    inspectionPointsLengthAtEntry
  );
  console.log(
    "[dashboard-render] statistics.num_inspection_points=",
    mission?.statistics?.num_inspection_points
  );
  console.log(
    "[point-flow] ui_draw_points=",
    inspectionPointsLengthAtEntry,
    "cv_after_merge=",
    mission?.metadata?.image_detection_stats?.after_merge ??
      mission?.metadata?.image_detection_stats?.merged_points
  );
  const segments = mission?.segments || [];
  const inspectSegs = [];
  const connectX = [],
    connectY = [],
    connectCustom = [];

  segments.forEach((seg) => {
    if (isSkippableMissionDebugSegment(seg)) return;
    const geom = seg.geometry_2d || [];
    if (geom.length < 2) return;
    const segLabel = seg.segment_id
      ? `巡检区段 ${String(seg.segment_id).replace(/^seg_/, "")}`
      : "巡检区段";
    const typeLabel = seg.type === "inspect" ? "巡检段" : seg.type === "connect" ? "连接段" : "区段";
    const hoverLine = `${segLabel}<br>${typeLabel} · ${Math.round(seg.length || 0)}px`;

    if (seg.type === "inspect") {
      inspectSegs.push({
        geom: geom.map((p) => [Number(p[0]), Number(p[1])]),
        hoverLine,
      });
    } else if (seg.type === "connect") {
      connectX.push(...geom.map((p) => p[0]), null);
      connectY.push(...geom.map((p) => p[1]), null);
      connectCustom.push(...geom.map(() => hoverLine), null);
    }
  });

  [connectX, connectY, connectCustom].forEach((arr) => {
    if (arr.length && arr[arr.length - 1] === null) arr.pop();
  });

  /** Dashboard：巡检点 marker（不连线）；蓝线顶点 = 原 inspect 折线 + 按弧长投影插入的巡检点 */
  const drawPts = Array.isArray(mission.inspection_points) ? mission.inspection_points : [];
  const rowsWithXY = [];
  if (drawPts.length) {
    const totalPoints = drawPts.length;
    drawPts.forEach((p, idx) => {
      logInspectionPointCoordAudit(p);
      const xy = inspectionPointDisplayXY(p);
      if (!xy) return;
      const pointLabel = `巡检点 ${String(p.point_id || p.id || idx + 1).replace(/^point_/, "")}`;
      const segmentLabel = p.segment_id
        ? `巡检区段 ${String(p.segment_id).replace(/^seg_/, "")}`
        : "—";
      const cd = [
        segmentLabel,
        p.image_available ? "真实图片" : "占位图",
        `${p.progress_index || idx + 1}/${totalPoints}`,
        pointLabel,
      ];
      rowsWithXY.push({ x: xy.x, y: xy.y, cd, missionIndex: idx, point: p });
    });
  }

  let inspectX = [];
  let inspectY = [];
  let inspectCustom = [];
  if (inspectSegs.length) {
    const aug = augmentInspectSegmentsWithInspectionPoints(inspectSegs, mission);
    inspectX = aug.inspectX;
    inspectY = aug.inspectY;
    inspectCustom = aug.inspectCustom;
  } else if (rowsWithXY.length) {
    logRoutePointAudit(drawPts.length, 0, rowsWithXY.map((r) => ({
      point_id: String(r.point?.point_id || r.point?.id || r.missionIndex + 1),
      x: r.x,
      y: r.y,
      nearest_dist: null,
    })));
  }

  const traces = [];
  const { trace: unvisitedRedTrace, diag: redDiag } = buildUnvisitedRedLinesTrace(mission);
  traces.push(unvisitedRedTrace);

  const redTraceFiniteCount = (unvisitedRedTrace.x || []).filter(
    (v) => v != null && Number.isFinite(Number(v))
  ).length;
  console.log(
    "[dashboard-render] red_unvisited_trace_points=",
    redTraceFiniteCount
  );

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
      legendrank: 1,
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
      legendrank: 2,
    });
  }

  if (rowsWithXY.length) {
    traces.push({
      x: rowsWithXY.map((r) => r.x),
      y: rowsWithXY.map((r) => r.y),
      mode: "markers",
      name: "巡检点",
      marker: {
        color: "#22c55e",
        size: 11,
        symbol: "circle",
        line: { color: "#052e16", width: 1.5 },
      },
      text: rowsWithXY.map((r) => r.cd[3]),
      customdata: rowsWithXY.map((r) => {
        const p = r.point;
        const pointId = String(p.point_id || p.id || "").trim();
        return {
          point_id: pointId,
          image_url:
            (typeof p.image_url === "string" && p.image_url.trim()) ||
            `/api/inspection-image/${pointId}.jpg`,
        };
      }),
      hovertemplate:
        "<b>%{text}</b><br>" +
        "point_id: %{customdata.point_id}<br>" +
        "url: %{customdata.image_url}<br>" +
        "<span style='opacity:0.85'>点击在侧栏查看巡检图</span><extra></extra>",
      hoverinfo: "text",
      showlegend: true,
      legendgroup: "points",
      legendrank: 3,
    });
    console.log(
      "[demo-point-render] inspection_points=",
      drawPts.length,
      "rendered_green_points=",
      rowsWithXY.length
    );
  } else {
    console.log("[demo-point-render] inspection_points=0 rendered_green_points=0");
  }

  const markers = mission.markers || {};
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
      legendrank: 7,
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
      legendrank: 8,
    });
  }

  cacheDrawnRouteForPlayback(mission, connectX, connectY, inspectX, inspectY, rowsWithXY);
  buildMissionRouteSequence(mission, rowsWithXY);

  return traces;
}

function buildLegendLayout(fullscreen = false) {
  return {
    orientation: "v",
    x: 0.02,
    y: 0.02,
    xanchor: "left",
    yanchor: "bottom",
    bgcolor: "rgba(10, 16, 28, 0.72)",
    bordercolor: "rgba(100,160,200,0.18)",
    borderwidth: 1,
    font: { size: fullscreen ? 11 : 10, color: "#b8d4e8" },
    traceorder: "normal",
    itemsizing: "constant",
    itemwidth: 28,
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
  return getFixedSatelliteLayoutImages(result);
}

function buildMissionLayout(result, options = {}) {
  const { fullscreen = false } = options;
  const mission = result || buildIdleImageMapPayload();
  const { width, height } = getPipelineBackgroundSpec(mission);
  const transparent = isImagePipeline() && usesImagePixelCoords(mission);
  const hiddenPixelAxes = transparent
    ? {
        showgrid: false,
        zeroline: false,
        showline: false,
        showticklabels: false,
        ticks: "",
      }
    : null;
  const margin = transparent
    ? { l: 0, r: 0, t: 0, b: 0 }
    : fullscreen
      ? { l: 6, r: 6, t: 2, b: 4 }
      : { l: 14, r: 8, t: 4, b: 6 };

  const layout = {
    paper_bgcolor: transparent ? "rgba(0,0,0,0)" : "#1a2332",
    plot_bgcolor: transparent ? "rgba(0,0,0,0)" : "rgba(255,255,255,0.06)",
    font: { color: "#e6edf3", size: 11 },
    margin,
    legend: buildLegendLayout(fullscreen),
    hovermode: "closest",
    images: [],
    uirevision: `mission-map-v3-${typeof MissionStore !== "undefined" && MissionStore.version != null ? MissionStore.version : 0}`,
    autosize: true,
    xaxis: {
      title: "",
      range: [0, width],
      domain: [0.0, 1.0],
      autorange: false,
      fixedrange: true,
      constrain: "domain",
      ...(hiddenPixelAxes || {
        showgrid: true,
        gridcolor: "rgba(255,255,255,0.10)",
        tickfont: { size: 9, color: "#5a7a94" },
        zeroline: false,
      }),
    },
    yaxis: {
      title: "",
      range: [height, 0],
      domain: [0.0, 1.0],
      autorange: false,
      fixedrange: true,
      scaleanchor: "x",
      scaleratio: 1,
      constrain: "domain",
      ...(hiddenPixelAxes || {
        showgrid: true,
        gridcolor: "rgba(255,255,255,0.10)",
        tickfont: { size: 9, color: "#5a7a94" },
        zeroline: false,
      }),
    },
  };

  if (fullscreen) {
    const sz = getFullscreenPlotSize();
    layout.autosize = false;
    layout.width = sz.width;
    layout.height = sz.height;
  }

  return applyFixedSatelliteToLayout(layout, mission);
}

function getMissionMapPlotLayout(mission, plotId = "mapPlot") {
  const payload = mission || buildIdleImageMapPayload() || { metadata: { pipeline: "image" } };
  return buildMissionLayout(payload, { fullscreen: plotId === "mapPlotFs" });
}

function plotlyUpdateMissionPlot(plotId, traces, mission, options = {}) {
  const normalized = normalizeMissionResult(mission || getCurrentMission() || {});
  const layout = applyFixedSatelliteToLayout(buildMissionLayout(normalized, options), normalized);
  const config = {
    responsive: !options.fullscreen,
    displayModeBar: true,
    scrollZoom: true,
  };
  const el = document.getElementById(plotId);
  if (!el) return Promise.resolve();
  const tracesCopy = Array.isArray(traces) ? traces.slice() : [];
  if (state.plotReady[plotId] && el.data) {
    if (plotId === "mapPlot" || plotId === "mapPlotFs") {
      return Plotly.newPlot(plotId, tracesCopy, layout, config).then(() => {
        state.plotReady[plotId] = true;
      });
    }
    return Plotly.react(plotId, tracesCopy, layout, config);
  }
  return Plotly.newPlot(plotId, tracesCopy, layout, config).then(() => {
    state.plotReady[plotId] = true;
  });
}

function buildPlotFromMission(result, options = {}) {
  const normalized = normalizeMissionResult(result || {});
  let traces = buildMissionTraces(normalized);
  if (typeof window.appendPlaybackTraces === "function") {
    traces = window.appendPlaybackTraces(traces, normalized);
  }
  const layout = buildMissionLayout(normalized, options);
  return {
    traces: Array.isArray(traces) ? traces.slice() : [],
    layout,
    config: {
      responsive: !options.fullscreen,
      displayModeBar: true,
      scrollZoom: true,
    },
  };
}

window.resolveCleanMapImagePath = resolveCleanMapImagePath;
window.buildMapBackgroundForResult = buildMapBackgroundForResult;
window.buildBackgroundImages = buildBackgroundImages;
window.getImagePipelineMapPath = getImagePipelineMapPath;
window.getPipelineBackgroundSpec = getPipelineBackgroundSpec;
window.toStaticDataUrl = toStaticDataUrl;
window.getFixedSatelliteLayoutImages = getFixedSatelliteLayoutImages;
window.applyFixedSatelliteToLayout = applyFixedSatelliteToLayout;
window.getMissionMapPlotLayout = getMissionMapPlotLayout;
window.plotlyUpdateMissionPlot = plotlyUpdateMissionPlot;
window.refreshMissionMap = refreshMissionMap;

function buildWeatherZoneTraces() {
  return [];
}

function buildExperimentOverlayTraces() {
  return [];
}

function buildAdaptiveReplanTraces() {
  return [];
}

function logInspectionImageStatsFromMission(mission) {
  const st = mission?.metadata?.inspection_image_stats;
  if (st && typeof st === "object") {
    console.log("[inspection-image]", `total_images=${Number(st.total_images) || 0}`);
    console.log("[inspection-image]", `mapped_points=${Number(st.mapped_points) || 0}`);
    console.log("[inspection-image]", `missing_images=${Number(st.missing_images) || 0}`);
    return;
  }
  console.log("[inspection-image]", "total_images=— (reload mission via server to refresh stats)");
  console.log("[inspection-image]", "mapped_points=—");
  console.log("[inspection-image]", "missing_images=—");
}

function onMissionLoaded(result, options = {}) {
  const normalized = normalizeMissionResult(result || {});
  if (window.MissionStore) {
    if (!options.skipStoreLoad) {
      MissionStore.loadFromDashboard(normalized, { kind: options.kind || "initial" });
    }
  } else {
    state.lastResult = normalized;
  }
  logInspectionImageStatsFromMission(getCurrentMission() || normalized);
  updateSpacingControlsVisibility();
  updateReplanInputLimits();
  prefillReplanFormFromMission(getCurrentMission() || normalized);
  if (typeof window.onMissionLoadedForPlayback === "function") {
    window.onMissionLoadedForPlayback(getCurrentMission() || normalized);
  }
}

function bindMapClick(plotId) {
  const el = document.getElementById(plotId);
  if (!el || el.__dashMapClick) return;
  el.__dashMapClick = true;
  el.on("plotly_click", (ev) => {
    if (!ev?.points?.length) return;
    const p = ev.points[0];
    const data = p.data || {};
    const traceName = data.name;

    if (traceName === "巡检点" && p.customdata && typeof p.customdata === "object" && !Array.isArray(p.customdata)) {
      const cd = p.customdata;
      const point_id = String(cd.point_id || "").trim();
      if (!point_id) return;
      const image_url =
        (typeof cd.image_url === "string" && cd.image_url.trim()) ||
        `/api/inspection-image/${point_id}.jpg`;
      console.log(
        `[inspection-image] map_click point_id=${point_id} url=${image_url}`
      );
      const mission = typeof getCurrentMission === "function" ? getCurrentMission() : null;
      const ipFull =
        mission?.inspection_points?.find(
          (x) => String(x.point_id || x.id || "").trim() === point_id
        ) || { point_id, id: point_id, image_url };
      if (typeof window.showInspectionImage === "function") {
        window.showInspectionImage(ipFull, { logAs: "map_click" });
      }
      if (mission?.inspection_points?.length && typeof window.showInspectCardForDashboardPoint === "function") {
        const missionIdx = mission.inspection_points.findIndex(
          (x) => String(x.point_id || x.id || "").trim() === point_id
        );
        if (missionIdx >= 0) {
          const ip = mission.inspection_points[missionIdx];
          const total = mission.inspection_points.length;
          window.showInspectCardForDashboardPoint(ip, missionIdx + 1, total, {
            skipImageRefresh: true,
          });
        }
      }
    }
  });
}

function renderMission(result, targetId = "mapPlot", options = {}) {
  if (!result) return;
  readUiToState();
  updateMapModeHint();

  purgeSingleMissionPlot(targetId);

  const normalized = normalizeMissionResult(result);
  const plot = buildPlotFromMission(normalized, options);
  const el = document.getElementById(targetId);
  if (!el) return;

  const draw = () => {
    bindMapClick(targetId);
    syncHtmlBasemapImages(getCurrentMission() || normalized);
    if (options.fullscreen) Plotly.Plots.resize(targetId);
  };

  const afterDraw = () => {
    draw();
    const mission = getCurrentMission() || normalized;
    if (typeof window.resetPlayback === "function" && mission && targetId === "mapPlot") {
      window.resetPlayback(mission);
    }
  };

  const layout = applyFixedSatelliteToLayout(plot.layout, normalized);
  const tracesFresh = Array.isArray(plot.traces) ? plot.traces.slice() : [];

  if (state.plotReady[targetId] && el.data) {
    Plotly.newPlot(targetId, tracesFresh, layout, plot.config).then(afterDraw);
  } else {
    Plotly.newPlot(targetId, tracesFresh, layout, plot.config).then(() => {
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

function updateReplanInputLimits() {
  if (!isImagePipeline()) return;
  const { width, height } = getImageSize(getCurrentMission() || { map_background: state.mapConfig });
  if (!width || !height) return;
  [
    ["replanStartX", 0, width],
    ["replanEndX", 0, width],
    ["replanStartY", 0, height],
    ["replanEndY", 0, height],
  ].forEach(([id, min, max]) => {
    const el = $(id);
    if (!el) return;
    el.min = String(min);
    el.max = String(max);
  });
}

function validateReplanCoords() {
  const sx = parseFloat($("replanStartX")?.value);
  const sy = parseFloat($("replanStartY")?.value);
  const ex = parseFloat($("replanEndX")?.value);
  const ey = parseFloat($("replanEndY")?.value);
  if ([sx, sy, ex, ey].some((v) => Number.isNaN(v))) {
    return { ok: false, message: "请填写完整的起点/终点坐标" };
  }
  const { width, height } = getImageSize(getCurrentMission() || { map_background: state.mapConfig });
  if (width > 0 && height > 0) {
    if (sx < 0 || sx > width || ex < 0 || ex > width || sy < 0 || sy > height || ey < 0 || ey > height) {
      return {
        ok: false,
        message: `起点/终点坐标超出图像范围：x∈[0,${width}]，y∈[0,${height}]`,
      };
    }
  }
  return { ok: true, start_x: sx, start_y: sy, end_x: ex, end_y: ey };
}

function prefillReplanFormFromMission(mission) {
  if (!mission || !isImagePipeline()) return;
  const mk = mission.markers || {};
  if (mk.start && $("replanStartX") && $("replanStartY")) {
    $("replanStartX").value = String(Math.round(Number(mk.start.x)));
    $("replanStartY").value = String(Math.round(Number(mk.start.y)));
  }
  if (mk.end && $("replanEndX") && $("replanEndY")) {
    $("replanEndX").value = String(Math.round(Number(mk.end.x)));
    $("replanEndY").value = String(Math.round(Number(mk.end.y)));
  }
}

let replanRequestInFlight = false;

async function runStartEndReplan() {
  if (replanRequestInFlight) {
    setStatus("正在重规划，请等待当前请求完成…", "running");
    return;
  }
  if (!isImagePipeline()) {
    setStatus("起终点重规划仅支持图像管线", "err");
    return;
  }
  const check = validateReplanCoords();
  if (!check.ok) {
    setStatus(check.message, "err");
    return;
  }

  const body = {
    inspection_point_source: getInspectionPointSource(),
    image_path: getImagePathForSource(getInspectionPointSource()),
    start_x: check.start_x,
    start_y: check.start_y,
    end_x: check.end_x,
    end_y: check.end_y,
  };

  const replanButton = $("runReplanBtn");
  const previousButtonText = replanButton?.textContent || "应用起终点 / 重新规划路径";
  replanRequestInFlight = true;
  if (replanButton) {
    replanButton.disabled = true;
    replanButton.textContent = "正在重规划…";
  }
  setTaskControlPhase("replanning");
  setStatus("正在按起终点重新规划路径…", "running");

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
    onMissionLoaded(normalized, { kind: "replan" });
    state.imageMissionAvailable = true;

    const mission = getCurrentMission();
    renderStats(mission?.statistics);
    renderVisitOrder(mission?.visit_order);
    renderMission(mission || normalized);
    renderMeta(mission?.metadata || data.metadata || {});
    updateDownloadLinks(data.output_files || {}, "image");
    updateReplanInputLimits();

    const nPts = mission?.inspection_points?.length ?? data.inspection_points?.length ?? 0;
    setTaskControlPhase("mission_ready");
    setStatus(`起终点重规划完成 · ${data.statistics?.num_segments ?? 0} 段 · 巡检点 ${nPts}`, "ok");

    if (typeof window.syncPlaybackAfterMissionChange === "function") {
      window.syncPlaybackAfterMissionChange(mission || normalized);
    } else if (typeof window.resetInspectionPlayback === "function") {
      window.resetInspectionPlayback();
    }
  } catch (err) {
    setTaskControlPhase("replan_failed");
    setStatus(`重规划错误: ${err.message}`, "err");
    console.error(err);
  } finally {
    replanRequestInFlight = false;
    if (replanButton) {
      replanButton.disabled = false;
      replanButton.textContent = previousButtonText;
    }
  }
}


async function runPlanning() {
  const body = buildCurrentPlanRequest();
  const pipeline = body.pipeline;

  $("runBtn").disabled = true;
  purgeDashboardMapPlots();
  setTaskControlPhase("planning");
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

    const normalized = normalizeMissionResult(data);
    onMissionLoaded(normalized, { kind: "plan" });
    if (pipeline === "image") {
      state.imageMissionAvailable = true;
      const st = await checkImageMission();
      state.imageMissionAvailable = st.available;
      const displayPath = resolveCleanMapImagePath(normalized);
      if (displayPath) await loadMapConfig(displayPath);
    }

    const mission = getCurrentMission();
    renderStats(mission?.statistics);
    renderVisitOrder(mission?.visit_order);
    renderMission(mission || normalized);
    renderMeta(mission?.metadata || data.metadata || {});
    updateDownloadLinks(data.output_files || {}, pipeline);

    const gen = data.metadata?.image_generation;
    const msg = gen?.generated
      ? "已自动生成并加载图像主线"
      : pipeline === "image"
        ? "已加载 result/latest/mission_output.json"
        : `完成 · ${$("plannerSelect")?.selectedOptions?.[0]?.textContent || body.planner}`;
    setTaskControlPhase("mission_ready");
    setStatus(
      `${msg} · ${data.statistics?.num_segments ?? 0} 段`,
      "ok"
    );
  } catch (err) {
    setTaskControlPhase("idle");
    setStatus(`错误: ${err.message}`, "err");
    console.error(err);
  } finally {
    $("runBtn").disabled = false;
  }
}

async function forceRegenerateImage() {
  $("forceRegenBtn").disabled = true;
  setTaskControlPhase("planning");
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
    setTaskControlPhase("idle");
    setStatus(`重新生成失败: ${err.message}`, "err");
  } finally {
    $("forceRegenBtn").disabled = false;
  }
}

document.addEventListener("DOMContentLoaded", async () => {
  try {
    await Promise.all([loadDatasets(), loadImageDatasetProfiles()]);
    await loadMapConfig(getDisplayMapPath());
    const st = await checkImageMission();
    state.imageMissionAvailable = st.available;
    applyLayerDefaultsForPipeline();
    applyPipelineUi();
    if (isImagePipeline()) renderInitialMapPreview("mapPlot");
    renderSystemStatus();
    if (window.MissionStore?.subscribePhase) {
      MissionStore.subscribePhase(() => renderSystemStatus());
    }
    setStatus("就绪 · 点击「生成任务」开始", "");
    setTaskControlPhase("idle");
  } catch (e) {
    setTaskControlPhase("idle");
    setStatus(`初始化失败: ${e.message}`, "err");
  }

  $("pipelineSelect").addEventListener("change", () => {
    applyPipelineUi();
    if (window.MissionStore) {
      MissionStore.clear();
    } else {
      state.lastResult = null;
    }
    purgeDashboardMapPlots();
    state.plotReady.mapPlot = false;
    if (typeof window.resetInspectionPlayback === "function") window.resetInspectionPlayback();
    if (isImagePipeline()) {
      loadMapConfig(getDisplayMapPath()).then(() => renderInitialMapPreview("mapPlot"));
    }
    setStatus("就绪", "");
    setTaskControlPhase("idle");
  });

  $("inspectionPointSourceSelect")?.addEventListener("change", async () => {
    applyPipelineUi();
    updateSpacingControlsVisibility();
    applyPipelineUi();
    await loadMapConfig(getDisplayMapPath());
    if (getCurrentMission()) refreshMapView();
    else if (isImagePipeline()) renderInitialMapPreview("mapPlot");
  });

  $("runBtn").addEventListener("click", runPlanning);
  $("runReplanBtn")?.addEventListener("click", runStartEndReplan);
  $("forceRegenBtn").addEventListener("click", forceRegenerateImage);
  $("fullscreenBtn").addEventListener("click", openFullscreen);
  $("closeFullscreenBtn").addEventListener("click", closeFullscreen);
  $("modalBackdrop")?.addEventListener("click", closeFullscreen);

  ["toggleBaseMap", "toggleInspect", "toggleConnect", "togglePointsLayer"].forEach((id) => {
    $(id)?.addEventListener("change", () => refreshMapView());
  });
  $("fullscreenBtnHeader")?.addEventListener("click", openFullscreen);
  $("toolsFullscreenBtn")?.addEventListener("click", openFullscreen);
  $("pointModeSelect")?.addEventListener("change", () => refreshMapView());

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
        if (typeof syncHtmlBasemapImages === "function") {
          syncHtmlBasemapImages(
            typeof getCurrentMission === "function" ? getCurrentMission() : null
          );
        }
      });
    }).observe(plotStageEl);
  }
});
