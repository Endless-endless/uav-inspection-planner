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
  showWeatherLayer: false,
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
    enabled: false,
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
  imageDatasetRegistry: null,
  imageDatasetProfile: null,
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
  const res = await fetch("/api/datasets");
  const data = await res.json();
  state.datasets = data.datasets || [];
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
  const dwEl = $("dynamicWeatherToggle");
  if (dwEl?.disabled) {
    state.dynamicWeather.enabled = false;
  } else {
    state.dynamicWeather.enabled = dwEl?.checked === true;
  }
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

function syncDynamicWeatherToggleFromState() {
  const el = $("dynamicWeatherToggle");
  if (!el) return;
  if (!state.weatherAware) {
    state.dynamicWeather.enabled = false;
  }
  el.disabled = !state.weatherAware;
  el.checked = state.dynamicWeather.enabled;
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
  syncDynamicWeatherToggleFromState();
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
  return {
    ...base,
    paper_bgcolor: transparent ? "rgba(0,0,0,0)" : base.paper_bgcolor || "#1a2332",
    plot_bgcolor: transparent ? "rgba(0,0,0,0)" : base.plot_bgcolor || "rgba(255,255,255,0.06)",
    images: [],
    xaxis: {
      ...(base.xaxis || {}),
      range: [0, width],
      autorange: false,
    },
    yaxis: {
      ...(base.yaxis || {}),
      range: [height, 0],
      autorange: false,
      scaleanchor: "x",
      scaleratio: 1,
      constrain: "domain",
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
  if (out.metadata.weather_mode != null) {
    out.statistics.weather_mode = out.metadata.weather_mode;
  }

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
  const draw = () => {
    bindMapClick(targetId);
    syncHtmlBasemapImages(getCurrentMission() || buildIdleImageMapPayload());
  };
  if (state.plotReady[targetId] && el.data) {
    Plotly.react(targetId, plot.traces, layout, plot.config).then(draw);
  } else {
    Plotly.newPlot(targetId, plot.traces, layout, plot.config).then(() => {
      state.plotReady[targetId] = true;
      draw();
    });
  }
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
  const weatherOn = state.weatherAware === true;

  let label = "NORMAL";
  let mod = "normal";
  if (phase === "replanning") {
    label = "REPLANNING";
    mod = "replan";
  } else if (flash) {
    label = "REROUTING";
    mod = "reroute";
  } else if (weatherOn && (phase === "adaptive_warning" || risk > th)) {
    label = "WEATHER ALERT";
    mod = "alert";
  } else if (weatherOn && risk > th * 0.55) {
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
  const weatherOn = state.weatherAware === true;
  const rk = weatherOn ? formatRiskTier(state.dynamicWeather.predictedRisk) : "未启用";
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
  const weatherOn = state.weatherAware === true;

  const primary = $("statCardsPrimary");
  const secondary = $("statCardsSecondary");
  const itemsPrimary = [
    ["total_length", "总长度", s.total_length, " px"],
    ["num_inspection_points", "巡检点数", s.num_inspection_points, ""],
    ["phase", "任务状态", phaseText, ""],
    ["predicted_risk", "天气风险", weatherOn ? riskVal.toFixed(2) : "—", ""],
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
    [
      "risky_distance",
      "高风险穿越",
      weatherOn ? (s.risky_distance ?? "—") : "—",
      weatherOn && s.risky_distance != null ? " px" : "",
    ],
    ["pred_samples", "预测采样点", weatherOn ? (state.dynamicWeather.predictedAffectedSegments ?? 0) : "—", ""],
    ["weather_penalty", "天气惩罚", weatherOn ? wpStr : "—", ""],
    ["reroute", "预测重规划", weatherOn ? (state.dynamicWeather.predictiveReplanCount ?? 0) : "—", weatherOn ? " 次" : ""],
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
  return getFixedSatelliteLayoutImages(result);
}

function buildMissionLayout(result, options = {}) {
  const { fullscreen = false } = options;
  const mission = result || buildIdleImageMapPayload();
  const { width, height } = getPipelineBackgroundSpec(mission);
  const transparent = isImagePipeline() && usesImagePixelCoords(mission);
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
    uirevision: "mission-map-v3",
    autosize: true,
    xaxis: {
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
      showgrid: true,
      gridcolor: "rgba(255,255,255,0.10)",
      tickfont: { size: 9, color: "#5a7a94" },
      zeroline: false,
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
  if (state.plotReady[plotId] && el.data) {
    return Plotly.react(plotId, traces, layout, config);
  }
  return Plotly.newPlot(plotId, traces, layout, config).then(() => {
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
    traces,
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

function onMissionLoaded(result, options = {}) {
  const normalized = normalizeMissionResult(result || {});
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

  if (state.plotReady[targetId] && el.data) {
    Plotly.react(targetId, plot.traces, layout, plot.config).then(afterDraw);
  } else {
    Plotly.newPlot(targetId, plot.traces, layout, plot.config).then(() => {
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
    await Promise.all([loadDatasets(), loadImageDatasetProfiles()]);
    await loadMapConfig(getDisplayMapPath());
    const st = await checkImageMission();
    state.imageMissionAvailable = st.available;
    applyLayerDefaultsForPipeline();
    applyPipelineUi();
    if (isImagePipeline()) renderInitialMapPreview("mapPlot");
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
    if (isImagePipeline()) {
      loadMapConfig(getDisplayMapPath()).then(() => renderInitialMapPreview("mapPlot"));
    }
    setStatus("就绪", "");
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
    if (!state.weatherAware) {
      state.dynamicWeather.enabled = false;
      state.showWeatherLayer = false;
      if ($("toggleWeatherLayer")) $("toggleWeatherLayer").checked = false;
      stopDynamicWeatherLoop();
      if (typeof setPredictiveWarningHud === "function") setPredictiveWarningHud(null);
      if (typeof setAdaptiveStatus === "function") setAdaptiveStatus("天气监测已关闭");
    } else if (state.dynamicWeather.enabled) {
      state.dynamicWeather.lastTick = performance.now();
      startDynamicWeatherLoop();
    }
    syncStateToUi();
    if (state.lastResult) renderMeta(state.lastResult.metadata || {});
    renderSystemStatus();
    refreshMapView();
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
    const dw = $("dynamicWeatherToggle");
    if (dw?.disabled) {
      if (dw) dw.checked = false;
      state.dynamicWeather.enabled = false;
      return;
    }
    readUiToState();
    if (state.dynamicWeather.enabled) {
      if (!state.weatherAware) {
        state.dynamicWeather.enabled = false;
        syncDynamicWeatherToggleFromState();
        return;
      }
      state.dynamicWeather.lastTick = performance.now();
      startDynamicWeatherLoop();
      pushAdaptiveEvent("动态天气模拟已开启");
      if (typeof setAdaptiveStatus === "function") setAdaptiveStatus("天气监测");
      if (typeof updateWeatherDynamicSummary === "function") updateWeatherDynamicSummary();
      if (typeof updateWeatherPlotTraces === "function") updateWeatherPlotTraces();
      if (typeof updatePredictiveMetrics === "function" && typeof predictFutureWeatherRisk === "function") {
        updatePredictiveMetrics(predictFutureWeatherRisk());
      }
    } else {
      stopDynamicWeatherLoop();
      if (typeof setAdaptiveStatus === "function") setAdaptiveStatus("天气监测空闲");
      pushAdaptiveEvent("动态天气模拟已暂停");
      if (typeof updateWeatherDynamicSummary === "function") updateWeatherDynamicSummary();
      if (typeof updateWeatherPlotTraces === "function") updateWeatherPlotTraces();
    }
    renderSystemStatus();
  });
  $("adaptiveRiskThresholdInput")?.addEventListener("change", () => {
    readUiToState();
    pushAdaptiveEvent(`风险阈值更新为 ${state.dynamicWeather.riskThreshold.toFixed(2)}`);
  });
  $("predictionWindowSelect")?.addEventListener("change", () => {
    readUiToState();
    pushAdaptiveEvent(`预测窗口更新为 ${state.dynamicWeather.predictionWindow}s`);
    if (state.weatherAware) updatePredictiveMetrics(predictFutureWeatherRisk());
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
        if (typeof syncHtmlBasemapImages === "function") {
          syncHtmlBasemapImages(
            typeof getCurrentMission === "function" ? getCurrentMission() : null
          );
        }
      });
    }).observe(plotStageEl);
  }
});
