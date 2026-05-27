/**
 * Weather UI — weather zones, dynamic weather, adaptive replan
 */
function getWeatherZones(result) {
  const zones = state.dynamicWeather.zones?.length
    ? state.dynamicWeather.zones
    : result?.weather_zones || result?.metadata?.weather_zones || [];
  return Array.isArray(zones) ? zones : [];
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
  const flash = state.dynamicWeather.adaptiveFlash;
  if (!flash?.path || flash.path.length < 2) return [];
  const elapsed = Math.max(0, performance.now() - flash.startedAt);
  const phase = Math.min(1, elapsed / flash.durationMs);
  const opacity = phase < 0.65 ? 0.95 : Math.max(0, 0.95 * (1 - (phase - 0.65) / 0.35));
  return [{
    x: flash.path.map((p) => p[0]),
    y: flash.path.map((p) => p[1]),
    mode: "lines",
    name: "自适应重规划路径",
    line: {
      color: `rgba(74, 222, 128, ${opacity.toFixed(3)})`,
      width: 5 + (1 - phase) * 2,
      dash: "solid",
    },
    hovertemplate: "临时绕行路径<extra></extra>",
    showlegend: false,
    legendgroup: "adaptive",
  }];
}

function polylineLength(points) {
  if (!points || points.length < 2) return 0;
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    total += Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]);
  }
  return total;
}

function applyAdaptivePathToMission(adaptivePath) {
  const uav = getPlaybackUavPoint();
  if (window.MissionStore) {
    return MissionStore.applyAdaptiveConnectReroute(adaptivePath, { uav });
  }
  const mission = getCurrentMission();
  if (!mission?.segments?.length || !adaptivePath?.length) return false;
  const segments = mission.segments.map((seg) => ({ ...seg, geometry_2d: [...(seg.geometry_2d || [])] }));
  let targetIdx = -1;
  let bestDist = Infinity;
  segments.forEach((seg, idx) => {
    if (seg.type !== "connect") return;
    const geom = seg.geometry_2d || [];
    if (geom.length < 2) return;
    const anchor = uav || geom[0];
    const d = Math.hypot(geom[0][0] - anchor[0], geom[0][1] - anchor[1]);
    if (d < bestDist) {
      bestDist = d;
      targetIdx = idx;
    }
  });
  if (targetIdx < 0) return false;
  const newGeom = adaptivePath.map((p) => [Number(p[0]), Number(p[1])]);
  segments[targetIdx] = {
    ...segments[targetIdx],
    geometry_2d: newGeom,
    length: Math.round(polylineLength(newGeom)),
    role: "adaptive_reroute",
  };
  state.lastResult = { ...mission, segments };
  return true;
}

function clearAdaptiveFlashTimer() {
  if (state.dynamicWeather.adaptiveFlashTimer) {
    clearInterval(state.dynamicWeather.adaptiveFlashTimer);
    state.dynamicWeather.adaptiveFlashTimer = null;
  }
}

function finishAdaptiveFlash() {
  const flash = state.dynamicWeather.adaptiveFlash;
  clearAdaptiveFlashTimer();
  state.dynamicWeather.adaptiveFlash = null;
  state.dynamicWeather.adaptivePath = null;
  if (window.LayerManager) {
    LayerManager.clearLayer(window.LayerIds?.T1_ADAPTIVE_FLASH);
  }
  if (flash?.path?.length) {
    applyAdaptivePathToMission(flash.path);
    syncPlaybackAfterMissionChange({ restart: AppPhaseManager?.getPhase?.() === AppPhase?.PLAYING });
  }
  if (AppPhaseManager) AppPhaseManager.endAdaptiveWarning();
  updateAdaptivePlotTraces();
  refreshMapView();
}

function startAdaptiveFlash(path) {
  if (!path || path.length < 2) return;
  clearAdaptiveFlashTimer();
  state.dynamicWeather.adaptivePath = null;
  state.dynamicWeather.adaptiveFlash = {
    path,
    startedAt: performance.now(),
    durationMs: 1800,
  };
  setAdaptiveStatus("路径更新完成", "reroute");
  const traces = buildAdaptiveReplanTraces();
  if (window.LayerManager && window.LayerIds) {
    LayerManager.setLayer(LayerIds.T1_ADAPTIVE_FLASH, traces, {
      durationMs: state.dynamicWeather.adaptiveFlash.durationMs,
      onExpire: () => finishAdaptiveFlash(),
    });
  }
  updateAdaptivePlotTraces();
  state.dynamicWeather.adaptiveFlashTimer = window.setInterval(() => {
    const flash = state.dynamicWeather.adaptiveFlash;
    if (!flash) {
      clearAdaptiveFlashTimer();
      return;
    }
    if (performance.now() - flash.startedAt >= flash.durationMs) {
      finishAdaptiveFlash();
      return;
    }
    if (window.LayerManager && window.LayerIds) {
      LayerManager.setLayer(LayerIds.T1_ADAPTIVE_FLASH, buildAdaptiveReplanTraces());
    }
    updateAdaptivePlotTraces();
  }, 60);
}

function setPredictiveWarningHud(text, durationMs = 6000) {
  const ids = ["predictiveWarningHud", "predictiveWarningHudFs"];
  if (!text) {
    ids.forEach((id) => {
      const el = $(id);
      if (!el) return;
      el.textContent = "";
      el.classList.add("hidden");
    });
    state.dynamicWeather.predictiveHud = null;
    return;
  }
  ids.forEach((id) => {
    const el = $(id);
    if (!el) return;
    el.textContent = text;
    el.classList.remove("hidden");
  });
  state.dynamicWeather.predictiveHud = {
    text,
    activeUntil: performance.now() + durationMs,
  };
}

function tickPredictiveWarningHud() {
  const hud = state.dynamicWeather.predictiveHud;
  if (!hud) return;
  if (performance.now() >= hud.activeUntil) {
    setPredictiveWarningHud(null);
  }
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
  state.dynamicWeather.adaptiveFlash = null;
  clearAdaptiveFlashTimer();
  state.dynamicWeather.predictiveHud = null;
  setPredictiveWarningHud(null);
  state.dynamicWeather.predictiveReplanCount = 0;
  state.dynamicWeather.predictedRisk = 0;
  state.dynamicWeather.predictedAffectedSegments = 0;
  state.dynamicWeather.timeToRisk = null;
  state.dynamicWeather.lastReplanAt = 0;
  state.dynamicWeather.lastWarningAt = 0;
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

function getPlaybackSpeed() {
  const visual = typeof window.getPlaybackVisualState === "function"
    ? window.getPlaybackVisualState()
    : null;
  if (visual?.playbackSpeed > 0) return Number(visual.playbackSpeed);
  return Math.max(0.1, parseFloat($("playbackSpeedSelect")?.value || "1") || 1);
}

const PLAYBACK_BASE_PX_PER_SEC = 500;

function buildPathFromUav(uav, path, startIdx) {
  const sub = [[uav[0], uav[1]]];
  for (let i = startIdx + 1; i < path.length; i += 1) {
    sub.push([path[i][0], path[i][1]]);
  }
  return sub;
}

function pointAlongPolyline(polyline, distance) {
  if (!polyline?.length) return null;
  if (distance <= 0) return [polyline[0][0], polyline[0][1]];
  let remain = distance;
  for (let i = 1; i < polyline.length; i += 1) {
    const ax = polyline[i - 1][0];
    const ay = polyline[i - 1][1];
    const bx = polyline[i][0];
    const by = polyline[i][1];
    const segLen = Math.hypot(bx - ax, by - ay);
    if (segLen <= 1e-6) continue;
    if (remain <= segLen) {
      const t = remain / segLen;
      return [ax + t * (bx - ax), ay + t * (by - ay)];
    }
    remain -= segLen;
  }
  const last = polyline[polyline.length - 1];
  return [last[0], last[1]];
}

function forecastWeatherZones(zones, horizonSec, elapsed, bounds) {
  const horizon = Math.max(0, Number(horizonSec) || 0);
  if (horizon <= 1e-6) return cloneWeatherZones(zones);
  const xr = bounds?.x_range || [0, 1000];
  const yr = bounds?.y_range || [1000, 0];
  const xmin = Math.min(xr[0], xr[1]);
  const xmax = Math.max(xr[0], xr[1]);
  const ymin = Math.min(yr[0], yr[1]);
  const ymax = Math.max(yr[0], yr[1]);
  const steps = Math.max(1, Math.ceil(horizon / 2));
  const stepDt = horizon / steps;
  let out = cloneWeatherZones(zones);
  let curElapsed = Number(elapsed) || 0;
  for (let i = 0; i < steps; i += 1) {
    curElapsed += stepDt;
    out = cloneWeatherZones(out).map((z) => {
      if (!z.dynamic) return z;
      const lifetime = Number(z.lifetime || 0);
      if (lifetime > 0 && curElapsed > lifetime) {
        const phase = (curElapsed - lifetime) / Math.max(lifetime, 1);
        z.severity = Math.max(0, Math.min(1, Number(z.severity || 0) * Math.exp(-phase)));
        if (z.severity <= 0.03) return null;
      }
      const vel = z.velocity || [0, 0];
      z.center[0] += Number(vel[0] || 0) * stepDt;
      z.center[1] += Number(vel[1] || 0) * stepDt;
      z.radius = Math.max(10, Number(z.radius || 0) + Number(z.expand_rate || 0) * stepDt);
      z.severity = Math.max(0, Math.min(1, Number(z.severity || 0) + Number(z.severity_rate || 0) * stepDt));
      if (z.center[0] < xmin || z.center[0] > xmax) {
        z.velocity[0] = -Number(z.velocity[0] || 0);
        z.center[0] = Math.max(xmin, Math.min(xmax, z.center[0]));
      }
      if (z.center[1] < ymin || z.center[1] > ymax) {
        z.velocity[1] = -Number(z.velocity[1] || 0);
        z.center[1] = Math.max(ymin, Math.min(ymax, z.center[1]));
      }
      return z;
    }).filter(Boolean);
  }
  return out;
}

function updatePredictiveMetrics(prediction) {
  if (!prediction) {
    state.dynamicWeather.predictedRisk = 0;
    state.dynamicWeather.predictedAffectedSegments = 0;
    state.dynamicWeather.timeToRisk = null;
  } else {
    state.dynamicWeather.predictedRisk = prediction.predicted_risk;
    state.dynamicWeather.predictedAffectedSegments = prediction.predicted_affected_segments;
    state.dynamicWeather.timeToRisk = prediction.time_to_risk;
  }
  if (state.lastResult?.statistics) {
    renderStats(state.lastResult.statistics);
  }
}

function predictFutureWeatherRisk() {
  const uav = getPlaybackUavPoint();
  const path = flattenMissionPathPoints(state.lastResult);
  const windowSec = state.dynamicWeather.predictionWindow || 10;
  const threshold = state.dynamicWeather.riskThreshold;
  if (!uav || path.length < 2 || !state.dynamicWeather.zones.length) return null;

  const start = nearestPathIndex(path, uav);
  const futurePath = buildPathFromUav(uav, path, start);
  const speed = getPlaybackSpeed();
  const bounds = state.lastResult?.bounds || { x_range: [0, 1000], y_range: [1000, 0] };
  const elapsed = state.dynamicWeather.elapsed || 0;
  const sampleStep = 0.5;

  let maxRisk = 0;
  let timeToRisk = null;
  let hitZone = null;
  let hitPoint = null;
  let affectedSamples = 0;

  for (let t = 0; t <= windowSec + 1e-6; t += sampleStep) {
    const dist = PLAYBACK_BASE_PX_PER_SEC * speed * t;
    const pt = pointAlongPolyline(futurePath, dist);
    if (!pt) break;
    const forecastZones = forecastWeatherZones(state.dynamicWeather.zones, t, elapsed, bounds);
    let stepRisk = 0;
    let stepZone = null;
    for (const z of forecastZones) {
      const risk = pointZoneRisk(pt, z);
      if (risk > stepRisk) {
        stepRisk = risk;
        stepZone = z;
      }
    }
    if (stepRisk > maxRisk) {
      maxRisk = stepRisk;
      hitZone = stepZone;
      hitPoint = pt;
    }
    if (stepRisk > threshold) {
      affectedSamples += 1;
      if (timeToRisk == null) timeToRisk = t;
    }
  }

  return {
    predicted_risk: maxRisk,
    prediction_window: windowSec,
    predicted_affected_segments: affectedSamples,
    time_to_risk: timeToRisk,
    hitZone,
    hitPoint,
    uav,
    path,
    startIndex: start,
    zoneLabel: hitZone ? weatherColor(hitZone.type, hitZone.severity).label : "天气区",
    isPredictive: timeToRisk != null && timeToRisk > 0.25,
  };
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

function setAdaptiveStatus(status, tone = "normal") {
  state.dynamicWeather.status = adaptiveStatusText(status);
  const el = $("adaptiveStatus");
  if (el) {
    el.textContent = state.dynamicWeather.status;
    el.classList.remove("is-warning", "is-reroute");
    if (tone === "warning") el.classList.add("is-warning");
    if (tone === "reroute") el.classList.add("is-reroute");
  }
  if (typeof window.renderSystemStatus === "function") window.renderSystemStatus();
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

function canTriggerAdaptiveReplan() {
  const cooldownMs = (state.dynamicWeather.replanCooldownSec || 10) * 1000;
  return (performance.now() - (state.dynamicWeather.lastReplanAt || 0)) >= cooldownMs;
}

function triggerAdaptiveReplan(riskInfo, { predictive = false } = {}) {
  if (AppPhaseManager && !AppPhaseManager.canRunAdaptive()) return false;
  const zone = riskInfo.hitZone;
  const uav = riskInfo.uav;
  const hitPoint = riskInfo.hitPoint;
  if (!zone || !uav || !hitPoint) return false;

  if (AppPhaseManager) AppPhaseManager.beginAdaptiveWarning();

  const label = riskInfo.zoneLabel || weatherColor(zone.type, zone.severity).label;
  if (predictive) {
    setAdaptiveStatus("预测风险预警", "warning");
    pushAdaptiveEvent("提前触发自适应重规划");
    state.dynamicWeather.predictiveReplanCount += 1;
  } else {
    pushAdaptiveEvent(`检测到${label}进入前方航迹`);
    pushAdaptiveEvent(`风险 ${(riskInfo.maxRisk ?? riskInfo.predicted_risk ?? 0).toFixed(2)} 超过阈值`);
    pushAdaptiveEvent("触发自动重规划");
  }

  setAdaptiveStatus("自动重规划", "reroute");
  const path = buildAdaptivePathAroundZone(uav, hitPoint, zone);
  state.dynamicWeather.replanTriggered = true;
  state.dynamicWeather.lastReplanAt = performance.now();
  setPredictiveWarningHud(null);
  pushAdaptiveEvent("新路径已生成");
  startAdaptiveFlash(path);
  return true;
}

function maybeTriggerAdaptiveReplan() {
  if (AppPhaseManager && !AppPhaseManager.canRunAdaptive()) return;
  const visual = typeof window.getPlaybackVisualState === "function" ? window.getPlaybackVisualState() : null;
  if (!visual || visual.status !== "playing") return;

  const threshold = state.dynamicWeather.riskThreshold;
  const prediction = predictFutureWeatherRisk();
  updatePredictiveMetrics(prediction);

  if (prediction?.predicted_risk > threshold) {
    const label = prediction.zoneLabel || "风险区";
    const eta = Math.max(1, Math.round(prediction.time_to_risk ?? prediction.prediction_window ?? 10));
    setPredictiveWarningHud(`⚠ 预计 ${eta}s 后进入${label}`);
    setAdaptiveStatus("预测风险分析", "warning");
    const now = performance.now();
    const warnCooldownMs = 5000;
    if ((now - (state.dynamicWeather.lastWarningAt || 0)) >= warnCooldownMs) {
      pushAdaptiveEvent(`预测到前方${label}覆盖航迹`);
      pushAdaptiveEvent(`预计 ${eta}s 后进入风险区`);
      state.dynamicWeather.lastWarningAt = now;
    }

    if (
      prediction.isPredictive
      && state.dynamicWeather.autoPredictiveReplan
      && canTriggerAdaptiveReplan()
    ) {
      triggerAdaptiveReplan(prediction, { predictive: true });
      return;
    }
    return;
  }

  setPredictiveWarningHud(null);
  setAdaptiveStatus("风险分析");
  const risk = analyzeFutureWeatherRisk();
  if (!risk?.hitZone) {
    setAdaptiveStatus("正常巡航");
    return;
  }
  if (risk.maxRisk > threshold) {
    if (canTriggerAdaptiveReplan()) {
      triggerAdaptiveReplan(risk, { predictive: false });
    }
  } else {
    setAdaptiveStatus("天气监测");
  }
}

function startDynamicWeatherLoop() {
  stopDynamicWeatherLoop();
  state.dynamicWeather.timer = window.setInterval(() => {
    if (!getCurrentMission() || !state.dynamicWeather.enabled) return;
    if (AppPhaseManager && ![AppPhase.PLAYING, AppPhase.ADAPTIVE_WARNING, AppPhase.MISSION_READY].includes(AppPhaseManager.getPhase())) {
      return;
    }
    const now = performance.now();
    const dt = Math.min(2.0, Math.max(0.2, (now - state.dynamicWeather.lastTick) / 1000));
    state.dynamicWeather.lastTick = now;
    updateDynamicWeatherZones(dt);
    maybeTriggerAdaptiveReplan();
    tickPredictiveWarningHud();
    updateWeatherPlotTraces();
  }, 900);
}

function stopDynamicWeatherLoop() {
  if (state.dynamicWeather.timer) {
    clearInterval(state.dynamicWeather.timer);
    state.dynamicWeather.timer = null;
  }
  clearAdaptiveFlashTimer();
}
function replaceTraceSet(plotId, predicate, traces) {
  const el = document.getElementById(plotId);
  if (!el?.data || !state.plotReady[plotId]) return;
  const keep = el.data.filter((t) => !predicate(t));
  Plotly.react(plotId, keep.concat(traces), el.layout, { responsive: plotId !== "mapPlotFs", displayModeBar: true, scrollZoom: true });
}

function updateWeatherPlotTraces() {
  const traces = buildWeatherZoneTraces(getCurrentMission());
  if (window.LayerManager && window.LayerIds) {
    LayerManager.setLayer(LayerIds.L3_WEATHER, traces);
  }
  ["mapPlot", "mapPlotFs"].forEach((plotId) => {
    if (window.LayerManager && window.LayerIds) {
      LayerManager.replaceLayerTraces(plotId, LayerIds.L3_WEATHER, traces, state);
    } else {
      replaceTraceSet(plotId, (t) => String(t.name || "").startsWith("天气区 ") || t.name === "天气标签", traces);
    }
  });
}

function updateAdaptivePlotTraces() {
  const traces = buildAdaptiveReplanTraces();
  if (window.LayerManager && window.LayerIds) {
    LayerManager.setLayer(LayerIds.T1_ADAPTIVE_FLASH, traces, {
      durationMs: state.dynamicWeather.adaptiveFlash?.durationMs || 1800,
    });
  }
  ["mapPlot", "mapPlotFs"].forEach((plotId) => {
    if (window.LayerManager && window.LayerIds) {
      LayerManager.replaceLayerTraces(plotId, LayerIds.T1_ADAPTIVE_FLASH, traces, state);
    } else {
      replaceTraceSet(plotId, (t) => t.name === "自适应重规划路径", traces);
    }
  });
}
