/**
 * Experiment UI — A/B weather experiment overlay
 */
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
function updateExperimentPlotTraces() {
  const traces = buildExperimentOverlayTraces();
  if (window.LayerManager && window.LayerIds) {
    LayerManager.setLayer(LayerIds.T3_AB_EXPERIMENT, traces);
  }
  ["mapPlot", "mapPlotFs"].forEach((plotId) => {
    if (window.LayerManager && window.LayerIds) {
      LayerManager.replaceLayerTraces(plotId, LayerIds.T3_AB_EXPERIMENT, traces, state);
    }
  });
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
          ...buildReplanPayload(check),
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

    if (window.MissionStore) {
      MissionStore.loadFromDashboard(runB, { kind: "experiment" });
    } else {
      state.lastResult = runB;
    }
    updateExperimentPlotTraces();
    const mission = getCurrentMission();
    renderStats(mission?.statistics);
    renderVisitOrder(mission?.visit_order);
    renderMeta(mission?.metadata || {});
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
