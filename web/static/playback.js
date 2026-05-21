/**
 * Dashboard 巡检过程模拟播放
 */
(function () {
  const $ = (id) => document.getElementById(id);

  const MOVE_STEP_PX = 10;
  const MOVE_FRAME_MS = 20;
  const POINT_TRIGGER_DIST = 8;
  const CONTINUITY_EPS = 5;

  const PLAYBACK = {
    status: "idle",
    timeline: [],
    index: 0,
    timer: null,
    speed: 1,
    dwellMs: 1500,
    visitedIds: new Set(),
    visitedCoords: new Map(),
    currentPointId: null,
    uavPos: null,
    currentPoint: null,
    totalInspectCount: 0,
    usingPlaceholder: false,
    debug: null,
    imageCache: new Map(),
  };

  function dist(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  function interpolatePolyline(points, stepPx) {
    if (!points || points.length < 2) {
      return points?.length ? [{ x: points[0][0], y: points[0][1] }] : [];
    }
    const out = [{ x: points[0][0], y: points[0][1] }];
    for (let i = 1; i < points.length; i += 1) {
      const a = { x: points[i - 1][0], y: points[i - 1][1] };
      const b = { x: points[i][0], y: points[i][1] };
      const segLen = dist(a, b);
      if (segLen < 1e-6) continue;
      const n = Math.max(1, Math.ceil(segLen / stepPx));
      for (let k = 1; k <= n; k += 1) {
        const t = k / n;
        out.push({
          x: a.x + t * (b.x - a.x),
          y: a.y + t * (b.y - a.y),
        });
      }
    }
    return out;
  }

  function toPoint(v) {
    if (!v) return null;
    if (Array.isArray(v) && v.length >= 2) {
      return { x: Number(v[0]), y: Number(v[1]) };
    }
    if (typeof v === "object" && v.x != null && v.y != null) {
      return { x: Number(v.x), y: Number(v.y) };
    }
    return null;
  }

  function getMissionStart(result) {
    const meta = result?.metadata || {};
    return (
      toPoint(meta.start) ||
      toPoint(meta.start_point) ||
      toPoint(result?.markers?.start)
    );
  }

  function getMissionEnd(result) {
    const meta = result?.metadata || {};
    return (
      toPoint(meta.end) ||
      toPoint(meta.end_point) ||
      toPoint(result?.markers?.end)
    );
  }

  function getSegmentPolyline(seg) {
    const candidates = [
      seg?.geometry_2d,
      seg?.polyline,
      seg?.path,
      seg?.coordinates,
      seg?.points,
    ];
    for (const c of candidates) {
      if (!Array.isArray(c)) continue;
      const pts = c
        .map((p) => (Array.isArray(p) ? { x: Number(p[0]), y: Number(p[1]) } : null))
        .filter((p) => p && Number.isFinite(p.x) && Number.isFinite(p.y));
      if (pts.length >= 2) return pts;
    }
    return [];
  }

  function reverseIfNeeded(points, currentPos) {
    if (!currentPos || points.length < 2) return { points, reversed: false };
    const d1 = dist(currentPos, points[0]);
    const d2 = dist(currentPos, points[points.length - 1]);
    if (d2 < d1) {
      return { points: [...points].reverse(), reversed: true };
    }
    return { points, reversed: false };
  }

  function buildMoveFrames(points, stepPx, segmentId, segmentType) {
    const raw = points.map((p) => [p.x, p.y]);
    const dense = interpolatePolyline(raw, stepPx);
    const out = [];
    let d = 0;
    for (let i = 0; i < dense.length; i += 1) {
      if (i > 0) d += dist(dense[i - 1], dense[i]);
      out.push({
        type: "move",
        x: dense[i].x,
        y: dense[i].y,
        d,
        segment_id: segmentId,
        segment_type: segmentType,
      });
    }
    return out;
  }

  function cumulativeDistances(line) {
    const cum = [0];
    for (let i = 1; i < line.length; i += 1) {
      cum.push(cum[i - 1] + dist(line[i - 1], line[i]));
    }
    return cum;
  }

  function projectOnPolyline(pt, line) {
    let bestD = 0;
    let bestDist = Infinity;
    const cum = cumulativeDistances(line);
    for (let i = 0; i < line.length - 1; i += 1) {
      const a = line[i];
      const b = line[i + 1];
      const abx = b.x - a.x;
      const aby = b.y - a.y;
      const len2 = abx * abx + aby * aby;
      let t = 0;
      if (len2 > 1e-9) {
        t = Math.max(0, Math.min(1, ((pt.x - a.x) * abx + (pt.y - a.y) * aby) / len2));
      }
      const px = a.x + t * abx;
      const py = a.y + t * aby;
      const d = Math.hypot(pt.x - px, pt.y - py);
      if (d < bestDist) {
        bestDist = d;
        bestD = cum[i] + t * (cum[i + 1] - cum[i]);
      }
    }
    return bestD;
  }

  function slicePolylineByDistance(line, d0, d1) {
    const cum = cumulativeDistances(line);
    const total = cum[cum.length - 1];
    d0 = Math.max(0, Math.min(d0, total));
    d1 = Math.max(d0, Math.min(d1, total));
    const out = [];
    for (let i = 0; i < line.length; i += 1) {
      if (cum[i] >= d0 - 1e-6 && cum[i] <= d1 + 1e-6) {
        out.push({ x: line[i].x, y: line[i].y });
      }
    }
    if (out.length < 2) {
      const pick = (d) => {
        for (let i = 1; i < cum.length; i += 1) {
          if (cum[i] >= d) {
            const seg = cum[i] - cum[i - 1];
            const t = seg > 1e-9 ? (d - cum[i - 1]) / seg : 0;
            return {
              x: line[i - 1].x + t * (line[i].x - line[i - 1].x),
              y: line[i - 1].y + t * (line[i].y - line[i - 1].y),
            };
          }
        }
        return line[line.length - 1];
      };
      return [pick(d0), pick(d1)];
    }
    return out;
  }

  function pointsForSegment(allPoints, seg) {
    const sid = seg.segment_id;
    const eid = seg.edge_id;
    return allPoints.filter(
      (p) =>
        (sid && p.segment_id === sid) ||
        (eid && p.edge_id === eid && (!p.segment_id || p.segment_id === sid))
    );
  }

  function sortPointsAlongPolyline(points, line) {
    return [...points].sort(
      (a, b) => projectOnPolyline(a, line) - projectOnPolyline(b, line)
    );
  }

  window.buildInspectionPlaybackTimeline = function (result, dwellMs) {
    const timeline = [];
    const segments = result?.segments || [];
    const allPoints = result?.inspection_points || [];
    const start = getMissionStart(result);
    const end = getMissionEnd(result);
    let currentPos = start;
    let inspectSeq = 0;
    let reversedSegments = 0;
    let gapConnects = 0;

    if (start) {
      timeline.push({
        type: "move",
        x: start.x,
        y: start.y,
        segment_id: "start",
        segment_type: "start",
      });
    }

    segments.forEach((seg, idx) => {
      let points = getSegmentPolyline(seg);
      if (points.length < 2) return;

      const oriented = reverseIfNeeded(points, currentPos);
      points = oriented.points;
      if (oriented.reversed) reversedSegments += 1;

      if (currentPos && dist(currentPos, points[0]) > CONTINUITY_EPS) {
        const gap = buildMoveFrames(
          [currentPos, points[0]],
          MOVE_STEP_PX,
          `${seg.segment_id || `seg_${idx}`}_gap`,
          "connect_gap"
        );
        if (gap.length > 0) {
          timeline.push(...gap);
          gapConnects += 1;
        }
      }

      const segId = seg.segment_id || `seg_${idx}`;
      const segType = seg.type || "inspect";
      const frames = buildMoveFrames(points, MOVE_STEP_PX, segId, segType);
      if (!frames.length) return;

      if (segType === "inspect") {
        const segPoints = sortPointsAlongPolyline(
          pointsForSegment(allPoints, seg),
          points
        ).map((pt) => ({
          point: pt,
          proj: projectOnPolyline(pt, points),
          triggered: false,
        }));

        let pIdx = 0;
        frames.forEach((frame) => {
          timeline.push(frame);
          while (pIdx < segPoints.length) {
            const it = segPoints[pIdx];
            const dd = dist(frame, it.point);
            if (dd <= POINT_TRIGGER_DIST || frame.d >= it.proj) {
              inspectSeq += 1;
              timeline.push({
                type: "inspect",
                point: it.point,
                dwell_ms: dwellMs,
                inspect_index: inspectSeq,
                segment_id: segId,
              });
              it.triggered = true;
              pIdx += 1;
              continue;
            }
            break;
          }
        });
      } else {
        timeline.push(...frames);
      }

      currentPos = { x: frames[frames.length - 1].x, y: frames[frames.length - 1].y };
    });

    if (end) {
      const needEndConnect = !currentPos || dist(currentPos, end) > CONTINUITY_EPS;
      if (needEndConnect) {
        const from = currentPos || start || end;
        const endFrames = buildMoveFrames(
          [from, end],
          MOVE_STEP_PX,
          "to_end",
          "connect_end"
        );
        if (endFrames.length) {
          timeline.push(...endFrames);
          currentPos = { x: end.x, y: end.y };
        }
      } else if (currentPos) {
        timeline.push({
          type: "move",
          x: end.x,
          y: end.y,
          segment_id: "end",
          segment_type: "end",
        });
        currentPos = { x: end.x, y: end.y };
      }
    }

    const firstMove = timeline.find((e) => e.type === "move");
    const lastMove = [...timeline].reverse().find((e) => e.type === "move");
    const debug = {
      frame_count: timeline.length,
      first_point: firstMove ? [firstMove.x, firstMove.y] : null,
      last_point: lastMove ? [lastMove.x, lastMove.y] : null,
      inserted_gap_connects: gapConnects,
      reversed_segments: reversedSegments,
      inspect_events: inspectSeq,
    };
    PLAYBACK.debug = debug;
    window.__playbackDebug = debug;
    console.log("[playback timeline]", debug);

    return timeline;
  };

  function getDwellMs() {
    const sel = $("dwellTimeSelect");
    if (sel) return parseFloat(sel.value) * 1000;
    return PLAYBACK.dwellMs;
  }

  function getSpeed() {
    const sel = $("playbackSpeedSelect");
    if (sel) return parseFloat(sel.value) || 1;
    return PLAYBACK.speed;
  }

  function clearInspectCard() {
    ["", "Fs"].forEach((p) => {
      const card = $(`inspectInfoCard${p}`);
      if (!card) return;
      card.classList.add("hidden");
      const img = $(`inspectCardImg${p}`);
      if (img) img.src = "/static/inspection_placeholder.svg";
      const hint = $(`inspectCardImgHint${p}`);
      if (hint) hint.textContent = "";
      const uav = $(`inspectCardUavStatus${p}`);
      if (uav) uav.textContent = "待命";
      const time = $(`inspectCardTime${p}`);
      if (time) time.textContent = "—";
      const st = $(`inspectCardStatus${p}`);
      if (st) {
        st.textContent = "待命";
        st.className = "inspect-status idle";
      }
    });
  }

  function fillInspectCard(prefix, point, index, total) {
    const p = prefix || "";
    const card = $(`inspectInfoCard${p}`);
    if (!card) return;
    card.classList.remove("hidden");
    const set = (id, val) => {
      const el = $(`${id}${p}`);
      if (el) el.textContent = val;
    };
    set("inspectCardId", point.id || point.point_id || "—");
    set("inspectCardSegment", point.segment_id || point.edge_id || "—");
    set("inspectCardCoord", `${Math.round(point.x)}, ${Math.round(point.y)}`);
    set("inspectCardType", point.point_type || "sample");
    set("inspectCardPriority", point.priority || "normal");
    set("inspectCardProgress", `${index} / ${Math.max(total, 1)}`);
    set("inspectCardUavStatus", "正在巡检");
    set("inspectCardTime", new Date().toLocaleTimeString("zh-CN"));
    const img = $(`inspectCardImg${p}`);
    const imageUrl =
      point.image_url ||
      point.image_path ||
      point.image_placeholder ||
      "/static/inspection_placeholder.svg";
    const finalUrl = PLAYBACK.imageCache.get(imageUrl) || imageUrl;
    if (img) {
      img.onerror = () => {
        img.src = point.image_placeholder || "/static/inspection_placeholder.svg";
        PLAYBACK.usingPlaceholder = true;
      };
      img.src = finalUrl;
    }
    if (!PLAYBACK.imageCache.has(imageUrl)) {
      const preload = new Image();
      preload.onload = () => PLAYBACK.imageCache.set(imageUrl, imageUrl);
      preload.src = imageUrl;
    }
    const hint = $(`inspectCardImgHint${p}`);
    if (hint) {
      hint.textContent =
        point.image_available && point.image_url
          ? "巡检图片已加载"
          : "暂无巡检图片，显示占位图";
    }
    const st = $(`inspectCardStatus${p}`);
    if (st) {
      st.textContent = "正在拍照";
      st.className = "inspect-status shooting";
    }
  }

  function showInspectCard(point, index, total) {
    fillInspectCard("", point, index, total);
    fillInspectCard("Fs", point, index, total);
    const pid = point.id || point.point_id;
    setTimeout(() => {
      if (PLAYBACK.currentPointId === pid) {
        ["", "Fs"].forEach((pfx) => {
          const st = $(`inspectCardStatus${pfx}`);
          if (st) {
            st.textContent = "已完成 · 正常";
            st.className = "inspect-status done";
          }
        });
      }
    }, Math.min(800, getDwellMs() * 0.5));
  }

  function buildPlaybackTraces() {
    return [
      {
        name: "UAV Glow",
        x: [],
        y: [],
        mode: "markers",
        marker: {
          size: 26,
          color: "rgba(34, 211, 238, 0.23)",
          symbol: "circle",
          line: { width: 0, color: "rgba(0,0,0,0)" },
        },
        showlegend: false,
        hoverinfo: "skip",
      },
      {
        name: "UAV",
        x: [],
        y: [],
        mode: "markers",
        marker: {
          size: 16,
          color: "#7dd3fc",
          symbol: "triangle-up",
          line: { width: 2, color: "#0e7490" },
        },
        showlegend: false,
        hoverinfo: "skip",
      },
      {
        name: "Current Inspect",
        x: [],
        y: [],
        mode: "markers",
        marker: {
          size: 14,
          color: "#fbbf24",
          opacity: 1,
          line: { width: 3, color: "#fff" },
        },
        showlegend: false,
        hoverinfo: "skip",
      },
      {
        name: "Visited",
        x: [],
        y: [],
        mode: "markers",
        marker: {
          size: 7,
          color: "#22c55e",
          opacity: 0.7,
          line: { width: 1, color: "#14532d" },
        },
        showlegend: false,
        hoverinfo: "skip",
      },
    ];
  }

  window.appendPlaybackTraces = function (traces) {
    return traces.concat(buildPlaybackTraces());
  };

  window.registerPlaybackTraceIndices = function () {
    /* 改为按 name 动态查找，无需预注册 */
  };

  window.getPlaybackVisualState = function () {
    return {
      status: PLAYBACK.status,
      currentPointId: PLAYBACK.currentPointId,
      visitedIds: new Set(PLAYBACK.visitedIds),
    };
  };

  function getTraceIndices(plotId) {
    const el = document.getElementById(plotId);
    if (!el?.data?.length) return null;
    const idx = (name) => el.data.findIndex((t) => t.name === name);
    const uav = idx("UAV");
    if (uav < 0) return null;
    return {
      glow: idx("UAV Glow"),
      uav,
      current: idx("Current Inspect"),
      visited: idx("Visited"),
    };
  }

  function restylePlayback(plotId) {
    const el = document.getElementById(plotId);
    if (!el?.data) return;
    const idx = getTraceIndices(plotId);
    if (!idx || idx.uav < 0) return;

    try {
      if (idx.glow >= 0) {
        Plotly.restyle(
          plotId,
          PLAYBACK.uavPos
            ? {
                x: [[PLAYBACK.uavPos.x]],
                y: [[PLAYBACK.uavPos.y]],
              }
            : { x: [[]], y: [[]] },
          [idx.glow]
        );
      }

      Plotly.restyle(
        plotId,
        PLAYBACK.uavPos
          ? {
              x: [[PLAYBACK.uavPos.x]],
              y: [[PLAYBACK.uavPos.y]],
            }
          : { x: [[]], y: [[]] },
        [idx.uav]
      );

      if (idx.current >= 0) {
        if (PLAYBACK.currentPoint) {
          Plotly.restyle(
            plotId,
            {
              x: [[PLAYBACK.currentPoint.x]],
              y: [[PLAYBACK.currentPoint.y]],
            },
            [idx.current]
          );
        } else {
          Plotly.restyle(plotId, { x: [[]], y: [[]] }, [idx.current]);
        }
      }

      if (idx.visited >= 0) {
        const visX = [];
        const visY = [];
        PLAYBACK.visitedIds.forEach((id) => {
          const c = PLAYBACK.visitedCoords.get(id);
          if (c) {
            visX.push(c.x);
            visY.push(c.y);
          }
        });
        Plotly.restyle(plotId, { x: [visX], y: [visY] }, [idx.visited]);
      }
    } catch (err) {
      console.warn("playback restyle failed:", plotId, err);
    }
  }

  function updateAllPlotPlayback() {
    restylePlayback("mapPlot");
    const modal = $("mapFullscreenModal");
    if (modal && !modal.classList.contains("hidden")) {
      restylePlayback("mapPlotFs");
    }
  }

  function stopTimer() {
    if (PLAYBACK.timer != null) {
      clearTimeout(PLAYBACK.timer);
      PLAYBACK.timer = null;
    }
  }

  function setPlaybackUi() {
    const running = PLAYBACK.status === "playing";
    const paused = PLAYBACK.status === "paused";
    const hasTimeline = PLAYBACK.timeline.length > 0;
    $("playbackStartBtn")?.toggleAttribute("disabled", running);
    $("playbackPauseBtn")?.toggleAttribute("disabled", !running);
    $("playbackResumeBtn")?.toggleAttribute("disabled", !paused);
    $("playbackResetBtn")?.toggleAttribute("disabled", PLAYBACK.status === "idle" && !hasTimeline);
    const prog = $("playbackProgress");
    if (!prog) return;
    if (!hasTimeline) {
      prog.textContent = "待命";
      return;
    }
    const pct = Math.min(
      100,
      Math.round((PLAYBACK.index / PLAYBACK.timeline.length) * 100)
    );
    prog.textContent = `${pct}% · ${PLAYBACK.index}/${PLAYBACK.timeline.length}`;
  }

  function scheduleNext(delayMs) {
    stopTimer();
    PLAYBACK.timer = setTimeout(tickPlayback, Math.max(4, delayMs));
  }

  function onInspectComplete() {
    const ev = PLAYBACK.timeline[PLAYBACK.index - 1];
    if (!ev || ev.type !== "inspect") return;
    const pt = ev.point;
    const pid = pt.id || pt.point_id;
    PLAYBACK.visitedIds.add(pid);
    PLAYBACK.visitedCoords.set(pid, { x: pt.x, y: pt.y });
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;
    updateAllPlotPlayback();
    if (PLAYBACK.status === "playing") tickPlayback();
  }

  function tickPlayback() {
    if (PLAYBACK.status !== "playing") return;

    if (PLAYBACK.index >= PLAYBACK.timeline.length) {
      PLAYBACK.status = "finished";
      stopTimer();
      setPlaybackUi();
      ["", "Fs"].forEach((p) => {
        const st = $(`inspectCardStatus${p}`);
        if (st) {
          st.textContent = "任务完成";
          st.className = "inspect-status done";
        }
      });
      return;
    }

    const ev = PLAYBACK.timeline[PLAYBACK.index];
    PLAYBACK.index += 1;

    if (ev.type === "inspect") {
      PLAYBACK.uavPos = { x: ev.point.x, y: ev.point.y };
      PLAYBACK.currentPoint = { x: ev.point.x, y: ev.point.y };
      PLAYBACK.currentPointId = ev.point.id || ev.point.point_id;
      updateAllPlotPlayback();
      showInspectCard(ev.point, ev.inspect_index, PLAYBACK.totalInspectCount);
      stopTimer();
      const dwell = (ev.dwell_ms || getDwellMs()) / getSpeed();
      PLAYBACK.timer = setTimeout(onInspectComplete, dwell);
      setPlaybackUi();
      return;
    }

    if (ev.type === "move") {
      PLAYBACK.uavPos = { x: ev.x, y: ev.y };
      PLAYBACK.currentPoint = null;
      PLAYBACK.currentPointId = null;
      updateAllPlotPlayback();
      setPlaybackUi();
      scheduleNext(MOVE_FRAME_MS / getSpeed());
      return;
    }

    setPlaybackUi();
    scheduleNext(8 / getSpeed());
  }

  function buildTimelineFromResult(result) {
    PLAYBACK.dwellMs = getDwellMs();
    PLAYBACK.speed = getSpeed();
    PLAYBACK.timeline = window.buildInspectionPlaybackTimeline(result, PLAYBACK.dwellMs);
    PLAYBACK.totalInspectCount = PLAYBACK.timeline.filter((e) => e.type === "inspect").length;
    return PLAYBACK.timeline;
  }

  function initialUavPosition(result) {
    const start = result?.markers?.start;
    if (start) return { x: start.x, y: start.y };
    const first = PLAYBACK.timeline.find((e) => e.type === "move");
    if (first) return { x: first.x, y: first.y };
    return null;
  }

  window.resetPlayback = function (result) {
    stopTimer();
    PLAYBACK.status = "idle";
    PLAYBACK.index = 0;
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;
    PLAYBACK.visitedIds = new Set();
    PLAYBACK.visitedCoords = new Map();
    PLAYBACK.timeline = [];
    PLAYBACK.totalInspectCount = 0;
    clearInspectCard();

    if (result) {
      buildTimelineFromResult(result);
      PLAYBACK.uavPos = null;
    } else {
      PLAYBACK.uavPos = null;
    }

    setPlaybackUi();
    updateAllPlotPlayback();
  };

  window.onMissionLoadedForPlayback = function (result) {
    window.resetPlayback(result);
  };

  window.startInspectionPlayback = function () {
    const result = window.state?.lastResult;
    if (!result) {
      alert("请先生成/加载 Mission");
      return;
    }

    stopTimer();
    buildTimelineFromResult(result);

    if (!PLAYBACK.timeline.length) {
      alert("当前 Mission 无可播放轨迹");
      return;
    }

    PLAYBACK.index = 0;
    PLAYBACK.visitedIds = new Set();
    PLAYBACK.visitedCoords = new Map();
    PLAYBACK.uavPos = initialUavPosition(result);
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;
    PLAYBACK.status = "playing";
    PLAYBACK.speed = getSpeed();

    clearInspectCard();
    setPlaybackUi();
    updateAllPlotPlayback();

    requestAnimationFrame(() => {
      tickPlayback();
    });
  };

  window.pauseInspectionPlayback = function () {
    if (PLAYBACK.status !== "playing") return;
    stopTimer();
    PLAYBACK.status = "paused";
    setPlaybackUi();
  };

  window.resumeInspectionPlayback = function () {
    if (PLAYBACK.status !== "paused") return;
    PLAYBACK.status = "playing";
    PLAYBACK.speed = getSpeed();
    setPlaybackUi();
    tickPlayback();
  };

  window.resetInspectionPlayback = function () {
    window.resetPlayback(window.state?.lastResult || null);
  };

  document.addEventListener("DOMContentLoaded", () => {
    $("playbackStartBtn")?.addEventListener("click", () => window.startInspectionPlayback());
    $("playbackPauseBtn")?.addEventListener("click", () => window.pauseInspectionPlayback());
    $("playbackResumeBtn")?.addEventListener("click", () => window.resumeInspectionPlayback());
    $("playbackResetBtn")?.addEventListener("click", () => window.resetInspectionPlayback());
    clearInspectCard();
    setPlaybackUi();
  });
})();
