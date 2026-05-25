/**
 * Dashboard 巡检过程模拟播放
 */
(function () {
  const $ = (id) => document.getElementById(id);

  const MOVE_STEP_PX = 10;
  const MOVE_FRAME_MS = 20;
  const POINT_TRIGGER_DIST = 8;
  const CONTINUITY_EPS = 5;
  const STREAM_INTERVAL_MIN = 300;
  const STREAM_INTERVAL_MAX = 1000;
  const MAX_EVENT_ITEMS = 10;
  const STATE_TEXT = {
    IDLE: "待命",
    TAKEOFF: "起飞中",
    CRUISING: "巡航中",
    LEAVING: "离开巡检点",
    APPROACHING: "接近巡检点",
    INSPECTING: "巡检中",
    CAPTURING: "采集中",
    COMPLETED: "任务完成",
    PAUSED: "已暂停",
  };
  const STREAM_FALLBACKS = [
    "/static/inspection_placeholder.svg",
    "/static/inspection_placeholder_1.svg",
    "/static/inspection_placeholder_2.svg",
  ];
  const CRUISE_STREAM_IMAGES = [...STREAM_FALLBACKS];
  const APPROACH_STREAM_IMAGES = [
    "/static/inspection_placeholder.svg",
    "/static/inspection_placeholder_1.svg",
    "/static/inspection_placeholder.svg",
  ];

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
    streamTimer: null,
    streamImages: [],
    streamIndex: 0,
    streamCurrentUrl: STREAM_FALLBACKS[0],
    streamFrozen: false,
    missionState: "IDLE",
    events: [],
    startTimestamp: 0,
    telemetry: {
      fps: "--",
      signal: "--",
      battery: "--",
      altitude: "--",
      speed: "--",
      mode: "待命",
    },
    phaseTimers: [],
  };

  function getMissionForPlayback(fallback) {
    if (window.MissionStore?.getCurrentMission) {
      return MissionStore.getCurrentMission() || fallback || null;
    }
    if (window.MissionStore?.getMission) {
      return MissionStore.getMission() || fallback || null;
    }
    // deprecated alias
    return window.state?.lastResult || fallback || null;
  }

  function notifyPlaybackPhase(status) {
    if (window.AppPhaseManager?.setPlaybackStatus) {
      AppPhaseManager.setPlaybackStatus(status);
    }
  }

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

  function setForBoth(id, value) {
    ["", "Fs"].forEach((p) => {
      const el = $(`${id}${p}`);
      if (el) el.textContent = value;
    });
  }

  function setMissionState(state) {
    PLAYBACK.missionState = state;
    const text = STATE_TEXT[state] || state;
    setForBoth("inspectCardUavStatus", text);
    setForBoth("inspectTelemMode", text);
    ["", "Fs"].forEach((p) => {
      const badge = $(`inspectCardStatus${p}`);
      if (!badge) return;
      if (state === "IDLE" || state === "PAUSED") badge.className = "inspect-status idle";
      else if (state === "COMPLETED") badge.className = "inspect-status done";
      else badge.className = "inspect-status shooting";
      if (state === "PAUSED") badge.textContent = "已暂停";
      else if (state === "COMPLETED") badge.textContent = "任务完成";
      else if (state === "CAPTURING") badge.textContent = "正在拍摄";
      else badge.textContent = text;
    });
  }

  function elapsedTag() {
    const ms = Math.max(0, Date.now() - (PLAYBACK.startTimestamp || Date.now()));
    const sec = Math.floor(ms / 1000);
    const mm = String(Math.floor(sec / 60)).padStart(2, "0");
    const ss = String(sec % 60).padStart(2, "0");
    return `T+${mm}:${ss}`;
  }

  function renderEventTimeline(prefix) {
    const list = $(`inspectEventList${prefix || ""}`);
    if (!list) return;
    list.innerHTML = PLAYBACK.events
      .map((item) => `<li><span>${item.ts}</span><b>${item.text}</b></li>`)
      .join("");
    list.scrollTop = list.scrollHeight;
  }

  function pushEvent(text) {
    PLAYBACK.events.push({ ts: elapsedTag(), text });
    if (PLAYBACK.events.length > MAX_EVENT_ITEMS) {
      PLAYBACK.events = PLAYBACK.events.slice(-MAX_EVENT_ITEMS);
    }
    renderEventTimeline("");
    renderEventTimeline("Fs");
  }

  function updateProgressVisual() {
    const total = Math.max(PLAYBACK.timeline.length, 1);
    const pct = Math.round((Math.min(PLAYBACK.index, total) / total) * 100);
    setForBoth("inspectCardProgressBarText", `${pct}%`);
    ["", "Fs"].forEach((pfx) => {
      const fill = document.querySelector(`#inspectInfoCard${pfx} .mission-progress i`);
      if (fill) fill.style.width = `${pct}%`;
    });
  }

  function renderTelemetry() {
    setForBoth("inspectHudFps", PLAYBACK.telemetry.fps);
    setForBoth("inspectHudSignal", PLAYBACK.telemetry.signal);
    setForBoth("inspectHudBattery", PLAYBACK.telemetry.battery);
    setForBoth("inspectHudAlt", PLAYBACK.telemetry.altitude);
    setForBoth("inspectHudSpeed", PLAYBACK.telemetry.speed);

    setForBoth("inspectTelemSignal", PLAYBACK.telemetry.signal);
    setForBoth("inspectTelemBattery", PLAYBACK.telemetry.battery);
    setForBoth("inspectTelemAlt", PLAYBACK.telemetry.altitude);
    setForBoth("inspectTelemMode", STATE_TEXT[PLAYBACK.missionState] || "自动");
  }

  function updateTelemetryFromPlayback(stateHint) {
    const total = Math.max(PLAYBACK.timeline.length, 1);
    const progress = Math.min(1, PLAYBACK.index / total);
    const speedFactor = getSpeed();
    const fps = Math.round(24 + Math.random() * 8);
    const signal = Math.max(66, Math.round(97 - progress * 18 - (Math.random() * 3)));
    const battery = Math.max(18, Math.round(100 - progress * 78));
    const altitude = Math.round(30 + Math.sin(PLAYBACK.index * 0.12) * 3 + (stateHint === "APPROACHING" ? -1 : 1));
    const speedVal =
      stateHint === "INSPECTING" || stateHint === "CAPTURING"
        ? 0
        : stateHint === "APPROACHING"
          ? (5.5 * speedFactor)
          : (10.8 * speedFactor);

    PLAYBACK.telemetry = {
      fps: `${fps}`,
      signal: `${signal}%`,
      battery: `${battery}%`,
      altitude: `${altitude}m`,
      speed: `${speedVal.toFixed(1)}m/s`,
      mode: STATE_TEXT[PLAYBACK.missionState] || "自动",
    };
    renderTelemetry();
    updateProgressVisual();
  }

  function setStreamImage(url, hintText) {
    PLAYBACK.streamCurrentUrl = url || STREAM_FALLBACKS[0];
    ["", "Fs"].forEach((p) => {
      const img = $(`inspectCardImg${p}`);
      if (!img) return;
      img.onerror = () => {
        img.src = STREAM_FALLBACKS[(PLAYBACK.streamIndex + 1) % STREAM_FALLBACKS.length];
      };
      img.src = PLAYBACK.streamCurrentUrl;
      const hint = $(`inspectCardImgHint${p}`);
      if (hint) hint.textContent = hintText || "实时图传 · 图像持续更新中";
    });
  }

  function buildStreamPool(result) {
    const set = new Set();
    (result?.inspection_points || []).forEach((pt) => {
      [pt.image_url, pt.image_path, pt.image_placeholder].forEach((v) => {
        if (v && typeof v === "string") set.add(v);
      });
    });
    for (let i = 1; i <= 36; i += 1) {
      const name = `IP_${String(i).padStart(4, "0")}.jpg`;
      set.add(`/api/inspection-image/${name}`);
    }
    STREAM_FALLBACKS.forEach((v) => set.add(v));
    return [...set];
  }

  function stopStreamLoop() {
    if (PLAYBACK.streamTimer != null) {
      clearTimeout(PLAYBACK.streamTimer);
      PLAYBACK.streamTimer = null;
    }
  }

  function clearPhaseTimers() {
    PLAYBACK.phaseTimers.forEach((t) => clearTimeout(t));
    PLAYBACK.phaseTimers = [];
  }

  function streamDelayByState() {
    if (PLAYBACK.missionState === "APPROACHING") {
      return Math.round(700 + Math.random() * 250);
    }
    if (PLAYBACK.missionState === "TAKEOFF") {
      return Math.round(460 + Math.random() * 260);
    }
    return Math.round(STREAM_INTERVAL_MIN + Math.random() * (STREAM_INTERVAL_MAX - STREAM_INTERVAL_MIN) * 0.6);
  }

  function scheduleStreamLoop() {
    stopStreamLoop();
    if (PLAYBACK.status !== "playing" || PLAYBACK.streamFrozen) return;
    PLAYBACK.streamTimer = setTimeout(() => {
      if (PLAYBACK.status !== "playing" || PLAYBACK.streamFrozen) return;
      const state = PLAYBACK.missionState;
      const streamPool =
        state === "APPROACHING" ? APPROACH_STREAM_IMAGES : CRUISE_STREAM_IMAGES;
      PLAYBACK.streamIndex = (PLAYBACK.streamIndex + 1) % streamPool.length;
      const hintText =
        state === "APPROACHING"
          ? "接近巡检点 · 图传轻微闪烁"
          : "实时图传 · 巡航占位画面";
      setStreamImage(streamPool[PLAYBACK.streamIndex], hintText);
      updateTelemetryFromPlayback(PLAYBACK.missionState);
      scheduleStreamLoop();
    }, streamDelayByState());
  }

  function ensureInspectCardVisible() {
    ["", "Fs"].forEach((p) => {
      const card = $(`inspectInfoCard${p}`);
      if (card) card.classList.remove("hidden");
    });
  }

  function freezeStreamAtPoint(point) {
    PLAYBACK.streamFrozen = true;
    stopStreamLoop();
    const capture = point?.image_url || point?.image_path || PLAYBACK.streamCurrentUrl || STREAM_FALLBACKS[0];
    setStreamImage(capture, "图像冻结 · 拍摄中");
  }

  function nextInspectOffset(fromIndex) {
    for (let i = fromIndex; i < PLAYBACK.timeline.length; i += 1) {
      if (PLAYBACK.timeline[i]?.type === "inspect") return i - fromIndex;
    }
    return -1;
  }

  function clearInspectCard() {
    clearPhaseTimers();
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
      const list = $(`inspectEventList${p}`);
      if (list) list.innerHTML = "<li><span>T+00:00</span><b>系统待命</b></li>";
    });
    PLAYBACK.events = [{ ts: "T+00:00", text: "系统待命" }];
    PLAYBACK.streamCurrentUrl = STREAM_FALLBACKS[0];
    PLAYBACK.streamFrozen = false;
    PLAYBACK.telemetry = {
      fps: "--",
      signal: "--",
      battery: "--",
      altitude: "--",
      speed: "--",
      mode: "待命",
    };
    setMissionState("IDLE");
    renderTelemetry();
    updateProgressVisual();
  }

  function formatPointLabel(raw) {
    if (raw == null) return "—";
    const core = String(raw).replace(/^point_?/i, "");
    return `巡检点 ${core}`;
  }

  function formatSegmentLabel(raw) {
    if (raw == null) return "—";
    const core = String(raw).replace(/^seg_?/i, "");
    return `巡检区段 ${core}`;
  }

  function formatPointType(raw) {
    const t = String(raw || "").toLowerCase();
    if (t === "endpoint") return "终端巡检点";
    if (t === "turning") return "转角巡检点";
    if (t === "sample") return "普通巡检点";
    return raw || "普通巡检点";
  }

  function formatPriority(raw) {
    const p = String(raw || "").toLowerCase();
    if (p === "high") return "高";
    if (p === "normal") return "普通";
    return raw || "普通";
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
    set("inspectCardId", formatPointLabel(point.point_id || point.id));
    set("inspectCardSegment", formatSegmentLabel(point.segment_id || point.edge_id));
    set("inspectCardCoord", `${Math.round(point.x)}, ${Math.round(point.y)}`);
    set("inspectCardType", formatPointType(point.point_type));
    set("inspectCardPriority", formatPriority(point.priority));
    set("inspectCardProgress", `${index} / ${Math.max(total, 1)}`);
    set("inspectCardUavStatus", STATE_TEXT[PLAYBACK.missionState] || "巡检中");
    set("inspectCardTime", new Date().toLocaleTimeString("zh-CN"));
    const img = $(`inspectCardImg${p}`);
    const imageUrl =
      point.image_url ||
      point.image_path ||
      point.image_placeholder;
    const finalUrl = PLAYBACK.imageCache.get(imageUrl) || imageUrl;
    if (img && finalUrl) {
      img.onerror = () => {
        img.src = point.image_placeholder || PLAYBACK.streamCurrentUrl || STREAM_FALLBACKS[0];
        PLAYBACK.usingPlaceholder = true;
      };
      img.src = finalUrl;
      PLAYBACK.streamCurrentUrl = finalUrl;
    }
    if (imageUrl && !PLAYBACK.imageCache.has(imageUrl)) {
      const preload = new Image();
      preload.onload = () => PLAYBACK.imageCache.set(imageUrl, imageUrl);
      preload.src = imageUrl;
    }
    const hint = $(`inspectCardImgHint${p}`);
    if (hint) {
      hint.textContent =
        point.image_available && point.image_url
          ? "巡检图像采集成功"
          : "巡检图像已冻结，使用占位图";
    }
    const st = $(`inspectCardStatus${p}`);
    if (st) {
      st.textContent = "正在拍照";
      st.className = "inspect-status shooting";
    }
  }

  function showInspectCard(point, index, total) {
    ensureInspectCardVisible();
    fillInspectCard("", point, index, total);
    fillInspectCard("Fs", point, index, total);
    const pid = point.id || point.point_id;
    const dwell = Math.max(600, getDwellMs());
    clearPhaseTimers();
    setMissionState("APPROACHING");
    updateTelemetryFromPlayback("APPROACHING");
    ["", "Fs"].forEach((pfx) => {
      const hint = $(`inspectCardImgHint${pfx}`);
      if (hint) hint.textContent = "接近巡检点 · 正在稳定云台";
    });
    pushEvent(`到达巡检点 ${formatPointLabel(point.point_id || pid)}`);

    const t1 = setTimeout(() => {
      if (PLAYBACK.currentPointId !== pid) return;
      setMissionState("CAPTURING");
      updateTelemetryFromPlayback("CAPTURING");
      ["", "Fs"].forEach((pfx) => {
        const hint = $(`inspectCardImgHint${pfx}`);
        if (hint) hint.textContent = "正在拍摄 · 图像冻结采集中";
      });
      pushEvent("正在拍摄");
    }, Math.min(260, dwell * 0.2));
    PLAYBACK.phaseTimers.push(t1);

    const t2 = setTimeout(() => {
      if (PLAYBACK.currentPointId !== pid) return;
      ["", "Fs"].forEach((pfx) => {
        const st = $(`inspectCardStatus${pfx}`);
        if (st) {
          st.textContent = "图像采集完成";
          st.className = "inspect-status done";
        }
        const hint = $(`inspectCardImgHint${pfx}`);
        if (hint) hint.textContent = "✓ 图像采集完成";
      });
      setForBoth("inspectCardUavStatus", "图像采集完成");
      pushEvent("图像采集完成");
    }, Math.min(720, dwell * 0.55));
    PLAYBACK.phaseTimers.push(t2);

    const t3 = setTimeout(() => {
      if (PLAYBACK.currentPointId !== pid) return;
      ["", "Fs"].forEach((pfx) => {
        const st = $(`inspectCardStatus${pfx}`);
        if (st) {
          st.textContent = "AI 分析完成";
          st.className = "inspect-status done";
        }
        const hint = $(`inspectCardImgHint${pfx}`);
        if (hint) hint.textContent = "✓ 巡检完成 · ✓ 图像已采集 · ✓ AI 分析完成";
      });
      setForBoth("inspectCardUavStatus", "AI 分析完成");
      pushEvent("AI 分析完成");
    }, Math.min(1200, dwell * 0.82));
    PLAYBACK.phaseTimers.push(t3);
  }

  function buildPlaybackTraces() {
    return [
      {
        name: "无人机光晕",
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
        name: "无人机",
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
        name: "当前巡检点",
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
        name: "已巡检点",
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
      uavPos: PLAYBACK.uavPos ? { ...PLAYBACK.uavPos } : null,
      timelineIndex: PLAYBACK.index,
      timelineLength: PLAYBACK.timeline.length,
      playbackSpeed: PLAYBACK.speed || 1,
    };
  };

  function getTraceIndices(plotId) {
    const el = document.getElementById(plotId);
    if (!el?.data?.length) return null;
    const idx = (name) => el.data.findIndex((t) => t.name === name);
    const uav = idx("无人机");
    if (uav < 0) return null;
    return {
      glow: idx("无人机光晕"),
      uav,
      current: idx("当前巡检点"),
      visited: idx("已巡检点"),
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
      console.warn("巡检播放重绘失败:", plotId, err);
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
    const stateText = STATE_TEXT[PLAYBACK.missionState] || "执行中";
    prog.textContent = `${pct}% · ${PLAYBACK.index}/${PLAYBACK.timeline.length} · ${stateText}`;
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
    clearPhaseTimers();
    PLAYBACK.streamFrozen = false;
    setMissionState("LEAVING");
    updateTelemetryFromPlayback("LEAVING");
    setStreamImage(CRUISE_STREAM_IMAGES[PLAYBACK.streamIndex % CRUISE_STREAM_IMAGES.length], "离开巡检点 · 恢复巡航图传");
    const toCruise = setTimeout(() => {
      if (PLAYBACK.status !== "playing") return;
      setMissionState("CRUISING");
      updateTelemetryFromPlayback("CRUISING");
    }, 450);
    PLAYBACK.phaseTimers.push(toCruise);
    scheduleStreamLoop();
    updateAllPlotPlayback();
    if (PLAYBACK.status === "playing") tickPlayback();
  }

  function tickPlayback() {
    if (PLAYBACK.status !== "playing") return;

    if (PLAYBACK.index >= PLAYBACK.timeline.length) {
      PLAYBACK.status = "finished";
      notifyPlaybackPhase("finished");
      stopTimer();
      stopStreamLoop();
      PLAYBACK.streamFrozen = true;
      setMissionState("COMPLETED");
      updateTelemetryFromPlayback("COMPLETED");
      pushEvent("任务执行完成");
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
      freezeStreamAtPoint(ev.point);
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
      const nearOffset = nextInspectOffset(PLAYBACK.index);
      if (PLAYBACK.index < 8) setMissionState("TAKEOFF");
      else if (nearOffset >= 0 && nearOffset < 18) setMissionState("APPROACHING");
      else if (PLAYBACK.missionState !== "LEAVING") setMissionState("CRUISING");
      updateTelemetryFromPlayback(PLAYBACK.missionState);
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
    stopStreamLoop();
    clearPhaseTimers();
    PLAYBACK.status = "idle";
    PLAYBACK.index = 0;
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;
    PLAYBACK.visitedIds = new Set();
    PLAYBACK.visitedCoords = new Map();
    PLAYBACK.timeline = [];
    PLAYBACK.totalInspectCount = 0;
    PLAYBACK.startTimestamp = 0;
    PLAYBACK.streamFrozen = false;
    clearInspectCard();

    if (result) {
      buildTimelineFromResult(result);
      PLAYBACK.streamImages = buildStreamPool(result);
      PLAYBACK.streamIndex = 0;
      PLAYBACK.uavPos = null;
    } else {
      PLAYBACK.streamImages = [...STREAM_FALLBACKS];
      PLAYBACK.streamIndex = 0;
      PLAYBACK.uavPos = null;
    }

    setPlaybackUi();
    updateAllPlotPlayback();
    notifyPlaybackPhase("idle");
  };

  window.onMissionLoadedForPlayback = function (result) {
    window.resetPlayback(result || getMissionForPlayback());
  };

  window.reloadPlaybackTimeline = function (result, options = {}) {
    const mission = result || getMissionForPlayback();
    const wasPlaying = PLAYBACK.status === "playing";
    window.resetPlayback(mission);
    if (options.restart && wasPlaying) {
      window.startInspectionPlayback({ autoStart: true });
    }
  };

  window.startInspectionPlayback = function (options = {}) {
    const result = getMissionForPlayback();
    if (!result) {
      alert("请先生成/加载任务");
      return;
    }

    stopTimer();
    stopStreamLoop();
    buildTimelineFromResult(result);

    if (!PLAYBACK.timeline.length) {
      alert("当前任务无可播放轨迹");
      return;
    }

    PLAYBACK.index = 0;
    PLAYBACK.visitedIds = new Set();
    PLAYBACK.visitedCoords = new Map();
    PLAYBACK.uavPos = initialUavPosition(result);
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;
    PLAYBACK.status = "playing";
    notifyPlaybackPhase("playing");
    PLAYBACK.speed = getSpeed();
    PLAYBACK.startTimestamp = Date.now();
    PLAYBACK.streamFrozen = false;
    PLAYBACK.streamImages = buildStreamPool(result);
    PLAYBACK.streamIndex = 0;

    clearInspectCard();
    ensureInspectCardVisible();
    setMissionState("TAKEOFF");
    pushEvent("任务开始执行");
    setStreamImage(CRUISE_STREAM_IMAGES[0], "实时图传已启动");
    updateTelemetryFromPlayback("TAKEOFF");
    scheduleStreamLoop();
    setPlaybackUi();
    updateAllPlotPlayback();

    requestAnimationFrame(() => {
      tickPlayback();
    });
  };

  window.pauseInspectionPlayback = function () {
    if (PLAYBACK.status !== "playing") return;
    stopTimer();
    stopStreamLoop();
    clearPhaseTimers();
    PLAYBACK.status = "paused";
    notifyPlaybackPhase("paused");
    setMissionState("PAUSED");
    updateTelemetryFromPlayback("PAUSED");
    pushEvent("任务已暂停");
    setPlaybackUi();
  };

  window.resumeInspectionPlayback = function () {
    if (PLAYBACK.status !== "paused") return;
    PLAYBACK.status = "playing";
    notifyPlaybackPhase("playing");
    PLAYBACK.speed = getSpeed();
    setMissionState("CRUISING");
    updateTelemetryFromPlayback("CRUISING");
    scheduleStreamLoop();
    pushEvent("任务继续执行");
    setPlaybackUi();
    tickPlayback();
  };

  window.resetInspectionPlayback = function () {
    window.resetPlayback(getMissionForPlayback());
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
