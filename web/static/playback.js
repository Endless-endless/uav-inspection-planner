/**
 * Dashboard 巡检过程模拟播放
 */
(function () {
  const $ = (id) => document.getElementById(id);

  /** 关闭图传/快照/右侧巡检卡片相关 DOM，避免干扰主播放链路 */
  const ENABLE_INSPECTION_IMAGE = true;

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
    EXECUTING: "巡检执行",
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
    /** segment_id -> 已对该 inspect 段批量点亮同 edge_id 巡检点 */
    activatedInspectSegments: new Set(),
    /** 巡检 API 图显示期间禁止图传轮播覆盖快照 */
    holdInspectSnapshot: false,
    /** 与 status 同步，便于调试与 UI 文案 */
    running: false,
    paused: false,
    t: 0,
  };

  function getMissionForPlayback() {
    const result =
      window.MissionStore?.current?.dashboard ||
      window.MissionStore?.current ||
      window.currentMission?.dashboard ||
      window.currentMission ||
      window.LATEST_DASHBOARD ||
      null;
    console.log("[getMissionForPlayback]", result);
    return result;
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
                edge_id:
                  seg.edge_id != null ? String(seg.edge_id).replace(/[+-]$/, "") : "",
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
    if (!ENABLE_INSPECTION_IMAGE) return;
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
    if (!ENABLE_INSPECTION_IMAGE) return;
    renderEventTimeline("");
    renderEventTimeline("Fs");
  }

  function updateProgressVisual() {
    if (!ENABLE_INSPECTION_IMAGE) return;
    const total = Math.max(PLAYBACK.timeline.length, 1);
    const pct = Math.round((Math.min(PLAYBACK.index, total) / total) * 100);
    setForBoth("inspectCardProgressBarText", `${pct}%`);
    ["", "Fs"].forEach((pfx) => {
      const fill = document.querySelector(`#inspectInfoCard${pfx} .mission-progress i`);
      if (fill) fill.style.width = `${pct}%`;
    });
  }

  function hudDisplayValue(raw, kind) {
    if (raw == null || raw === "" || raw === "--") return "--";
    const text = String(raw);
    if (kind === "speed") return text.replace(/m\/s/gi, "").trim();
    if (kind === "altitude") return text.replace(/m$/i, "").trim();
    if (kind === "battery") return text.replace(/%$/i, "").trim();
    return text;
  }

  function riskLabelFromSignal(signalStr) {
    const raw = String(signalStr || "").trim();
    const n = parseInt(raw.replace(/\D/g, ""), 10);
    if (!Number.isFinite(n)) return raw || "—";
    if (n >= 90) return `良好（${raw}）`;
    if (n >= 75) return `一般（${raw}）`;
    return `关注（${raw}）`;
  }

  /** 右侧精简面板：当前点 / 速度 / 高度 / 风险 / 进度 */
  function syncInspectCompactMeta(suffix) {
    if (!ENABLE_INSPECTION_IMAGE) return;
    const p = suffix || "";
    const set = (baseId, val) => {
      const el = $(`${baseId}${p}`);
      if (el) el.textContent = val;
    };
    const t = PLAYBACK.telemetry || {};
    set("inspectMetaSpeed", t.speed || "—");
    set("inspectMetaAlt", t.altitude || "—");
    set("inspectMetaRisk", riskLabelFromSignal(t.signal));
    const total = Math.max(PLAYBACK.timeline?.length || 0, 1);
    const pct = Math.min(100, Math.round(((PLAYBACK.index || 0) / total) * 100));
    set("inspectMetaProgress", `${pct}%`);
    let pid = PLAYBACK.currentPointId ? String(PLAYBACK.currentPointId) : "";
    if (!pid && PLAYBACK.currentPoint) {
      pid = String(PLAYBACK.currentPoint.point_id || PLAYBACK.currentPoint.id || "").trim();
    }
    set("inspectMetaCurrentPoint", pid || "—");
  }

  function ensureInspectCompactMetaBlock(mount, suffix) {
    const p = suffix || "";
    const tag = p ? "fs" : "main";
    if (!mount || mount.querySelector(`[data-inspect-compact-meta="${tag}"]`)) return;
    const wrap = document.createElement("div");
    wrap.className = "inspect-compact-meta";
    wrap.setAttribute("data-inspect-compact-meta", tag);
    wrap.innerHTML = `
      <div class="inspect-compact-meta-row"><span class="inspect-compact-k">当前巡检点</span><span class="inspect-compact-v" id="inspectMetaCurrentPoint${p}">—</span></div>
      <div class="inspect-compact-meta-row"><span class="inspect-compact-k">速度</span><span class="inspect-compact-v" id="inspectMetaSpeed${p}">—</span></div>
      <div class="inspect-compact-meta-row"><span class="inspect-compact-k">高度</span><span class="inspect-compact-v" id="inspectMetaAlt${p}">—</span></div>
      <div class="inspect-compact-meta-row"><span class="inspect-compact-k">风险等级</span><span class="inspect-compact-v" id="inspectMetaRisk${p}">—</span></div>
      <div class="inspect-compact-meta-row"><span class="inspect-compact-k">进度</span><span class="inspect-compact-v" id="inspectMetaProgress${p}">—</span></div>`;
    mount.appendChild(wrap);
  }

  function hideLegacyInspectStreamUI(mount, suffix) {
    const p = suffix || "";
    const sid = p === "Fs" ? "inspectStreamPanelFs" : "inspectStreamPanel";
    const el = (mount && mount.querySelector(`#${sid}`)) || document.getElementById(sid);
    if (el) {
      el.classList.add("inspect-ui-hidden-legacy");
      el.setAttribute("aria-hidden", "true");
    }
  }

  function renderTelemetry() {
    if (!ENABLE_INSPECTION_IMAGE) return;
    setForBoth("inspectHudFps", PLAYBACK.telemetry.fps);
    setForBoth("inspectHudSignal", PLAYBACK.telemetry.signal);
    setForBoth("inspectHudBattery", hudDisplayValue(PLAYBACK.telemetry.battery, "battery"));
    setForBoth("inspectHudAlt", hudDisplayValue(PLAYBACK.telemetry.altitude, "altitude"));
    setForBoth("inspectHudSpeed", hudDisplayValue(PLAYBACK.telemetry.speed, "speed"));

    setForBoth("inspectTelemSignal", PLAYBACK.telemetry.signal);
    setForBoth("inspectTelemBattery", PLAYBACK.telemetry.battery);
    setForBoth("inspectTelemAlt", PLAYBACK.telemetry.altitude);
    setForBoth("inspectTelemMode", STATE_TEXT[PLAYBACK.missionState] || "自动");
    syncInspectCompactMeta("");
    syncInspectCompactMeta("Fs");
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
    if (ENABLE_INSPECTION_IMAGE) {
      renderTelemetry();
      updateProgressVisual();
    }
  }

  function setStreamImage(url, hintText) {
    if (!ENABLE_INSPECTION_IMAGE) return;
    if (PLAYBACK.streamFrozen) return;
    PLAYBACK.streamCurrentUrl = url || STREAM_FALLBACKS[0];
    ["", "Fs"].forEach((p) => {
      ensureInspectDualSlotsMounted(p);
      const img = $(`inspectCardImg${p}`);
      if (!img) return;
      img.onerror = () => {
        img.src = STREAM_FALLBACKS[(PLAYBACK.streamIndex + 1) % STREAM_FALLBACKS.length];
      };
      img.src = PLAYBACK.streamCurrentUrl;
      img.style.display = "block";
      const cap = $(`inspectCardCaption${p}`);
      if (cap) {
        cap.textContent = hintText || "";
        cap.classList.remove("inspect-live-placeholder--show");
      }
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
    if (!ENABLE_INSPECTION_IMAGE) return;
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
    if (!ENABLE_INSPECTION_IMAGE) return;
    ensureInspectFallbackCardIfMissing();
    ["", "Fs"].forEach((p) => {
      const card = $(`inspectInfoCard${p}`);
      if (card) card.classList.remove("hidden");
    });
  }

  function freezeStreamAtPoint(point) {
    PLAYBACK.streamFrozen = true;
    stopStreamLoop();
    if (!ENABLE_INSPECTION_IMAGE) return;
    const u =
      typeof point?.image_url === "string" && point.image_url.trim()
        ? point.image_url.trim()
        : null;
    PLAYBACK.streamCurrentUrl =
      u ||
      (typeof point?.image_path === "string" && point.image_path.trim()
        ? point.image_path.trim()
        : null) ||
      PLAYBACK.streamCurrentUrl ||
      STREAM_FALLBACKS[0];
  }

  function nextInspectOffset(fromIndex) {
    for (let i = fromIndex; i < PLAYBACK.timeline.length; i += 1) {
      if (PLAYBACK.timeline[i]?.type === "inspect") return i - fromIndex;
    }
    return -1;
  }

  function clearInspectCard() {
    clearPhaseTimers();
    if (!ENABLE_INSPECTION_IMAGE) {
      PLAYBACK.events = [{ ts: "T+00:00", text: "系统待命" }];
      PLAYBACK.streamCurrentUrl = STREAM_FALLBACKS[0];
      PLAYBACK.streamFrozen = false;
      PLAYBACK.holdInspectSnapshot = false;
      PLAYBACK.telemetry = {
        fps: "--",
        signal: "--",
        battery: "--",
        altitude: "--",
        speed: "--",
        mode: "待命",
      };
      PLAYBACK.missionState = "IDLE";
      return;
    }
    ["", "Fs"].forEach((p) => {
      const card = $(`inspectInfoCard${p}`);
      if (!card) return;
      card.classList.add("hidden");
      const sImg = $(`inspectStreamImg${p}`);
      if (sImg) {
        sImg.onerror = null;
        sImg.src = "/static/inspection_placeholder.svg";
      }
      const img = $(`inspectCardImg${p}`);
      if (img) {
        img.onerror = null;
        img.removeAttribute("src");
        img.style.display = "none";
      }
      const cap = $(`inspectCardCaption${p}`);
      if (cap) {
        cap.textContent = "暂无巡检图片";
        cap.classList.add("inspect-live-placeholder--show");
      }
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
    PLAYBACK.holdInspectSnapshot = false;
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

  function resolveInspectionCardImage(point) {
    const pointId = String(point?.point_id || point?.id || "").trim() || "—";
    const url =
      typeof point?.image_url === "string" && point.image_url.trim()
        ? point.image_url.trim()
        : null;
    return {
      url,
      logPath: "",
      pointId,
      hasFile: point?.image_available === true,
    };
  }

  function findFlightStatusMountEl(forFs) {
    if (forFs) {
      const c = document.getElementById("inspectInfoCardFs");
      if (c) return c;
    }
    const rowCard = document.querySelector(".map-inspect-row .inspect-info-card");
    if (rowCard) return rowCard;
    const ids = [
      "inspectInfoCard",
      "playbackFlightCard",
      "flightStatusCard",
      "dashboardFlightPanel",
      "playbackSidePanel",
    ];
    for (let i = 0; i < ids.length; i += 1) {
      const el = document.getElementById(ids[i]);
      if (el) return el;
    }
    return (
      document.querySelector("[data-inspect-card-mount]") ||
      document.querySelector("[data-flight-status-card]") ||
      document.querySelector(".flight-status-card") ||
      document.querySelector("#playbackConsole .console-body") ||
      null
    );
  }

  /**
   * 地图点选：仅用 payload 的 image_url 更新巡检快照 #inspectCardImg（与播放流 inspectStreamImg 分离）。
   * 不移动/重建整块布局，避免撑宽右侧栏。
   */
  function forceDashboardInspectionImagePanel(point) {
    if (!ENABLE_INSPECTION_IMAGE) return;
    const pointIdStr = String(point?.point_id || point?.id || "").trim() || "—";
    const url =
      typeof point?.image_url === "string" && point.image_url.trim()
        ? point.image_url.trim()
        : null;

    ensureInspectCardVisible();
    ensureInspectDualSlotsMounted("");

    const img = $("inspectCardImg");
    const cap = $("inspectCardCaption");
    if (!img || !cap) return;

    if (!url) {
      cap.textContent = "暂无巡检图片";
      cap.classList.add("inspect-live-placeholder--show");
      img.onerror = null;
      img.removeAttribute("src");
      img.style.display = "none";
      return;
    }

    cap.classList.remove("inspect-live-placeholder--show");
    cap.textContent = "加载中…";
    img.classList.add("inspect-snapshot-img");
    img.style.display = "block";
    img.onload = () => {
      cap.textContent = pointIdStr !== "—" ? pointIdStr : "巡检图像";
    };
    img.onerror = () => {
      cap.textContent = "暂无巡检图片";
      img.onerror = null;
      img.removeAttribute("src");
      img.style.display = "none";
    };
    img.src = url;
  }

  function insertAfter(parent, node, ref) {
    if (!ref || ref.parentNode !== parent) {
      parent.insertBefore(node, parent.firstChild);
      return;
    }
    if (ref.nextSibling) parent.insertBefore(node, ref.nextSibling);
    else parent.appendChild(node);
  }

  /**
   * 在飞行状态卡片内挂载「实时图传」单图区域（inspectCardImg），隐藏旧版双图结构。
   */
  function ensureInspectDualSlotsMounted(suffix) {
    const p = suffix || "";
    const snapPanelId = p === "Fs" ? "inspectImagePanelFs" : "inspectImagePanel";
    const snapImgId = `inspectCardImg${p}`;
    const capId = p === "Fs" ? "inspectCardCaptionFs" : "inspectCardCaption";

    let mount = null;
    if (p === "Fs") {
      mount = document.getElementById("inspectInfoCardFs") || findFlightStatusMountEl(true);
    }
    if (!mount) mount = findFlightStatusMountEl(false);
    if (!mount) {
      ensureInspectInfoCardShellIfMissing();
      mount = document.getElementById("inspectInfoCard");
    }
    if (!mount) return false;

    hideLegacyInspectStreamUI(mount, p);

    const tri = mount.querySelector(".triangle-marker");
    const anchor =
      tri ||
      mount.querySelector("[data-inspect-image-anchor]") ||
      mount.querySelector(".flight-status-body") ||
      mount.firstElementChild;

    let snapPanel = $(snapPanelId);
    if (!snapPanel) {
      snapPanel = document.createElement("div");
      snapPanel.id = snapPanelId;
      snapPanel.className = "inspect-image-panel inspect-snapshot-panel inspect-live-image-wrap";
      const img = document.createElement("img");
      img.id = snapImgId;
      img.className = "inspect-flight-img inspect-snapshot-img";
      img.alt = "实时图传";
      const capEl = document.createElement("div");
      capEl.id = capId;
      capEl.className = "inspect-card-caption inspect-live-placeholder inspect-live-placeholder--show";
      capEl.textContent = "暂无巡检图片";
      snapPanel.appendChild(img);
      snapPanel.appendChild(capEl);
      insertAfter(mount, snapPanel, anchor);
    } else {
      let imgEl = $(snapImgId);
      let capEl = $(capId);
      if (!imgEl) {
        imgEl = document.createElement("img");
        imgEl.id = snapImgId;
        imgEl.className = "inspect-flight-img inspect-snapshot-img";
        imgEl.alt = "实时图传";
        snapPanel.insertBefore(imgEl, snapPanel.firstChild);
      }
      if (!capEl) {
        capEl = document.createElement("div");
        capEl.id = capId;
        capEl.className = "inspect-card-caption inspect-live-placeholder inspect-live-placeholder--show";
        capEl.textContent = "暂无巡检图片";
        snapPanel.appendChild(capEl);
      }
      snapPanel.classList.add("inspect-live-image-wrap");
    }

    ensureInspectCompactMetaBlock(mount, p);
    return Boolean($(snapPanelId) && $(snapImgId));
  }

  function ensureInspectImagePanelMounted(suffix) {
    return ensureInspectDualSlotsMounted(suffix);
  }

  /** 保证巡检快照 img 存在 */
  function ensureInspectFlightImgElement(suffix) {
    const p = suffix || "";
    const imgId = `inspectCardImg${p}`;
    let img = document.getElementById(imgId);
    if (img) return img;
    ensureInspectDualSlotsMounted(p);
    return document.getElementById(imgId);
  }

  function ensureInspectInfoCardShellIfMissing() {
    if ($("inspectInfoCard")) return;
    const wrap = document.createElement("aside");
    wrap.id = "inspectInfoCard";
    wrap.className = "inspect-fallback-card";
    wrap.setAttribute(
      "style",
      "position:fixed;bottom:16px;right:16px;z-index:10020;max-width:360px;max-height:85vh;overflow-x:hidden;overflow-y:auto;background:rgba(15,23,42,.96);color:#e5e7eb;padding:12px;border-radius:10px;border:1px solid rgba(148,163,184,.25);box-shadow:0 8px 24px rgba(0,0,0,.35);"
    );
    wrap.innerHTML = `
      <div class="inspect-panel-compact">
        <div class="inspect-section-h">飞行状态</div>
        <div id="inspectCardUavStatus" class="inspect-flight-status-line">待命</div>
        <div class="inspect-section-h inspect-section-h--sub">实时图传</div>
        <div id="inspectImagePanel" class="inspect-image-panel inspect-snapshot-panel inspect-live-image-wrap">
          <img id="inspectCardImg" class="inspect-flight-img inspect-snapshot-img" alt="实时图传" />
          <div id="inspectCardCaption" class="inspect-card-caption inspect-live-placeholder inspect-live-placeholder--show">暂无巡检图片</div>
        </div>
        <div class="inspect-compact-meta" data-inspect-compact-meta="main">
          <div class="inspect-compact-meta-row"><span class="inspect-compact-k">当前巡检点</span><span class="inspect-compact-v" id="inspectMetaCurrentPoint">—</span></div>
          <div class="inspect-compact-meta-row"><span class="inspect-compact-k">速度</span><span class="inspect-compact-v" id="inspectMetaSpeed">—</span></div>
          <div class="inspect-compact-meta-row"><span class="inspect-compact-k">高度</span><span class="inspect-compact-v" id="inspectMetaAlt">—</span></div>
          <div class="inspect-compact-meta-row"><span class="inspect-compact-k">风险等级</span><span class="inspect-compact-v" id="inspectMetaRisk">—</span></div>
          <div class="inspect-compact-meta-row"><span class="inspect-compact-k">进度</span><span class="inspect-compact-v" id="inspectMetaProgress">—</span></div>
        </div>
        <div id="inspectCardImgHint" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardId" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardSegment" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardCoord" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardType" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardPriority" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardProgress" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardTime" class="inspect-ui-hidden-legacy" aria-hidden="true"></div>
        <div id="inspectCardStatus" class="inspect-ui-hidden-legacy inspect-status idle" aria-hidden="true"></div>
        <ul id="inspectEventList" class="inspect-ui-hidden-legacy" aria-hidden="true"></ul>
      </div>`;
    document.body.appendChild(wrap);
  }

  function createInspectImgInRightPanel() {
    ensureInspectInfoCardShellIfMissing();
    ensureInspectDualSlotsMounted("");
    let img = document.getElementById("inspectCardImg");
    if (img) return img;
    const panel = document.getElementById("inspectImagePanel");
    if (!panel) return null;
    img = document.createElement("img");
    img.id = "inspectCardImg";
    img.className = "inspect-flight-img inspect-snapshot-img";
    img.alt = "巡检图像";
    panel.insertBefore(img, panel.firstChild);
    return img;
  }

  window.showInspectionImage = function (point, opts) {
    opts = opts || {};
    if (!point) return;
    const pointId = String(point.point_id || point.id || "").trim();
    if (!pointId) {
      console.warn("[inspection-image-ui] missing point_id", point);
      return;
    }
    const url =
      (typeof point.image_url === "string" && point.image_url.trim()) ||
      `/api/inspection-image/${pointId}.jpg`;

    if (typeof url === "string" && url.indexOf("/api/inspection-image/") === 0) {
      stopStreamLoop();
      PLAYBACK.streamFrozen = true;
      if (opts.logAs === "current_point" || PLAYBACK.status === "playing") {
        PLAYBACK.holdInspectSnapshot = true;
      }
    }

    const applyToImg = (imgEl, sfx) => {
      if (!imgEl) return;
      const cap = $(`inspectCardCaption${sfx}`);
      if (cap) {
        cap.textContent = "";
        cap.classList.remove("inspect-live-placeholder--show");
      }
      imgEl.classList.add("inspect-flight-img", "inspect-snapshot-img");
      imgEl.style.display = "block";
      imgEl.onerror = () => {
        console.warn("[inspection-image-ui] img error", imgEl.src);
        if (cap) {
          cap.textContent = "暂无巡检图片";
          cap.classList.add("inspect-live-placeholder--show");
        }
        imgEl.onerror = null;
        imgEl.removeAttribute("src");
        imgEl.style.display = "none";
      };
      imgEl.onload = () => {
        console.log("[inspection-image-ui] load ok", imgEl.src);
        if (cap) cap.classList.remove("inspect-live-placeholder--show");
      };
      imgEl.src = url;
    };

    const main = document.getElementById("inspectCardImg") || createInspectImgInRightPanel();
    applyToImg(main, "");
    const fsImg = document.getElementById("inspectCardImgFs");
    if (fsImg) applyToImg(fsImg, "Fs");

    if (opts.logAs === "current_point") {
      console.log(`[inspection-image-ui] current_point=${pointId} url=${url}`);
    } else {
      console.log(`[inspection-image-ui] ${opts.logAs || "show"} point_id=${pointId} url=${url}`);
    }
  };

  /** 保证右侧飞行状态卡片内存在巡检图容器；无宿主卡片时创建最小 aside */
  function ensureInspectFallbackCardIfMissing() {
    ensureInspectInfoCardShellIfMissing();
    ensureInspectImagePanelMounted("");
    ensureInspectImagePanelMounted("Fs");
  }

  function fillInspectCard(prefix, point, index, total, opts) {
    if (!ENABLE_INSPECTION_IMAGE) return;
    ensureInspectFallbackCardIfMissing();
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

    const rawPid = String(point.point_id || point.id || "—").trim();
    set("inspectMetaCurrentPoint", rawPid);
    set(
      "inspectMetaProgress",
      `${Math.round((index / Math.max(total, 1)) * 100)}%`
    );
    syncInspectCompactMeta(p);

    if (!opts?.skipImage && typeof window.showInspectionImage === "function") {
      window.showInspectionImage(point, {
        suffix: p,
        quiet: !!opts?.silentLog,
        logAs: "point_id",
      });
    }

    const hint = $(`inspectCardImgHint${p}`);
    if (hint) {
      const hasUrl = typeof point?.image_url === "string" && point.image_url.trim();
      hint.textContent =
        hasUrl && point.image_available === true ? "巡检图像采集成功" : "暂无巡检图片";
    }
    const st = $(`inspectCardStatus${p}`);
    if (st) {
      st.textContent = "正在拍照";
      st.className = "inspect-status shooting";
    }
  }

  /**
   * Dashboard：点击地图上的巡检点 marker 时更新巡检卡片与图像（不进入播放时间轴动画）
   */
  window.showInspectCardForDashboardPoint = function (point, index, total, opts = {}) {
    if (!ENABLE_INSPECTION_IMAGE) return;
    if (!point) return;
    ensureInspectCardVisible();
    if (!opts.skipImageRefresh && typeof window.showInspectionImage === "function") {
      window.showInspectionImage(point, { logAs: "point_id", quiet: true });
    }
    fillInspectCard("", point, index, total, { silentLog: true, skipImage: true });
    fillInspectCard("Fs", point, index, total, { silentLog: true, skipImage: true });
    ["", "Fs"].forEach((pfx) => {
      const hint = $(`inspectCardImgHint${pfx}`);
      if (hint) {
        const hasUrl = typeof point?.image_url === "string" && point.image_url.trim();
        hint.textContent =
          hasUrl && point.image_available === true
            ? "巡检采集图像（地图选点）"
            : "暂无巡检图片";
      }
      const st = $(`inspectCardStatus${pfx}`);
      if (st) {
        st.textContent = "地图浏览";
        st.className = "inspect-status done";
      }
      const uav = $(`inspectCardUavStatus${pfx}`);
      if (uav) uav.textContent = "已选中巡检点";
    });
  };

  function showInspectCard(point, index, total, cardOpts = {}) {
    if (!ENABLE_INSPECTION_IMAGE) return;
    const sp = resolveInspectionCardImage(point);
    console.log("[inspection-image-ui]", `current_point=${sp.pointId}`, `url=${sp.url}`);
    ensureInspectCardVisible();
    fillInspectCard("", point, index, total, { silentLog: true, skipImage: true });
    fillInspectCard("Fs", point, index, total, { silentLog: true, skipImage: true });
    if (!cardOpts.skipInspectionImage && typeof window.showInspectionImage === "function") {
      window.showInspectionImage(point, { quiet: true, logAs: "current_point" });
    }
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
      telemetry: PLAYBACK.telemetry ? { ...PLAYBACK.telemetry } : null,
      currentEvent: PLAYBACK.timeline[PLAYBACK.index] || null,
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
    PLAYBACK.running = running;
    PLAYBACK.paused = paused;
    PLAYBACK.t = PLAYBACK.index;
    const hasTimeline = PLAYBACK.timeline.length > 0;
    const startBtn = $("playbackStartBtn");
    if (startBtn) {
      if (running && !paused) {
        startBtn.textContent = "巡检执行";
      } else {
        startBtn.textContent = "开始巡检";
      }
    }
    $("playbackStartBtn")?.toggleAttribute("disabled", running);
    $("playbackPauseBtn")?.toggleAttribute("disabled", !running);
    $("playbackResumeBtn")?.toggleAttribute("disabled", !paused);
    $("playbackResetBtn")?.toggleAttribute("disabled", PLAYBACK.status === "idle" && !hasTimeline);
    const pct = hasTimeline
      ? Math.min(100, Math.round((PLAYBACK.index / PLAYBACK.timeline.length) * 100))
      : 0;
    const stateText = STATE_TEXT[PLAYBACK.missionState] || "执行中";
    const ev = PLAYBACK.timeline[PLAYBACK.index];
    const segLabel =
      ev?.label ||
      (ev?.segment_id
        ? `区段 ${String(ev.segment_id).replace(/^seg_/, "")}`
        : ev?.type === "inspect"
          ? "巡检点"
          : "区段 —");
    if ($("timelinePercent")) $("timelinePercent").textContent = `${pct}%`;
    if ($("timelineSegment")) $("timelineSegment").textContent = segLabel;
    if ($("timelineFill")) $("timelineFill").style.width = `${pct}%`;
    if ($("timelineScrubber")) {
      $("timelineScrubber").value = String(pct);
      $("timelineScrubber").max = "100";
    }
    if ($("headerSpeedBadge")) {
      $("headerSpeedBadge").textContent = `${PLAYBACK.speed || 1}x`;
    }
    const prog = $("playbackProgress");
    if (!prog) {
      if (ENABLE_INSPECTION_IMAGE) {
        syncInspectCompactMeta("");
        syncInspectCompactMeta("Fs");
      }
      return;
    }
    if (!hasTimeline) {
      prog.textContent = "待命";
      if (typeof window.renderSystemStatus === "function") window.renderSystemStatus();
      if (ENABLE_INSPECTION_IMAGE) {
        syncInspectCompactMeta("");
        syncInspectCompactMeta("Fs");
      }
      return;
    }
    prog.textContent = `${pct}% · ${stateText}`;
    if (typeof window.renderSystemStatus === "function") window.renderSystemStatus();
    if (typeof window.renderStats === "function") {
      const mission = window.MissionStore?.getCurrentMission?.() || window.state?.lastResult;
      if (mission?.statistics) window.renderStats(mission.statistics);
    }
    if (ENABLE_INSPECTION_IMAGE) {
      syncInspectCompactMeta("");
      syncInspectCompactMeta("Fs");
    }
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
    if (!PLAYBACK.holdInspectSnapshot) {
      PLAYBACK.streamFrozen = false;
      setMissionState("LEAVING");
      updateTelemetryFromPlayback("LEAVING");
      setStreamImage(
        CRUISE_STREAM_IMAGES[PLAYBACK.streamIndex % CRUISE_STREAM_IMAGES.length],
        "离开巡检点 · 恢复巡航图传"
      );
      const toCruise = setTimeout(() => {
        if (PLAYBACK.status !== "playing") return;
        setMissionState("CRUISING");
        updateTelemetryFromPlayback("CRUISING");
      }, 450);
      PLAYBACK.phaseTimers.push(toCruise);
      scheduleStreamLoop();
    } else {
      setMissionState("LEAVING");
      updateTelemetryFromPlayback("LEAVING");
      const toCruise = setTimeout(() => {
        if (PLAYBACK.status !== "playing") return;
        setMissionState("EXECUTING");
        updateTelemetryFromPlayback("EXECUTING");
      }, 450);
      PLAYBACK.phaseTimers.push(toCruise);
    }
    updateAllPlotPlayback();
    if (PLAYBACK.status === "playing") tickPlayback();
  }

  function resolveInspectEdgeIdForSegment(segKey) {
    const mission = getMissionForPlayback();
    const segs = mission?.segments || [];
    const hit = segs.find((s) => String(s.segment_id || "") === String(segKey || ""));
    if (hit && hit.edge_id != null) {
      return String(hit.edge_id).replace(/[+-]$/, "");
    }
    return "";
  }

  function activateInspectSegmentForAllPoints(segKey, evEdgeId) {
    let eid = String(evEdgeId || "").replace(/[+-]$/, "");
    if (!eid) eid = resolveInspectEdgeIdForSegment(segKey);
    if (!eid || !segKey) return;
    if (PLAYBACK.activatedInspectSegments.has(segKey)) return;
    PLAYBACK.activatedInspectSegments.add(segKey);
    const mission = getMissionForPlayback();
    (mission?.inspection_points || []).forEach((p) => {
      const pe = p.edge_id != null ? String(p.edge_id).replace(/[+-]$/, "") : "";
      if (pe && pe === eid) {
        const pid = p.id || p.point_id;
        if (pid) {
          PLAYBACK.visitedIds.add(pid);
          PLAYBACK.visitedCoords.set(pid, { x: p.x, y: p.y });
        }
      }
    });
  }

  function markAllInspectionPointsVisited() {
    const mission = getMissionForPlayback();
    (mission?.inspection_points || []).forEach((p) => {
      const pid = p.id || p.point_id;
      if (pid) {
        PLAYBACK.visitedIds.add(pid);
        PLAYBACK.visitedCoords.set(pid, { x: p.x, y: p.y });
      }
    });
  }

  function tickPlayback() {
    if (PLAYBACK.status !== "playing") return;

    if (PLAYBACK.index >= PLAYBACK.timeline.length) {
      PLAYBACK.status = "finished";
      PLAYBACK.running = false;
      PLAYBACK.paused = false;
      notifyPlaybackPhase("finished");
      stopTimer();
      stopStreamLoop();
      PLAYBACK.holdInspectSnapshot = false;
      PLAYBACK.streamFrozen = false;
      markAllInspectionPointsVisited();
      setMissionState("COMPLETED");
      updateTelemetryFromPlayback("COMPLETED");
      pushEvent("任务执行完成");
      setPlaybackUi();
      if (ENABLE_INSPECTION_IMAGE) {
        ["", "Fs"].forEach((p) => {
          const st = $(`inspectCardStatus${p}`);
          if (st) {
            st.textContent = "任务完成";
            st.className = "inspect-status done";
          }
        });
      }
      updateAllPlotPlayback();
      return;
    }

    const ev = PLAYBACK.timeline[PLAYBACK.index];
    PLAYBACK.index += 1;

    if (ev.type === "inspect") {
      activateInspectSegmentForAllPoints(ev.segment_id || "", ev.edge_id || "");
      const inspPoint = ev.point || ev.inspection_point;
      PLAYBACK.uavPos = { x: inspPoint.x, y: inspPoint.y };
      PLAYBACK.currentPoint = { x: inspPoint.x, y: inspPoint.y };
      PLAYBACK.currentPointId = inspPoint.id || inspPoint.point_id;
      if (typeof window.showInspectionImage === "function") {
        window.showInspectionImage(inspPoint, { logAs: "current_point" });
      }
      updateAllPlotPlayback();
      if (ENABLE_INSPECTION_IMAGE) {
        showInspectCard(inspPoint, ev.inspect_index, PLAYBACK.totalInspectCount, {
          skipInspectionImage: true,
        });
      }
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
      setMissionState("EXECUTING");
      updateTelemetryFromPlayback("EXECUTING");
      updateAllPlotPlayback();
      setPlaybackUi();
      scheduleNext(MOVE_FRAME_MS / getSpeed());
      return;
    }

    setPlaybackUi();
    scheduleNext(8 / getSpeed());
  }

  function mergeInspectionStopsIntoPlotlyMoves(moveFrames, result) {
    const pts = Array.isArray(result?.inspection_points) ? result.inspection_points : [];
    if (!pts.length || !moveFrames.length) return moveFrames;

    const stalls = [];
    pts.forEach((rawPt, seq) => {
      const x = Number(rawPt.x);
      const y = Number(rawPt.y);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      let bestI = 0;
      let bestD = Infinity;
      for (let i = 0; i < moveFrames.length; i += 1) {
        const d = Math.hypot(moveFrames[i].x - x, moveFrames[i].y - y);
        if (d < bestD) {
          bestD = d;
          bestI = i;
        }
      }
      stalls.push({ at: bestI, bestD, point: { ...rawPt, x, y }, seq });
    });
    stalls.sort((a, b) => a.at - b.at || a.seq - b.seq);

    const byAt = new Map();
    stalls.forEach((st) => {
      const list = byAt.get(st.at) || [];
      list.push(st);
      byAt.set(st.at, list);
    });
    const sortedAts = [...byAt.keys()].sort((a, b) => a - b);

    const out = [];
    let mi = 0;
    let inspectSeq = 0;
    for (let ai = 0; ai < sortedAts.length; ai += 1) {
      const at = sortedAts[ai];
      while (mi < moveFrames.length && mi <= at) {
        out.push(moveFrames[mi]);
        mi += 1;
      }
      const group = byAt.get(at) || [];
      group.sort((a, b) => a.seq - b.seq);
      for (let gi = 0; gi < group.length; gi += 1) {
        const st = group[gi];
        inspectSeq += 1;
        out.push({
          type: "inspect",
          point: st.point,
          dwell_ms: getDwellMs(),
          inspect_index: inspectSeq,
          edge_id: st.point.edge_id,
          segment_id: st.point.segment_id,
        });
      }
    }
    while (mi < moveFrames.length) {
      out.push(moveFrames[mi]);
      mi += 1;
    }
    return out;
  }

  function buildTimelineFromResult(result) {
    const candidates = [
      document.getElementById("mapPlot"),
      document.getElementById("mapPlotFs"),
      document.querySelector(".js-plotly-plot"),
    ].filter(Boolean);
    let plot = null;
    for (let c = 0; c < candidates.length; c += 1) {
      const el = candidates[c];
      if (el && el.data && el.data.some((t) => t && t.name === "巡检路径")) {
        plot = el;
        break;
      }
    }
    if (!plot) {
      plot =
        document.getElementById("mapPlot") ||
        document.getElementById("mapPlotFs") ||
        document.querySelector(".js-plotly-plot");
    }

    if (!plot || !plot.data) {
      console.warn("[timeline] no plot data");
      return [];
    }

    const trace = plot.data.find((t) => t && t.name === "巡检路径");
    if (!trace || !Array.isArray(trace.x) || !Array.isArray(trace.y)) {
      console.warn("[timeline] no 巡检路径 trace", plot.data.map((t) => t && t.name));
      return [];
    }

    const plotMoves = [];
    for (let i = 0; i < trace.x.length; i += 1) {
      const x = Number(trace.x[i]);
      const y = Number(trace.y[i]);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      plotMoves.push({
        type: "move",
        x,
        y,
        label: `巡检路径 ${i + 1}`,
        progress: i / Math.max(1, trace.x.length - 1),
      });
    }

    if (!plotMoves.length) return [];

    PLAYBACK.dwellMs = getDwellMs();
    PLAYBACK.speed = getSpeed();
    return mergeInspectionStopsIntoPlotlyMoves(plotMoves, result);
  }

  window.resetPlayback = function (result) {
    stopTimer();
    stopStreamLoop();
    clearPhaseTimers();
    PLAYBACK.status = "idle";
    PLAYBACK.running = false;
    PLAYBACK.paused = false;
    PLAYBACK.t = 0;
    PLAYBACK.index = 0;
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;
    PLAYBACK.visitedIds = new Set();
    PLAYBACK.visitedCoords = new Map();
    PLAYBACK.activatedInspectSegments = new Set();
    PLAYBACK.timeline = [];
    PLAYBACK.totalInspectCount = 0;
    PLAYBACK.startTimestamp = 0;
    PLAYBACK.streamFrozen = false;
    PLAYBACK.holdInspectSnapshot = false;
    clearInspectCard();

    if (result) {
      PLAYBACK.timeline = buildTimelineFromResult(result) || [];
      PLAYBACK.totalInspectCount = PLAYBACK.timeline.filter((e) => e.type === "inspect").length;
      PLAYBACK.dwellMs = getDwellMs();
      PLAYBACK.speed = getSpeed();
      if (ENABLE_INSPECTION_IMAGE) {
        PLAYBACK.streamImages = buildStreamPool(result);
      } else {
        PLAYBACK.streamImages = [];
      }
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

  window.startInspectionPlayback = function () {
    console.log("[startInspectionPlayback] entered");

    const result = getMissionForPlayback();
    if (!result) {
      console.warn("[startInspectionPlayback] no mission");
      return;
    }

    const timeline = buildTimelineFromResult(result);
    console.log("[playback timeline]", timeline.length, timeline[0], timeline[timeline.length - 1]);

    if (!timeline || !timeline.length) {
      console.warn("[startInspectionPlayback] empty timeline");
      const plot =
        document.getElementById("mapPlot") || document.querySelector(".js-plotly-plot");
      if (plot && plot.data) {
        console.warn("[timeline] trace names:", plot.data.map((t) => t && t.name));
      }
      return;
    }

    stopTimer();
    stopStreamLoop();
    clearPhaseTimers();

    PLAYBACK.timeline = timeline;
    PLAYBACK.index = 0;
    PLAYBACK.t = 0;
    PLAYBACK.running = true;
    PLAYBACK.paused = false;
    PLAYBACK.status = "playing";
    PLAYBACK.speed = getSpeed();
    PLAYBACK.startTimestamp = Date.now();
    PLAYBACK.streamFrozen = false;
    PLAYBACK.holdInspectSnapshot = false;
    PLAYBACK.totalInspectCount = timeline.filter((e) => e.type === "inspect").length;
    PLAYBACK.visitedIds = new Set();
    PLAYBACK.visitedCoords = new Map();
    PLAYBACK.activatedInspectSegments = new Set();
    PLAYBACK.uavPos =
      timeline.length > 0 ? { x: timeline[0].x, y: timeline[0].y } : null;
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;

    notifyPlaybackPhase("playing");
    setMissionState("EXECUTING");
    clearInspectCard();
    pushEvent("任务开始执行");
    updateTelemetryFromPlayback("EXECUTING");
    setPlaybackUi();
    updateAllPlotPlayback();
    tickPlayback();
  };

  window.pauseInspectionPlayback = function () {
    if (PLAYBACK.status !== "playing") return;
    stopTimer();
    stopStreamLoop();
    clearPhaseTimers();
    PLAYBACK.status = "paused";
    PLAYBACK.running = false;
    PLAYBACK.paused = true;
    notifyPlaybackPhase("paused");
    setMissionState("PAUSED");
    updateTelemetryFromPlayback("PAUSED");
    pushEvent("任务已暂停");
    setPlaybackUi();
  };

  window.resumeInspectionPlayback = function () {
    if (PLAYBACK.status !== "paused") return;
    PLAYBACK.status = "playing";
    PLAYBACK.running = true;
    PLAYBACK.paused = false;
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

  document.addEventListener("click", (e) => {
    const btn = e.target && e.target.closest && e.target.closest("#playbackStartBtn");
    if (!btn) return;
    console.log("[play-click] DOM handler fired");
    window.startInspectionPlayback();
  });

  document.addEventListener("DOMContentLoaded", () => {
    $("playbackPauseBtn")?.addEventListener("click", () => window.pauseInspectionPlayback());
    $("playbackResumeBtn")?.addEventListener("click", () => window.resumeInspectionPlayback());
    $("playbackResetBtn")?.addEventListener("click", () => window.resetInspectionPlayback());
    clearInspectCard();
    setPlaybackUi();
  });
})();
