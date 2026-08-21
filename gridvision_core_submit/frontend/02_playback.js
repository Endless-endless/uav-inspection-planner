/**
 * GridVision 毕设答辩核心代码摘录 — 巡检播放
 * 源文件: web/static/playback.js
 * 行号: 1345-1434, 1615-1680
 * 说明: 摘录供答辩展示，需与原工程 static 资源配合运行。
 */

/* ===== playback.js L1345-L1434 ===== */
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
      printPlaybackCheck();
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
    const frameIndex = PLAYBACK.index;
    PLAYBACK.index += 1;
    PLAYBACK.uavPos = { x: ev.x, y: ev.y };
    PLAYBACK.currentRouteIndex = ev.routeIndex != null ? ev.routeIndex : frameIndex;
    PLAYBACK.currentPoint = null;
    PLAYBACK.currentPointId = null;
    checkInspectTriggersAtFrame(ev, frameIndex);
    const tickPointIds = getFramePointIds(ev);
    if (frameIndex % TICK_LOG_EVERY === 0 || tickPointIds.length) {
      console.log(
        `[tick] frame=${frameIndex} segment=${ev.segment_id || ""} point=${tickPointIds.join(",")}`
      );
    }
    setMissionState("EXECUTING");
    updateTelemetryFromPlayback("EXECUTING");
    updateAllPlotPlayback();
    setPlaybackUi();
    scheduleNext(MOVE_FRAME_MS / getSpeed());
  }

  function getFramePointIds(frame) {
    if (Array.isArray(frame.routePointIds)) return frame.routePointIds;
    if (frame.routePointId) return [frame.routePointId];
    return [];
  }

  function triggerInspectionPoint(pointId) {
    if (!pointId || PLAYBACK.visitedIds.has(pointId)) return;

    const points = PLAYBACK.inspectPlotPoints || [];
    const pt = points.find((p) => (p.point_id || p.id) === pointId);
    if (!pt) return;

    PLAYBACK.visitedIds.add(pointId);
    PLAYBACK.currentPointId = pointId;
    PLAYBACK.currentPoint = pt;
    if (typeof window.showInspectionImage === "function") {
      window.showInspectionImage(pt, { logAs: "route-point" });
    }
    if (ENABLE_INSPECTION_IMAGE) {
      const total = PLAYBACK.totalInspectCount || points.length;
      showInspectCard(
        pt,
        pt.orderIndex >= 0 ? pt.orderIndex + 1 : PLAYBACK.visitedIds.size,
        total,
        { skipInspectionImage: true }
      );
    }
  }

  function checkInspectTriggersAtFrame(ev, frameIndex) {
    const ids = getFramePointIds(ev);
    ids.forEach((pointId) => {
      triggerInspectionPoint(pointId);
    });
  }

  function getPlotlyMapElement() {

/* ===== playback.js L1615-L1680 ===== */
  window.startInspectionPlayback = function () {
    if (PLAYBACK.status === "playing" || PLAYBACK._startLock) return;
    PLAYBACK._startLock = true;

    stopTimer();
    stopStreamLoop();
    clearPhaseTimers();

    PLAYBACK.timeline = [];
    PLAYBACK.index = 0;
    PLAYBACK.t = 0;
    PLAYBACK.running = false;
    PLAYBACK.paused = false;
    PLAYBACK.inspectPlotPoints = [];

    const result = getMissionForPlayback();
    if (!result) {
      console.warn("[startInspectionPlayback] no mission");
      PLAYBACK._startLock = false;
      return;
    }

    const timeline = buildTimelineFromMissionRouteSequence();
    if (!timeline || !timeline.length) {
      PLAYBACK._startLock = false;
      return;
    }

    PLAYBACK.timeline = timeline;
    PLAYBACK.inspectPlotPoints =
      window.__INSPECTION_POINT_BINDINGS ||
      window.__DRAWN_ROUTE_DEBUG?.inspect_points ||
      [];
    PLAYBACK.running = true;
    PLAYBACK.paused = false;
    PLAYBACK.status = "playing";
    PLAYBACK.speed = getSpeed();
    PLAYBACK.dwellMs = getDwellMs();
    PLAYBACK.startTimestamp = Date.now();
    PLAYBACK.streamFrozen = false;
    PLAYBACK.holdInspectSnapshot = false;
    PLAYBACK.totalInspectCount = PLAYBACK.inspectPlotPoints.length;
    PLAYBACK.visitedIds = new Set();
    PLAYBACK.visitedCoords = new Map();
    PLAYBACK.activatedInspectSegments = new Set();
    PLAYBACK.uavPos = { x: timeline[0].x, y: timeline[0].y };
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
    PLAYBACK._startLock = false;
  };

  window.pauseInspectionPlayback = function () {
    if (PLAYBACK.status !== "playing") return;
    stopTimer();
    stopStreamLoop();
    clearPhaseTimers();
    PLAYBACK.status = "paused";
