/**
 * GridVision 毕设答辩核心代码摘录 — 图传切换
 * 源文件: web/static/playback.js
 * 行号: 496-551
 * 说明: 摘录供答辩展示，需与原工程 static 资源配合运行。
 */

/* ===== playback.js L496-L551 ===== */
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
