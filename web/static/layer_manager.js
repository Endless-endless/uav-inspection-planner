/**
 * Visualization Layer Manager — permanent vs transient Plotly layers.
 */
(function () {
  "use strict";

  const LayerIds = {
    L0_BACKGROUND: "L0_background",
    L1_TOPOLOGY: "L1_topology",
    L2_INSPECTION_POINTS: "L2_inspection_points",
    L3_WEATHER: "L3_weather",
    T1_ADAPTIVE_FLASH: "T1_adaptive_flash",
    T2_PREDICTIVE_WARNING: "T2_predictive_warning",
    T3_AB_EXPERIMENT: "T3_ab_experiment",
    T4_PLAYBACK: "T4_playback",
  };

  const TRANSIENT_DEFAULTS = {
    [LayerIds.T1_ADAPTIVE_FLASH]: 1800,
    [LayerIds.T2_PREDICTIVE_WARNING]: 6000,
    [LayerIds.T3_AB_EXPERIMENT]: null,
    [LayerIds.T4_PLAYBACK]: null,
  };

  const LayerManager = {
    timers: {},
    layers: {},

    isTransient(layerId) {
      return String(layerId || "").startsWith("T");
    },

    setLayer(layerId, traces, options = {}) {
      LayerManager.layers[layerId] = Array.isArray(traces) ? traces : [];
      if (LayerManager.isTransient(layerId)) {
        LayerManager._scheduleAutoRemove(layerId, options.durationMs, options.onExpire);
      }
      return LayerManager.layers[layerId];
    },

    getLayer(layerId) {
      return LayerManager.layers[layerId] || [];
    },

    clearLayer(layerId) {
      delete LayerManager.layers[layerId];
      if (LayerManager.timers[layerId]) {
        clearTimeout(LayerManager.timers[layerId]);
        delete LayerManager.timers[layerId];
      }
    },

    clearAllTransient() {
      Object.keys(LayerManager.layers).forEach((id) => {
        if (LayerManager.isTransient(id)) LayerManager.clearLayer(id);
      });
    },

    _scheduleAutoRemove(layerId, durationMs, onExpire) {
      if (LayerManager.timers[layerId]) {
        clearTimeout(LayerManager.timers[layerId]);
      }
      const ms = durationMs ?? TRANSIENT_DEFAULTS[layerId];
      if (!ms || ms <= 0) return;
      LayerManager.timers[layerId] = window.setTimeout(() => {
        LayerManager.clearLayer(layerId);
        if (typeof onExpire === "function") onExpire(layerId);
      }, ms);
    },

    tracePredicate(layerId) {
      const rules = {
        [LayerIds.L3_WEATHER]: (t) =>
          String(t.name || "").startsWith("天气区 ") || t.name === "天气标签",
        [LayerIds.T1_ADAPTIVE_FLASH]: (t) => t.name === "自适应重规划路径",
        [LayerIds.T3_AB_EXPERIMENT]: (t) =>
          t.legendgroup === "expab" || t.name === "A 天气关闭" || t.name === "B 天气开启",
        [LayerIds.T4_PLAYBACK]: (t) =>
          ["UAV", "UAV 光晕", "当前巡检点", "当前巡检点光晕", "已巡检点"].includes(t.name),
        [LayerIds.L1_TOPOLOGY]: (t) => t.name === "巡检路径" || t.name === "连接路径",
        [LayerIds.L2_INSPECTION_POINTS]: (t) =>
          t.name === "巡检点" || t.name === "巡检点光晕" || t.name === "已巡检点" || t.name === "当前巡检点",
      };
      return rules[layerId] || (() => false);
    },

    replaceLayerTraces(plotId, layerId, traces, plotState) {
      if (!plotState?.plotReady?.[plotId]) return;
      const el = document.getElementById(plotId);
      if (!el?.data) return;
      const predicate = LayerManager.tracePredicate(layerId);
      const keep = el.data.filter((t) => !predicate(t));
      const mission =
        typeof getCurrentMission === "function" ? getCurrentMission() : null;
      const layout =
        typeof applyFixedSatelliteToLayout === "function"
          ? applyFixedSatelliteToLayout(el.layout, mission)
          : {
              ...(el.layout || {}),
              images:
                typeof getFixedSatelliteLayoutImages === "function"
                  ? getFixedSatelliteLayoutImages(mission)
                  : el.layout?.images || [],
            };
      Plotly.react(
        plotId,
        keep.concat(traces || []),
        layout,
        { responsive: plotId !== "mapPlotFs", displayModeBar: true, scrollZoom: true }
      );
    },

    refreshTransientPlot(plotId, plotState, hooks = {}) {
      ["mapPlot", "mapPlotFs"].forEach((id) => {
        if (plotId && plotId !== id) return;
        LayerManager.replaceLayerTraces(
          id,
          LayerIds.L3_WEATHER,
          typeof hooks.buildWeather === "function" ? hooks.buildWeather() : [],
          plotState
        );
        LayerManager.replaceLayerTraces(
          id,
          LayerIds.T1_ADAPTIVE_FLASH,
          LayerManager.getLayer(LayerIds.T1_ADAPTIVE_FLASH),
          plotState
        );
        LayerManager.replaceLayerTraces(
          id,
          LayerIds.T3_AB_EXPERIMENT,
          LayerManager.getLayer(LayerIds.T3_AB_EXPERIMENT),
          plotState
        );
      });
    },
  };

  window.LayerIds = LayerIds;
  window.LayerManager = LayerManager;
})();
