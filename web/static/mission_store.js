/** Minimal mission state for Dashboard (image pipeline demo). */
(function () {
  "use strict";

  const VALID_SOURCES = new Set(["spacing", "image"]);
  const SOURCE_ALIASES = {
    image_points: "image",
    manual_json: "image",
    real_satellite_manual: "image",
    detect: "image",
  };

  const MissionStore = {
    current: null,
    version: 0,
    missionListeners: new Set(),

    normalizeSource(source) {
      const raw = String(source || "spacing").trim().toLowerCase();
      if (VALID_SOURCES.has(raw)) return raw;
      return SOURCE_ALIASES[raw] || "spacing";
    },

    normalizeDashboard(dashboard) {
      if (!dashboard || typeof dashboard !== "object") return dashboard;
      const out = { ...dashboard };
      out.metadata = { ...(dashboard.metadata || {}) };
      out.metadata.inspection_point_source = MissionStore.normalizeSource(
        out.metadata.inspection_point_source
      );
      return out;
    },

    _snapshot(dashboard, patch = {}) {
      const normalized = MissionStore.normalizeDashboard(dashboard);
      const meta = normalized.metadata || {};
      return {
        source: MissionStore.normalizeSource(meta.inspection_point_source),
        inspection_points: Array.isArray(normalized.inspection_points)
          ? normalized.inspection_points.map((p) => ({ ...p }))
          : [],
        segments: Array.isArray(normalized.segments)
          ? normalized.segments.map((s) => ({
              ...s,
              geometry_2d: [...(s.geometry_2d || [])],
            }))
          : [],
        version: MissionStore.version,
        last_update_kind: patch.last_update_kind || "load",
        dashboard: normalized,
        metadata: meta,
        markers: normalized.markers ? { ...normalized.markers } : {},
        statistics: normalized.statistics ? { ...normalized.statistics } : {},
        visit_order: normalized.visit_order,
        bounds: normalized.bounds,
        map_background: normalized.map_background,
      };
    },

    _emitMission() {
      MissionStore.missionListeners.forEach((fn) => {
        try {
          fn(MissionStore.getMission());
        } catch (err) {
          console.warn("[MissionStore] listener error", err);
        }
      });
    },

    subscribeMission(fn) {
      if (typeof fn !== "function") return () => {};
      MissionStore.missionListeners.add(fn);
      return () => MissionStore.missionListeners.delete(fn);
    },

    subscribePhase() {
      return () => {};
    },

    getMission() {
      return MissionStore.current?.dashboard || null;
    },

    getCurrentMission() {
      return MissionStore.getMission();
    },

    getSource() {
      return MissionStore.current?.source || "spacing";
    },

    isImageSource() {
      return MissionStore.getSource() === "image";
    },

    getInspectionPoints() {
      return MissionStore.current?.inspection_points || [];
    },

    clear() {
      MissionStore.current = null;
      MissionStore._emitMission();
    },

    loadFromDashboard(dashboard, patch = {}) {
      MissionStore.version += 1;
      MissionStore.current = MissionStore._snapshot(dashboard, patch);
      MissionStore._emitMission();
      return MissionStore.getMission();
    },

    updateMarkers(markers) {
      if (!MissionStore.current) return;
      MissionStore.current.markers = { ...MissionStore.current.markers, ...markers };
      MissionStore.current.dashboard = {
        ...MissionStore.current.dashboard,
        markers: MissionStore.current.markers,
      };
      MissionStore._emitMission();
    },
  };

  window.MissionStore = MissionStore;
  window.normalizeInspectionPointSource = MissionStore.normalizeSource.bind(MissionStore);
})();
