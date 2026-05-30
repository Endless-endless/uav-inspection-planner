/**
 * Mission System Governance — MissionStore + AppPhase
 * Single source of truth for mission state on the dashboard.
 */
(function () {
  "use strict";

  const VALID_SOURCES = new Set(["spacing", "image"]);
  const SOURCE_ALIASES = {
    image_points: "image",
    "image-point": "image",
    image_detected: "image",
    "image-point-source": "image",
    manual_json: "image",
    "manual-json": "image",
    real_satellite_manual: "image",
    auto: "spacing",
    sampled: "spacing",
    detect: "image",
    mixed: "spacing",
  };

  const AppPhase = {
    IDLE: "idle",
    PLANNING: "planning",
    MISSION_READY: "mission_ready",
    PLAYING: "playing",
    PAUSED: "paused",
    ADAPTIVE_WARNING: "adaptive_warning",
    REPLANNING: "replanning",
    FINISHED: "finished",
  };

  const MissionStore = {
    current: null,
    version: 0,
    phaseListeners: new Set(),
    missionListeners: new Set(),

    normalizeSource(source) {
      const raw = String(source || "spacing").trim().toLowerCase();
      if (VALID_SOURCES.has(raw)) return raw;
      if (SOURCE_ALIASES[raw]) return SOURCE_ALIASES[raw];
      return "spacing";
    },

    normalizeDashboard(dashboard) {
      if (!dashboard || typeof dashboard !== "object") return dashboard;
      const out = { ...dashboard };
      out.metadata = { ...(dashboard.metadata || {}) };
      out.metadata.inspection_point_source = MissionStore.normalizeSource(
        out.metadata.inspection_point_source
          || out.metadata.mission_metadata?.inspection_point_source
      );
      if (out.metadata.mission_metadata) {
        out.metadata.mission_metadata = {
          ...out.metadata.mission_metadata,
          inspection_point_source: out.metadata.inspection_point_source,
        };
      }
      return out;
    },

    _snapshot(dashboard, patch = {}) {
      const normalized = MissionStore.normalizeDashboard(dashboard);
      const meta = normalized.metadata || {};
      return {
        source: MissionStore.normalizeSource(meta.inspection_point_source),
        mission_json_path:
          patch.mission_json_path
          || meta.mission_json_path
          || normalized.output_files?.mission_snapshot
          || normalized.output_files?.source
          || null,
        inspection_points: Array.isArray(normalized.inspection_points)
          ? normalized.inspection_points.map((p) => ({ ...p }))
          : [],
        segments: Array.isArray(normalized.segments)
          ? normalized.segments.map((s) => ({
              ...s,
              geometry_2d: [...(s.geometry_2d || [])],
            }))
          : [],
        weather_context: patch.weather_context || MissionStore.current?.weather_context || null,
        version: MissionStore.version,
        last_replan_time: patch.last_replan_time || MissionStore.current?.last_replan_time || null,
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

    subscribePhase(fn) {
      if (typeof fn !== "function") return () => {};
      MissionStore.phaseListeners.add(fn);
      return () => MissionStore.phaseListeners.delete(fn);
    },

    getMission() {
      return MissionStore.current?.dashboard || null;
    },

    getSnapshot() {
      return MissionStore.current;
    },

    getSource() {
      return MissionStore.current?.source || "spacing";
    },

    /** @deprecated 使用 getSource() */
    getMissionSource() {
      return MissionStore.getSource();
    },

    /** 与 metadata.inspection_point_source 一致，仅 spacing | image */
    getInspectionPointSource() {
      return MissionStore.getSource();
    },

    getCurrentMission() {
      return MissionStore.getMission();
    },

    getCurrentSegments() {
      return MissionStore.getSegments();
    },

    getCurrentInspectionPoints() {
      return MissionStore.getInspectionPoints();
    },

    getSegments() {
      return MissionStore.current?.segments || [];
    },

    getInspectionPoints() {
      return MissionStore.current?.inspection_points || [];
    },

    isImageSource() {
      return MissionStore.getSource() === "image";
    },

    clear() {
      MissionStore.current = null;
      MissionStore._emitMission();
      AppPhaseManager.setPhase(AppPhase.IDLE, { reason: "mission_cleared" });
    },

    loadFromDashboard(dashboard, options = {}) {
      MissionStore.version += 1;
      const kind = options.kind || "initial";
      MissionStore.current = MissionStore._snapshot(dashboard, {
        mission_json_path: options.mission_json_path,
        weather_context: options.weather_context,
        last_replan_time: kind === "server_replan" ? Date.now() : null,
        last_update_kind: kind,
      });
      MissionStore._emitMission();
      AppPhaseManager.setPhase(AppPhase.MISSION_READY, { reason: kind });
      return MissionStore.getMission();
    },

    mergeReplanWithCurrentDashboard(current, replan) {
      if (!replan) return current;
      if (!current) return replan;

      const c = current;
      const r = replan;
      const cm = { ...(c.metadata || {}) };
      const rm = { ...(r.metadata || {}) };

      function cloneVal(v) {
        if (v == null) return v;
        try {
          if (typeof structuredClone === "function") return structuredClone(v);
        } catch (_) {
          /* fall through */
        }
        try {
          return JSON.parse(JSON.stringify(v));
        } catch (_) {
          return v;
        }
      }

      const metadata = {
        ...rm,
        map_image: cm.map_image,
        display_map_image: cm.display_map_image,
        clean_map_image: cm.clean_map_image,
        point_image: cm.point_image,
        image_width: cm.image_width,
        image_height: cm.image_height,
        coordinate_mode: cm.coordinate_mode,
        pixel_coordinate_mode: cm.pixel_coordinate_mode,
        map_background: cm.map_background != null ? cloneVal(cm.map_background) : cm.map_background,
        image_detection_stats:
          cm.image_detection_stats != null ? cloneVal(cm.image_detection_stats) : cm.image_detection_stats,
        topo_edges_pixel:
          cm.topo_edges_pixel != null ? cloneVal(cm.topo_edges_pixel) : cm.topo_edges_pixel,
        image_inspection_overlay:
          cm.image_inspection_overlay != null
            ? cloneVal(cm.image_inspection_overlay)
            : c.image_inspection_overlay != null
              ? cloneVal(c.image_inspection_overlay)
              : rm.image_inspection_overlay,
      };
      if (cm.line_image != null && cm.line_image !== "") {
        metadata.line_image = cm.line_image;
      }
      if (cm.dataset_type != null && cm.dataset_type !== "") {
        metadata.dataset_type = cm.dataset_type;
      }
      if (cm.dataset != null && cm.dataset !== "") {
        metadata.dataset = cm.dataset;
      }
      metadata.inspection_point_source = MissionStore.normalizeSource(
        cm.inspection_point_source || metadata.inspection_point_source
      );

      return {
        ...c,
        ...r,
        segments: r.segments,
        statistics: r.statistics,
        visit_order: r.visit_order,
        markers: r.markers,
        inspection_points: r.inspection_points,
        group_visit_order: r.group_visit_order != null ? r.group_visit_order : c.group_visit_order,
        weather_zones: r.weather_zones != null ? r.weather_zones : c.weather_zones,
        output_files: r.output_files ?? c.output_files,
        map_background: c.map_background != null ? cloneVal(c.map_background) : c.map_background,
        bounds: c.bounds != null ? cloneVal(c.bounds) : c.bounds,
        image_inspection_overlay:
          c.image_inspection_overlay != null ? cloneVal(c.image_inspection_overlay) : undefined,
        metadata,
      };
    },

    applyServerReplan(dashboard, options = {}) {
      if (typeof window.prepareMissionPlotForServerReplan === "function") {
        window.prepareMissionPlotForServerReplan();
      }
      const prevPoints = MissionStore.getInspectionPoints();
      const prevSource = MissionStore.getSource();
      const current = MissionStore.getMission();
      const normalized = MissionStore.normalizeDashboard(dashboard);
      const merged = MissionStore.mergeReplanWithCurrentDashboard(current, normalized);
      MissionStore.loadFromDashboard(merged, {
        ...options,
        kind: "server_replan",
      });
      const nextSource = MissionStore.getSource();
      const nextPoints = MissionStore.getInspectionPoints();
      if (prevSource === "image" && nextSource !== "image") {
        console.warn("[MissionStore] server replan attempted source switch; forcing image");
        if (MissionStore.current) {
          MissionStore.current.source = "image";
          MissionStore.current.dashboard.metadata.inspection_point_source = "image";
        }
      }
      if (
        prevSource === "image"
        && prevPoints.length
        && nextPoints.length !== prevPoints.length
      ) {
        console.warn(
          "[MissionStore] server replan changed inspection point count",
          prevPoints.length,
          "->",
          nextPoints.length
        );
      }
      AppPhaseManager.setPhase(AppPhase.MISSION_READY, { reason: "server_replan_done" });
      return MissionStore.getMission();
    },

    applyAdaptiveConnectReroute(adaptivePath, options = {}) {
      if (!MissionStore.current || !Array.isArray(adaptivePath) || adaptivePath.length < 2) {
        return false;
      }
      const segments = MissionStore.current.segments.map((seg) => ({
        ...seg,
        geometry_2d: [...(seg.geometry_2d || [])],
      }));
      let targetIdx = typeof options.connectIndex === "number" ? options.connectIndex : -1;
      if (targetIdx < 0) {
        let bestDist = Infinity;
        const anchor = options.uav || null;
        segments.forEach((seg, idx) => {
          if (seg.type !== "connect") return;
          const geom = seg.geometry_2d || [];
          if (geom.length < 2) return;
          const ref = anchor || geom[0];
          const d = Math.hypot(geom[0][0] - ref[0], geom[0][1] - ref[1]);
          if (d < bestDist) {
            bestDist = d;
            targetIdx = idx;
          }
        });
      }
      if (targetIdx < 0 || segments[targetIdx]?.type !== "connect") return false;

      const newGeom = adaptivePath.map((p) => [Number(p[0]), Number(p[1])]);
      const segLen = newGeom.reduce((total, p, i) => {
        if (i === 0) return 0;
        return total + Math.hypot(p[0] - newGeom[i - 1][0], p[1] - newGeom[i - 1][1]);
      }, 0);
      segments[targetIdx] = {
        ...segments[targetIdx],
        geometry_2d: newGeom,
        length: Math.round(segLen),
        role: "adaptive_reroute",
      };

      MissionStore.current.segments = segments;
      MissionStore.current.dashboard = {
        ...MissionStore.current.dashboard,
        segments,
        inspection_points: MissionStore.current.inspection_points.map((p) => ({ ...p })),
      };
      MissionStore.current.last_update_kind = "adaptive_connect";
      MissionStore._emitMission();
      return true;
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

    buildBaselineMissionForReplan() {
      const d = MissionStore.getMission();
      if (!d || typeof d !== "object") return null;
      try {
        if (typeof structuredClone === "function") {
          return structuredClone(d);
        }
      } catch (_) {
        /* fall through */
      }
      try {
        return JSON.parse(JSON.stringify(d));
      } catch (_) {
        return null;
      }
    },

    buildReplanPayload(basePayload = {}) {
      const snap = MissionStore.current;
      const meta = snap?.metadata || snap?.dashboard?.metadata || {};
      const stats = meta.image_detection_stats || {};
      return {
        ...basePayload,
        inspection_point_source: MissionStore.getSource(),
        image_path: meta.map_image || basePayload.image_path,
        snap_threshold: stats?.detector_config?.snap_threshold ?? basePayload.snap_threshold ?? 30,
        inspection_points: MissionStore.getInspectionPoints().map((pt) => ({
          point_id: pt.point_id || pt.id,
          edge_id: pt.edge_id,
          x: pt.x,
          y: pt.y,
          pixel_position: pt.snapped_coord || pt.pixel_position || [pt.x, pt.y],
          snapped_coord: pt.snapped_coord || [pt.x, pt.y],
          raw_coord: pt.raw_coord,
          point_type: pt.point_type || pt.type,
          source_reason: pt.source_reason || pt.description,
          detection_result: pt.detection_result,
          visit_order: pt.visit_order,
        })),
        baseline_mission: MissionStore.buildBaselineMissionForReplan(),
      };
    },
  };

  const AppPhaseManager = {
    phase: AppPhase.IDLE,
    _playbackStatus: "idle",

    getPhase() {
      return AppPhaseManager.phase;
    },

    setPlaybackStatus(status) {
      AppPhaseManager._playbackStatus = status || "idle";
      if (status === "playing" && AppPhaseManager.phase === AppPhase.MISSION_READY) {
        AppPhaseManager.setPhase(AppPhase.PLAYING, { reason: "playback_started" });
      } else if (status === "paused" && AppPhaseManager.phase === AppPhase.PLAYING) {
        AppPhaseManager.setPhase(AppPhase.PAUSED, { reason: "playback_paused" });
      } else if (status === "idle" && [AppPhase.PLAYING, AppPhase.PAUSED, AppPhase.FINISHED].includes(AppPhaseManager.phase)) {
        AppPhaseManager.setPhase(AppPhase.MISSION_READY, { reason: "playback_idle" });
      } else if (status === "finished") {
        AppPhaseManager.setPhase(AppPhase.FINISHED, { reason: "playback_finished" });
      }
    },

    setPhase(nextPhase, meta = {}) {
      if (!nextPhase || AppPhaseManager.phase === nextPhase) return;
      const prev = AppPhaseManager.phase;
      AppPhaseManager.phase = nextPhase;
      MissionStore.phaseListeners.forEach((fn) => {
        try {
          fn(nextPhase, prev, meta);
        } catch (err) {
          console.warn("[AppPhase] listener error", err);
        }
      });
    },

    canRunAdaptive() {
      return AppPhaseManager.phase === AppPhase.PLAYING;
    },

    beginPlanning() {
      AppPhaseManager.setPhase(AppPhase.PLANNING, { reason: "plan_start" });
    },

    beginReplanning() {
      AppPhaseManager.setPhase(AppPhase.REPLANNING, { reason: "server_replan_start" });
    },

    beginAdaptiveWarning() {
      AppPhaseManager.setPhase(AppPhase.ADAPTIVE_WARNING, { reason: "adaptive_warning" });
    },

    endAdaptiveWarning() {
      if (AppPhaseManager._playbackStatus === "playing") {
        AppPhaseManager.setPhase(AppPhase.PLAYING, { reason: "adaptive_warning_end" });
      } else if (AppPhaseManager._playbackStatus === "paused") {
        AppPhaseManager.setPhase(AppPhase.PAUSED, { reason: "adaptive_warning_end" });
      } else {
        AppPhaseManager.setPhase(AppPhase.MISSION_READY, { reason: "adaptive_warning_end" });
      }
    },
  };

  window.MissionStore = MissionStore;
  window.AppPhase = AppPhase;
  window.AppPhaseManager = AppPhaseManager;
  window.normalizeInspectionPointSource = MissionStore.normalizeSource.bind(MissionStore);
})();
