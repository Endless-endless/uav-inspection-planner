/**
 * GridVision 毕设答辩核心代码摘录 — 地图路径可视化
 * 源文件: web/static/app.js
 * 行号: 1505-1710, 1898-2250, 2307-2531
 * 说明: 摘录供答辩展示，需与原工程 static 资源配合运行。
 */

/* ===== app.js L1505-L1710 ===== */
const BAD_SNAPPED_DIST_PX = 40;
function inspectionPointCoordPair(a, b) {
  const x = Number(a);
  const y = Number(b);
  return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
}

function inspectionPointRawXY(p) {
  if (!p || typeof p !== "object") return null;
  return (
    inspectionPointCoordPair(p.raw_x, p.raw_y) ||
    inspectionPointCoordPair(p.original_pixel_x, p.original_pixel_y) ||
    inspectionPointCoordPair(p.x, p.y)
  );
}

function isSnappedCoordUsable(p) {
  if (!p || typeof p !== "object") return false;
  const snapped = inspectionPointCoordPair(p.snapped_x, p.snapped_y);
  if (!snapped) return false;
  const raw = inspectionPointRawXY(p);
  if (!raw) return true;
  const dist = Math.hypot(raw.x - snapped.x, raw.y - snapped.y);
  if (dist <= BAD_SNAPPED_DIST_PX) return true;
  return false;
}

/** Dashboard 主图坐标：snapped 仅在 raw 偏移 ≤40px 时采用，否则 fallback */
function inspectionPointDisplayXY(p) {
  if (!p || typeof p !== "object") return null;
  if (isSnappedCoordUsable(p)) {
    const snapped = inspectionPointCoordPair(p.snapped_x, p.snapped_y);
    if (snapped) return snapped;
  }
  let r = inspectionPointCoordPair(p.display_x, p.display_y);
  if (r) return r;
  if (p.metadata && typeof p.metadata === "object") {
    r = inspectionPointCoordPair(p.metadata.display_x, p.metadata.display_y);
    if (r) return r;
  }
  r = inspectionPointRawXY(p);
  if (r) return r;
  const dr = p.detection_result || {};
  const rc = dr.raw_coord || p.raw_coord;
  if (Array.isArray(rc) && rc.length >= 2) {
    r = inspectionPointCoordPair(rc[0], rc[1]);
    if (r) return r;
  }
  const pos = p.pixel_position || p.position_2d || p.position || p.pos2d;
  if (Array.isArray(pos) && pos.length >= 2) {
    r = inspectionPointCoordPair(pos[0], pos[1]);
    if (r) return r;
  }
  return inspectionPointCoordPair(p.x, p.y);
}




/**
 * 用 mission.inspection_points 的 display x/y，在全部蓝色 inspect 段折线边上找最近投影；
 * 确定性插入：每点投影到最近边，同边上按 t 排序；不创建绿色连线 trace。
 * @param {Array<{ geom: number[][], hoverLine: string }>} inspectSegs
 * @param {object} mission
 */
function augmentInspectSegmentsWithInspectionPoints(inspectSegs, mission) {
  const drawPts = Array.isArray(mission?.inspection_points) ? mission.inspection_points : [];
  const m = drawPts.length;
  const inspectionRows = [];
  const missingRows = [];

  drawPts.forEach((p, idx) => {
    const xy = inspectionPointDisplayXY(p);
    const pointId = String(p.point_id || p.id || idx + 1).replace(/^point_/, "");
    if (!xy) {
      missingRows.push({
        point_id: pointId,
        x: p.x ?? "—",
        y: p.y ?? "—",
        nearest_dist: null,
      });
      return;
    }
    const totalPoints = m;
    const pointLabel = `巡检点 ${pointId}`;
    const segmentLabel = p.segment_id
      ? `巡检区段 ${String(p.segment_id).replace(/^seg_/, "")}`
      : "—";
    const cd = [
      segmentLabel,
      p.image_available ? "真实图片" : "占位图",
      `${p.progress_index || idx + 1}/${totalPoints}`,
      pointLabel,
    ];
    inspectionRows.push({ x: xy.x, y: xy.y, cd, idx, point_id: pointId });
  });

  const auditAndReturn = (inspectX, inspectY, inspectCustom, inserted) => {
    return { inspectX, inspectY, inspectCustom };
  };

  if (!inspectSegs?.length) {
    drawPts.forEach((p, idx) => {
      if (missingRows.some((r) => r.point_id === String(p.point_id || p.id || idx + 1).replace(/^point_/, ""))) {
        return;
      }
      const xy = inspectionPointDisplayXY(p);
      if (xy) {
        missingRows.push({
          point_id: String(p.point_id || p.id || idx + 1).replace(/^point_/, ""),
          x: xy.x,
          y: xy.y,
          nearest_dist: null,
        });
      }
    });
    const flat = flattenInspectSegsToPlotly(inspectSegs || []);
    return auditAndReturn(flat.xs, flat.ys, flat.customs, 0);
  }

  if (!inspectionRows.length) {
    const flat = flattenInspectSegsToPlotly(inspectSegs);
    return auditAndReturn(flat.xs, flat.ys, flat.customs, 0);
  }

  const edges = [];
  let arcBase = 0;
  inspectSegs.forEach((seg, segIdx) => {
    const g = seg.geom;
    for (let j = 0; j < g.length - 1; j += 1) {
      const ax = g[j][0];
      const ay = g[j][1];
      const bx = g[j + 1][0];
      const by = g[j + 1][1];
      const dx = bx - ax;
      const dy = by - ay;
      const len = Math.hypot(dx, dy);
      edges.push({ ax, ay, bx, by, dx, dy, len, arc0: arcBase, segIdx, localJ: j });
      arcBase += len;
    }
  });

  if (!edges.length) {
    inspectionRows.forEach((row) => {
      missingRows.push({
        point_id: row.point_id,
        x: row.x,
        y: row.y,
        nearest_dist: null,
      });
    });
    const flat = flattenInspectSegsToPlotly(inspectSegs);

/* ===== app.js L1898-L2250 ===== */
function expandInspectSegmentByArc(seg, pointsOnSeg, segmentId) {
  const geom = seg.geometry_2d || [];
  const sid = segmentId || String(seg.segment_id || "");
  const rawCount = geom.length;
  const failedPoints = [];
  const insertMeta = [];

  if (geom.length < 2) {
    (pointsOnSeg || []).forEach((pt) => {
      failedPoints.push({ pt, reason: "empty_geometry" });
    });
    return { frames: [], insertMeta, failedPoints, rawCount, pointCount: (pointsOnSeg || []).length };
  }

  const vertexArcs = polylineVertexArcs(geom);
  const items = geom.map((p, i) => ({
    x: Number(p[0]),
    y: Number(p[1]),
    arc: vertexArcs[i],
    kind: "vertex",
    point_ids: [],
    orderIndex: -1,
  }));

  const insertedIds = new Set();

  (pointsOnSeg || []).forEach((pt) => {
    if (!Number.isFinite(pt.x) || !Number.isFinite(pt.y)) {
      failedPoints.push({ pt, reason: "invalid_coords" });
      return;
    }

    const vi = findVertexMatchIndex(pt, geom, POINT_FRAME_EPS);
    if (vi >= 0) {
      appendPointIdToItem(items[vi], pt.point_id);
      insertedIds.add(pt.point_id);
      insertMeta.push({
        point_id: pt.point_id,
        arc: vertexArcs[vi],
        x: pt.x,
        y: pt.y,
      });
      return;
    }

    const proj = projectPointOnPolyline(pt.x, pt.y, geom);
    if (!proj || !Number.isFinite(proj.arcS)) {
      failedPoints.push({ pt, reason: "no_projection" });
      return;
    }

    items.push({
      x: pt.x,
      y: pt.y,
      arc: proj.arcS,
      t: proj.t,
      kind: "inspection",
      point_ids: [pt.point_id],
      orderIndex: pt.orderIndex != null ? pt.orderIndex : 0,
    });
    insertedIds.add(pt.point_id);
    insertMeta.push({
      point_id: pt.point_id,
      arc: proj.arcS,
      x: pt.x,
      y: pt.y,
    });
  });

  (pointsOnSeg || []).forEach((pt) => {
    if (!insertedIds.has(pt.point_id)) {
      if (!failedPoints.some((f) => f.pt.point_id === pt.point_id)) {
      failedPoints.push({ pt, reason: "not_inserted" });
      }
    }
  });

  items.sort((a, b) => {
    if (Math.abs(a.arc - b.arc) > 1e-9) return a.arc - b.arc;
    if (a.kind !== b.kind) return a.kind === "vertex" ? -1 : 1;
    if (a.kind === "inspection" && b.kind === "inspection") {
      return a.orderIndex - b.orderIndex;
    }
    return 0;
  });

  const deduped = [];
  items.forEach((it) => {
    const prev = deduped[deduped.length - 1];
    if (
      prev &&
      Math.abs(prev.x - it.x) <= ROUTE_SEQ_EPS &&
      Math.abs(prev.y - it.y) <= ROUTE_SEQ_EPS
    ) {
      prev.point_ids = mergeRoutePointIds(prev.point_ids, it.point_ids);
      if (it.kind === "inspection") {
        prev.kind = "inspection";
      }
      return;
    }
    deduped.push({
      ...it,
      point_ids: normalizeRoutePointIds(it.point_ids),
    });
  });

  const frames = deduped.map((it) => ({
    x: it.x,
    y: it.y,
    ...frameRouteFields(it.kind === "inspection" ? it.point_ids : []),
  }));

  const pointCount = (pointsOnSeg || []).length;
  const finalCount = frames.length;raw=${rawCount} points=${pointCount} final=${finalCount} first=(${frames[0].x},${frames[0].y}) last=(${frames[finalCount - 1].x},${frames[finalCount - 1].y})`
    );
  }

  return {
    frames,
    insertMeta,
    failedPoints,
    rawCount,
    pointCount,
    finalCount,
  };
}

function buildMissionRouteSequence(mission, rowsWithXY) {
  const markers = mission?.markers || {};
  const meta = mission?.metadata || {};
  const sx = Number(markers.start?.x ?? meta.start?.x ?? meta.start_point?.x);
  const sy = Number(markers.start?.y ?? meta.start?.y ?? meta.start_point?.y);
  const segments = mission?.segments || [];
  const dist2 = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

  const inspectPoints = [];
  (rowsWithXY || []).forEach((r, i) => {
    const p = r.point || {};
    const pointId = String(p.point_id || p.id || i + 1).trim();
    let boundSeg = String(p.segment_id || "").trim();
    if (!boundSeg) {
      for (const seg of segments) {
        if (isSkippableMissionDebugSegment(seg) || seg.type !== "inspect") continue;
        const geom = seg.geometry_2d || [];
        if (geom.length < 2) continue;
        if (pointToPolylineDist(r.x, r.y, geom) <= ON_GEOM_EPS) {
          boundSeg = String(seg.segment_id || "");
          break;
        }
      }
    }
    if (!boundSeg) {
      let bestD = Infinity;
      for (const seg of segments) {
        if (isSkippableMissionDebugSegment(seg) || seg.type !== "inspect") continue;
        const d = pointToPolylineDist(r.x, r.y, seg.geometry_2d || []);
        if (d < bestD) {
          bestD = d;
          boundSeg = String(seg.segment_id || "");
        }
      }
    }
    inspectPoints.push({
      point_id: pointId,
      id: pointId,
      x: r.x,
      y: r.y,
      image_url:
        (typeof p.image_url === "string" && p.image_url.trim()) ||
        `/api/inspection-image/${pointId}.jpg`,
      segment_id: boundSeg,
      edge_id: p.edge_id || "",
      orderIndex: i,
    });
  });

  const pointsBySegment = new Map();
  inspectPoints.forEach((pt) => {
    const sid = pt.segment_id;
    if (!sid) return;
    if (!pointsBySegment.has(sid)) pointsBySegment.set(sid, []);
    pointsBySegment.get(sid).push(pt);
  });

  const sequence = [];
  let skippedStartDup = false;

  const pushFrame = (frame) => {
    const prev = sequence[sequence.length - 1];
    const incomingIds = frame.routePointIds || (frame.routePointId ? [frame.routePointId] : []);
    if (prev && dist2(prev, frame) <= ROUTE_SEQ_EPS) {
      const merged = mergeRoutePointIds(prev.routePointIds, incomingIds);
      prev.routePointIds = merged;
      prev.routePointId = merged.length === 1 ? merged[0] : null;
      return;
    }
    sequence.push({
      ...frame,
      ...frameRouteFields(incomingIds),
    });
  };

  if (Number.isFinite(sx) && Number.isFinite(sy)) {
    pushFrame({
      segment_id: "start",
      type: "start",
      segment_type: "start",
      x: sx,
      y: sy,
      edge_id: null,
      routePointIds: [],
      routePointId: null,
    });
  }

  segments.forEach((seg) => {
    if (isSkippableMissionDebugSegment(seg)) return;
    const geom = seg.geometry_2d || [];
    if (geom.length < 2) return;
    const sid = String(seg.segment_id || "");
    const stype =
      seg.type === "inspect"
        ? "inspect"
        : seg.type === "connect"
          ? "connect"
          : String(seg.type || "move");
    const edgeId = seg.edge_id != null ? String(seg.edge_id) : null;

    if (stype === "inspect") {
      const segStartIdx = sequence.length;
      const expanded = expandInspectSegmentByArc(
        seg,
        pointsBySegment.get(sid) || [],
        sid
      );
      expanded.frames.forEach((v, idx) => {
        if (!skippedStartDup && idx === 0 && sequence.length) {
          const startFrame = sequence[0];
          if (
            startFrame?.type === "start" &&
            dist2(startFrame, v) <= ROUTE_SEQ_EPS
          ) {
            skippedStartDup = true;
            return;
          }
        }
        pushFrame({
          segment_id: sid,
          type: stype,
          segment_type: stype,
          x: v.x,
          y: v.y,
          edge_id: edgeId,
          routePointIds: v.routePointIds || (v.routePointId ? [v.routePointId] : []),
          routePointId: v.routePointId || null,
        });
      });
      expanded.insertMeta.forEach((meta) => {
        let frameIdx = -1;
        for (let i = segStartIdx; i < sequence.length; i += 1) {
          const ids = sequence[i].routePointIds || [];
          if (ids.includes(meta.point_id)) {
            frameIdx = i;
            break;
          }
        }
        if (frameIdx < 0) return;
              });
      return;
    }

    geom.forEach((p, idx) => {
      const x = Number(p[0]);
      const y = Number(p[1]);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      if (!skippedStartDup && idx === 0 && sequence.length) {
        const startFrame = sequence[0];
        if (
          startFrame?.type === "start" &&
          dist2(startFrame, { x, y }) <= ROUTE_SEQ_EPS
        ) {
          skippedStartDup = true;
          return;
        }
      }
      pushFrame({
        segment_id: sid,
        type: stype,
        segment_type: stype,
        x,
        y,
        edge_id: edgeId,
        routePointIds: [],
        routePointId: null,
      });
    });
  });

  const pointFrameIndex = {};
  sequence.forEach((f, i) => {
    (f.routePointIds || []).forEach((pid) => {
      pointFrameIndex[pid] = i;
    });
  });
  inspectPoints.forEach((pt) => {
    pt.trigger_frame = pointFrameIndex[pt.point_id] ?? -1;
  });  sequence.forEach((f, i) => {
    const ids = f.routePointIds || [];
    if (ids.length > 1) {
          }
  });

  window.__MISSION_ROUTE_SEQUENCE = sequence;
  window.__INSPECTION_POINT_BINDINGS = inspectPoints;

  const segSummary = [];
  let lastKey = null;
  sequence.forEach((f) => {
    const key = `${f.segment_id}:${f.segment_type || f.type}`;
    if (key !== lastKey) {
      segSummary.push(key);
      lastKey = key;
    }
  });
/* ===== app.js L2307-L2531 ===== */
function buildMissionTraces(missionPayload) {
  const mission = missionPayload || getCurrentMission();
  const inspectionPointsLengthAtEntry = mission?.inspection_points?.length ?? 0;
        const segments = mission?.segments || [];
  const inspectSegs = [];
  const connectX = [],
    connectY = [],
    connectCustom = [];

  segments.forEach((seg) => {
    if (isSkippableMissionDebugSegment(seg)) return;
    const geom = seg.geometry_2d || [];
    if (geom.length < 2) return;
    const segLabel = seg.segment_id
      ? `巡检区段 ${String(seg.segment_id).replace(/^seg_/, "")}`
      : "巡检区段";
    const typeLabel = seg.type === "inspect" ? "巡检段" : seg.type === "connect" ? "连接段" : "区段";
    const hoverLine = `${segLabel}<br>${typeLabel} · ${Math.round(seg.length || 0)}px`;

    if (seg.type === "inspect") {
      inspectSegs.push({
        geom: geom.map((p) => [Number(p[0]), Number(p[1])]),
        hoverLine,
      });
    } else if (seg.type === "connect") {
      connectX.push(...geom.map((p) => p[0]), null);
      connectY.push(...geom.map((p) => p[1]), null);
      connectCustom.push(...geom.map(() => hoverLine), null);
    }
  });

  [connectX, connectY, connectCustom].forEach((arr) => {
    if (arr.length && arr[arr.length - 1] === null) arr.pop();
  });

  /** Dashboard：巡检点 marker（不连线）；蓝线顶点 = 原 inspect 折线 + 按弧长投影插入的巡检点 */
  const drawPts = Array.isArray(mission.inspection_points) ? mission.inspection_points : [];
  const rowsWithXY = [];
  if (drawPts.length) {
    const totalPoints = drawPts.length;
    drawPts.forEach((p, idx) => {
      const xy = inspectionPointDisplayXY(p);
      if (!xy) return;
      const pointLabel = `巡检点 ${String(p.point_id || p.id || idx + 1).replace(/^point_/, "")}`;
      const segmentLabel = p.segment_id
        ? `巡检区段 ${String(p.segment_id).replace(/^seg_/, "")}`
        : "—";
      const cd = [
        segmentLabel,
        p.image_available ? "真实图片" : "占位图",
        `${p.progress_index || idx + 1}/${totalPoints}`,
        pointLabel,
      ];
      rowsWithXY.push({ x: xy.x, y: xy.y, cd, missionIndex: idx, point: p });
    });
  }

  let inspectX = [];
  let inspectY = [];
  let inspectCustom = [];
  if (inspectSegs.length) {
    const aug = augmentInspectSegmentsWithInspectionPoints(inspectSegs, mission);
    inspectX = aug.inspectX;
    inspectY = aug.inspectY;
    inspectCustom = aug.inspectCustom;
  } else if (rowsWithXY.length) {
  }

  const traces = [];
  const { trace: unvisitedRedTrace, diag: redDiag } = buildUnvisitedRedLinesTrace(mission);
  traces.push(unvisitedRedTrace);

if (state.showInspect && inspectX.length) {
    traces.push({
      x: inspectX,
      y: inspectY,
      mode: "lines",
      name: "巡检路径",
      line: { color: MAP_STYLES.inspectColor, width: 4 },
      hoverinfo: "text",
      hovertext: inspectCustom,
      showlegend: true,
      legendgroup: "inspect",
      legendrank: 1,
    });
  }

  if (state.showConnect && connectX.length) {
    traces.push({
      x: connectX,
      y: connectY,
      mode: "lines",
      name: "连接路径",
      line: { color: MAP_STYLES.connectColor, width: 3, dash: "dash" },
      hoverinfo: "text",
      hovertext: connectCustom,
      showlegend: true,
      legendgroup: "connect",
      legendrank: 2,
    });
  }

  if (rowsWithXY.length) {
    traces.push({
      x: rowsWithXY.map((r) => r.x),
      y: rowsWithXY.map((r) => r.y),
      mode: "markers",
      name: "巡检点",
      marker: {
        color: "#22c55e",
        size: 11,
        symbol: "circle",
        line: { color: "#052e16", width: 1.5 },
      },
      text: rowsWithXY.map((r) => r.cd[3]),
      customdata: rowsWithXY.map((r) => {
        const p = r.point;
        const pointId = String(p.point_id || p.id || "").trim();
        return {
          point_id: pointId,
          image_url:
            (typeof p.image_url === "string" && p.image_url.trim()) ||
            `/api/inspection-image/${pointId}.jpg`,
        };
      }),
      hovertemplate:
        "<b>%{text}</b><br>" +
        "point_id: %{customdata.point_id}<br>" +
        "url: %{customdata.image_url}<br>" +
        "<span style='opacity:0.85'>点击在侧栏查看巡检图</span><extra></extra>",
      hoverinfo: "text",
      showlegend: true,
      legendgroup: "points",
      legendrank: 3,
    });
      } else {
      }

  const markers = mission.markers || {};
  if (markers.start) {
    traces.push({
      x: [markers.start.x],
      y: [markers.start.y],
      mode: "markers+text",
      name: "起点",
      marker: {
        size: 16,
        color: "#22c55e",
        symbol: "star",
        line: { width: 2, color: "#14532d" },
      },
      text: ["起点"],
      textfont: { size: 10, color: "#bbf7d0" },
      textposition: "top center",
      hovertemplate: "起点 (%{x:.0f}, %{y:.0f})<extra></extra>",
      showlegend: true,
      legendgroup: "start",
      legendrank: 7,
    });
  }
  if (markers.end) {
    traces.push({
      x: [markers.end.x],
      y: [markers.end.y],
      mode: "markers+text",
      name: "终点",
      marker: {
        size: 16,
        color: "#ef4444",
        symbol: "diamond",
        line: { width: 2, color: "#7f1d1d" },
      },
      text: ["终点"],
      textfont: { size: 10, color: "#fecaca" },
      textposition: "bottom center",
      hovertemplate: "终点 (%{x:.0f}, %{y:.0f})<extra></extra>",
      showlegend: true,
      legendgroup: "end",
      legendrank: 8,
    });
  }

  cacheDrawnRouteForPlayback(mission, connectX, connectY, inspectX, inspectY, rowsWithXY);
  buildMissionRouteSequence(mission, rowsWithXY);

  return traces;
}
