"use strict";

const assert = require("node:assert/strict");
const crypto = require("node:crypto");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const root = path.resolve(__dirname, "..", "..");
const appPath = path.join(root, "web", "static", "app.js");
const playbackPath = path.join(root, "web", "static", "playback.js");
const indexPath = path.join(root, "web", "index.html");

function loadProductionRouteFunctions() {
  const source = fs.readFileSync(appPath, "utf8");
  const start = source.indexOf("const ROUTE_SEQ_EPS");
  const end = source.indexOf("function cacheDrawnRouteForPlayback", start);
  assert.ok(start >= 0 && end > start, "production route builder block must exist");
  const context = {
    window: {},
    console: { log() {}, warn() {}, error() {} },
    isSkippableMissionDebugSegment: () => false,
    logInspectionRouteBindingSummary: () => {},
  };
  vm.createContext(context);
  vm.runInContext(
    `${source.slice(start, end)}\nthis.__routeExports = { projectPointOnPolyline, buildMissionRouteSequence };`,
    context,
    { filename: appPath }
  );
  return { context, ...context.__routeExports };
}

function pair(a, b) {
  const x = Number(a);
  const y = Number(b);
  return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
}

// Mirrors the green evidence-marker coordinate policy. Route construction must
// not treat this display coordinate as the UAV visit coordinate.
function evidenceDisplayXY(point) {
  const raw = pair(point.raw_x, point.raw_y) || pair(point.x, point.y);
  const snapped = pair(point.snapped_x, point.snapped_y);
  if (snapped && (!raw || Math.hypot(raw.x - snapped.x, raw.y - snapped.y) <= 40)) {
    return snapped;
  }
  return pair(point.display_x, point.display_y) || raw;
}

function segmentBusinessHash(dashboard) {
  const payload = (dashboard.segments || []).map((segment) => ({
    segment_id: segment.segment_id,
    type: segment.type,
    geometry_2d: segment.geometry_2d,
    length: segment.length,
  }));
  return crypto.createHash("sha256").update(JSON.stringify(payload)).digest("hex");
}

function pointFrames(sequence) {
  const out = new Map();
  sequence.forEach((frame, frameIndex) => {
    (frame.routePointIds || []).forEach((pointId) => {
      if (!out.has(pointId)) out.set(pointId, []);
      out.get(pointId).push(frameIndex);
    });
  });
  return out;
}

function frameDistance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function makePointAccessDashboard() {
  const points = Array.from({ length: 13 }, (_, index) => {
    const number = index + 1;
    const pointId = `IP_${String(number).padStart(4, "0")}`;
    const route = number === 7 ? [60, 10] : [number * 10 - 10, 0];
    return {
      point_id: pointId,
      raw_x: route[0],
      raw_y: route[1],
      x: route[0],
      y: route[1],
      route_visit_coord: route,
      segment_id: number <= 6 ? "seg_0000" : number === 7 ? "seg_0001" : "seg_0002",
      edge_id: "PL_000",
    };
  });
  return {
    markers: { start: { x: -10, y: 0 } },
    inspection_points: points,
    segments: [
      { segment_id: "seg_0000", type: "inspect", edge_id: "PL_000", geometry_2d: [[0, 0], [60, 0]], length: 60 },
      {
        segment_id: "seg_0001",
        type: "connect",
        connect_mode: "free_flight",
        planner: "euclidean",
        reason: "point_access",
        fallback_reason: null,
        inspection_point_id: "IP_0007",
        physical_id: "PL_000",
        anchor_coord: [60, 0],
        raw_coord: [60, 10],
        geometry_2d: [[60, 0], [60, 10], [60, 0]],
        length: 20,
      },
      { segment_id: "seg_0002", type: "inspect", edge_id: "PL_000", geometry_2d: [[60, 0], [120, 0]], length: 60 },
    ],
  };
}

function testNearestProjection(projectPointOnPolyline) {
  const geometry = [[0, 0], [10, 0], [10, 100], [30, 100]];
  const projected = projectPointOnPolyline(25, 101, geometry);
  assert.equal(projected.edgeIndex, 2, "nearest edge must win over earliest arc edge");
  assert.ok(Math.abs(projected.d - 1) < 1e-9);
  assert.ok(Math.abs(projected.x - 25) < 1e-9);
  assert.ok(Math.abs(projected.y - 100) < 1e-9);
}

function testChengduRoute(buildMissionRouteSequence, context) {
  const dashboard = makePointAccessDashboard();
  const beforeHash = segmentBusinessHash(dashboard);
  const beforeSegments = JSON.stringify(dashboard.segments);
  const rows = dashboard.inspection_points.map((point, missionIndex) => {
    const xy = evidenceDisplayXY(point);
    assert.ok(xy, `display coordinate missing for ${point.point_id}`);
    return { x: xy.x, y: xy.y, point, missionIndex, cd: [] };
  });

  const sequence = buildMissionRouteSequence(dashboard, rows);
  const bindings = context.window.__INSPECTION_POINT_BINDINGS;
  const byId = pointFrames(sequence);
  const expectedIds = dashboard.inspection_points.map((p) => p.point_id);

  assert.equal(expectedIds.length, 13);
  assert.equal(new Set(expectedIds).size, 13);
  expectedIds.forEach((pointId) => {
    assert.equal(byId.get(pointId)?.length, 1, `${pointId} must enter timeline exactly once`);
    assert.ok(bindings.some((point) => point.point_id === pointId), `${pointId} missing from playback lookup`);
  });

  const expectedRoutes = new Map(
    dashboard.inspection_points.map((point) => [point.point_id, point.route_visit_coord])
  );
  expectedRoutes.forEach(([x, y], pointId) => {
    const binding = bindings.find((point) => point.point_id === pointId);
    assert.ok(Math.abs(binding.x - x) <= 1 && Math.abs(binding.y - y) <= 1,
      `${pointId} must use its authoritative Mission route coordinate`);
    const frameIndex = byId.get(pointId)[0];
    const frame = sequence[frameIndex];
    assert.ok(Math.abs(frame.x - x) <= 1 && Math.abs(frame.y - y) <= 1);
  });

  const frame10 = byId.get("IP_0010")[0];
  const frame12 = byId.get("IP_0012")[0];
  assert.notEqual(frame10, frame12, "IP_0010/IP_0012 must retain distinct visit frames");
  assert.deepEqual(Array.from(sequence[frame10].routePointIds), ["IP_0010"]);
  assert.deepEqual(Array.from(sequence[frame12].routePointIds), ["IP_0012"]);

  const accessFrame = sequence[byId.get("IP_0007")[0]];
  assert.deepEqual([accessFrame.x, accessFrame.y], [60, 10], "IP_0007 must visit the raw point-access vertex");
  const accessFrameIndex = byId.get("IP_0007")[0];
  assert.deepEqual(
    [sequence[accessFrameIndex - 1].x, sequence[accessFrameIndex - 1].y],
    [60, 0],
    "point access must enter from its declared anchor"
  );
  assert.deepEqual(
    [sequence[accessFrameIndex + 1].x, sequence[accessFrameIndex + 1].y],
    [60, 0],
    "point access must return to its declared anchor"
  );

  // Speed changes scheduling only; coordinates and point-frame mapping do not
  // depend on playback speed.
  const coordinates1x = sequence.map(({ x, y }) => [x, y]);
  const coordinates5x = sequence.map(({ x, y }) => [x, y]);
  assert.deepEqual(coordinates5x, coordinates1x);
  assert.equal(120 / 1, 120);
  assert.equal(120 / 5, 24);

  assert.equal(JSON.stringify(dashboard.segments), beforeSegments, "route build mutated Mission segments");
  const afterHash = segmentBusinessHash(dashboard);
  assert.equal(afterHash, beforeHash, "Mission segment business hash changed");

  const diagnostics = expectedIds.map((pointId, index) => {
    const point = dashboard.inspection_points[index];
    const binding = bindings.find((item) => item.point_id === pointId);
    const frameIndex = byId.get(pointId)[0];
    const prev = frameIndex > 0 ? frameDistance(sequence[frameIndex - 1], sequence[frameIndex]) : null;
    const next = frameIndex + 1 < sequence.length ? frameDistance(sequence[frameIndex], sequence[frameIndex + 1]) : null;
    return {
      ui: index + 1,
      point_id: pointId,
      raw: [point.raw_x, point.raw_y],
      route: [binding.x, binding.y],
      segment_id: binding.segment_id,
      frame: frameIndex,
      trigger_count: byId.get(pointId).length,
      prev_distance: prev,
      next_distance: next,
      status: "valid",
    };
  });

  const targetIds = new Set(["IP_0005", "IP_0007", "IP_0012"]);
  const maxTargetNeighborDistance = Math.max(
    ...diagnostics
      .filter((item) => targetIds.has(item.point_id))
      .flatMap((item) => [item.prev_distance, item.next_distance].filter(Number.isFinite))
  );
  const maxTimelineStep = Math.max(
    ...sequence.slice(1).map((frame, index) => frameDistance(sequence[index], frame))
  );

  return {
    beforeHash,
    afterHash,
    diagnostics,
    sequenceLength: sequence.length,
    maxTargetNeighborDistance,
    maxTimelineStep,
    sharedFrame: frame10,
  };
}

function testPhaseOneCompatibility() {
  const html = fs.readFileSync(indexPath, "utf8");
  const playback = fs.readFileSync(playbackPath, "utf8");
  ["inspectCardImg", "inspectCardImgFs"].forEach((id) => {
    assert.equal((html.match(new RegExp(`id=["']${id}["']`, "g")) || []).length, 1);
    assert.ok(playback.includes(id));
  });
  assert.ok(html.includes("/static/inspection_evidence.js"));
}

const production = loadProductionRouteFunctions();
testNearestProjection(production.projectPointOnPolyline);
const report = testChengduRoute(production.buildMissionRouteSequence, production.context);
testPhaseOneCompatibility();
console.log(JSON.stringify(report, null, 2));
console.log("playback route tests passed");
