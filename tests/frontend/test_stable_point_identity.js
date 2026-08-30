"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const root = path.resolve(__dirname, "..", "..");
const source = fs.readFileSync(path.join(root, "web", "static", "app.js"), "utf8");
const start = source.indexOf("const ROUTE_SEQ_EPS");
const end = source.indexOf("function cacheDrawnRouteForPlayback", start);
const context = {
  window: {},
  console: { log() {}, warn() {}, error() {} },
  isSkippableMissionDebugSegment: () => false,
  logInspectionRouteBindingSummary: () => {},
};
vm.createContext(context);
vm.runInContext(
  `${source.slice(start, end)}\nthis.__exports = { buildMissionRouteSequence };`,
  context,
  { filename: path.join(root, "web", "static", "app.js") }
);

const points = [
  {
    point_id: "IP_0007", raw_x: 1139.42, raw_y: 521.37,
    snapped_x: 10, snapped_y: 0, segment_id: "inspect_left",
    edge_id: "PL_000", image_url: "/api/inspection-image/IP_0007.jpg",
  },
  {
    point_id: "IP_0012", raw_x: 1293.46, raw_y: 855.54,
    snapped_x: 90, snapped_y: 0, segment_id: "inspect_right",
    edge_id: "PL_005", image_url: "/api/inspection-image/IP_0012.jpg",
  },
];

function build(order) {
  const segments = order.map((side, index) => ({
    segment_id: side === "left" ? "inspect_left" : "inspect_right",
    type: "inspect",
    edge_id: side === "left" ? "PL_000" : "PL_005",
    geometry_2d: side === "left" ? [[0, 0], [20, 0]] : [[80, 0], [100, 0]],
    length: 20,
  }));
  const mission = { segments, inspection_points: points, markers: { start: { x: 0, y: 0 } } };
  const rows = points.map((point, missionIndex) => ({
    x: point.raw_x, y: point.raw_y, point, missionIndex, cd: [],
  }));
  const sequence = context.__exports.buildMissionRouteSequence(mission, rows);
  const frames = {};
  sequence.forEach((frame, frameIndex) => {
    (frame.routePointIds || []).forEach((pointId) => {
      if (!frames[pointId]) frames[pointId] = [];
      frames[pointId].push(frameIndex);
    });
  });
  return frames;
}

const framesA = build(["left", "right"]);
const framesB = build(["right", "left"]);
for (const point of points) {
  assert.equal(framesA[point.point_id].length, 1);
  assert.equal(framesB[point.point_id].length, 1);
  assert.equal(point.image_url, `/api/inspection-image/${point.point_id}.jpg`);
}
assert.notEqual(framesA.IP_0012[0], framesB.IP_0012[0]);
assert.equal(points[0].point_id, "IP_0007");
assert.equal(points[1].point_id, "IP_0012");

console.log("stable point identity frontend test passed");
