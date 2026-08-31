"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");

const root = path.resolve(__dirname, "..", "..");
const html = fs.readFileSync(path.join(root, "web", "index.html"), "utf8");
const evidence = fs.readFileSync(
  path.join(root, "web", "static", "inspection_evidence.js"),
  "utf8"
);
const plotlyPath = path.join(
  root,
  "web",
  "static",
  "vendor",
  "plotly-2.27.0.min.js"
);

assert.match(html, /\/static\/vendor\/plotly-2\.27\.0\.min\.js/);
assert.doesNotMatch(html, /cdn\.plot\.ly/);
assert.equal(fs.existsSync(plotlyPath), true);
const plotlyStat = fs.statSync(plotlyPath);
assert.ok(plotlyStat.size > 3_000_000 && plotlyStat.size < 6_000_000);
assert.match(fs.readFileSync(plotlyPath, "utf8").slice(0, 200), /plotly\.js v2\.27\.0/);

assert.doesNotMatch(html, /链路在线|等待视频识别接口|等待缺陷检测接口|模块接入后显示/);
assert.equal((html.match(/识别与缺陷结果/g) || []).length >= 4, true);
assert.equal((html.match(/仿真航线回放进度/g) || []).length, 2);
assert.match(evidence, /感知工作流进度/);
assert.match(evidence, /与仿真航线回放独立/);
assert.match(evidence, /工作流 ID/);
assert.match(evidence, /视频任务 ID/);
assert.match(evidence, /已完成（部分异常）/);

console.log("finalization presentation static tests passed");
