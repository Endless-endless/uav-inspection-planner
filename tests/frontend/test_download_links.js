"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const root = path.resolve(__dirname, "..", "..");
const source = fs.readFileSync(path.join(root, "web", "static", "app.js"), "utf8");
const start = source.indexOf("function isStartEndReplanMission");
const end = source.indexOf("\nfunction renderMeta", start + 1);

function element() {
  return {
    href: "",
    hidden: false,
    textContent: "任务数据",
    title: "",
    dataset: {},
    attributes: {},
    setAttribute(name, value) { this.attributes[name] = String(value); },
    removeAttribute(name) {
      delete this.attributes[name];
      if (name === "href") this.href = "";
      if (name === "title") this.title = "";
    },
    classList: {
      names: new Set(),
      add(name) {
        this.names.add(name);
        if (name === "hidden") this.owner.hidden = true;
      },
      remove(name) {
        this.names.delete(name);
        if (name === "hidden") this.owner.hidden = false;
      },
      contains(name) { return this.names.has(name); },
      owner: null,
    },
  };
}

function makeElements(ids) {
  const elements = {};
  for (const id of ids) {
    const node = element();
    node.classList.owner = node;
    elements[id] = node;
  }
  return elements;
}

function load(elements) {
  const context = { $: (id) => elements[id] || null, getCurrentMission: () => null };
  vm.createContext(context);
  vm.runInContext(`${source.slice(start, end)}\nthis.updateDownloadLinks = updateDownloadLinks;`, context);
  return context.updateDownloadLinks;
}

for (const missing of ["dlMission", "dlAnalysis", "dlCompare"]) {
  const all = ["headerDownload", "dlMission", "dlAnalysis", "dlCompare", "dlLegacyHtml"];
  const elements = makeElements(all.filter((id) => id !== missing));
  const updateDownloadLinks = load(elements);
  assert.doesNotThrow(() => updateDownloadLinks({}, "image"));
  assert.doesNotThrow(() => updateDownloadLinks({}, "manual"));
}

const elements = makeElements([
  "headerDownload", "dlMission", "dlAnalysis", "dlCompare", "dlLegacyHtml",
]);
const updateDownloadLinks = load(elements);
updateDownloadLinks({ mission_snapshot: "snapshot.json" }, "image");
assert.equal(elements.headerDownload.href, "/api/output/snapshot.json");
assert.equal(elements.dlMission.href, "/api/output/snapshot.json");
assert.equal(elements.dlAnalysis.hidden, true);
assert.equal(elements.dlCompare.hidden, true);

const missionB = {
  metadata: { source: "plan:start_end", planner: "start_end_replan", replan: {} },
};
updateDownloadLinks({ mission_snapshot: "snapshot.json" }, "image", missionB);
for (const id of ["headerDownload", "dlMission"]) {
  assert.equal(elements[id].href, "", `${id} must not retain Mission A href for B`);
  assert.equal(elements[id].attributes["aria-disabled"], "true");
  assert.equal(elements[id].classList.contains("is-disabled"), true);
  assert.match(elements[id].title, /重规划任务/);
}

updateDownloadLinks(
  { mission_snapshot: "snapshot-a-again.json" },
  "image",
  { metadata: { source: "plan:image" } }
);
for (const id of ["headerDownload", "dlMission"]) {
  assert.equal(elements[id].href, "/api/output/snapshot-a-again.json");
  assert.equal(elements[id].attributes["aria-disabled"], undefined);
  assert.equal(elements[id].classList.contains("is-disabled"), false);
  assert.equal(elements[id].textContent, "任务数据");
}

updateDownloadLinks({ mission: "mission.json", analysis: "analysis.json", compare: "compare.json" }, "manual");
assert.equal(elements.dlMission.href, "/api/output/mission.json");
assert.equal(elements.dlAnalysis.href, "/api/output/analysis.json");
assert.equal(elements.dlAnalysis.hidden, false);
assert.equal(elements.dlCompare.href, "/api/output/compare.json");
assert.equal(elements.dlCompare.hidden, false);

console.log("download links and replan export protection tests passed");
