"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const root = path.resolve(__dirname, "..", "..");
const source = fs.readFileSync(path.join(root, "web", "static", "app.js"), "utf8");
const start = source.indexOf("function updateDownloadLinks");
const end = source.indexOf("\nfunction ", start + 1);

function element() {
  return {
    href: "",
    hidden: false,
    classList: {
      add(name) { if (name === "hidden") this.owner.hidden = true; },
      remove(name) { if (name === "hidden") this.owner.hidden = false; },
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
  const context = { $: (id) => elements[id] || null };
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

updateDownloadLinks({ mission: "mission.json", analysis: "analysis.json", compare: "compare.json" }, "manual");
assert.equal(elements.dlMission.href, "/api/output/mission.json");
assert.equal(elements.dlAnalysis.href, "/api/output/analysis.json");
assert.equal(elements.dlAnalysis.hidden, false);
assert.equal(elements.dlCompare.href, "/api/output/compare.json");
assert.equal(elements.dlCompare.hidden, false);

console.log("download links null handling test passed");
