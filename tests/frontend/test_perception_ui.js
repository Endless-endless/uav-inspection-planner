"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const root = path.resolve(__dirname, "..", "..");
const source = fs.readFileSync(
  path.join(root, "web", "static", "inspection_evidence.js"),
  "utf8"
);
const MISSION_ID = "mission_test_binding";
const MISSION_SHA256 = "a".repeat(64);

class FakeElement {
  constructor(tag) {
    this.tagName = tag;
    this.children = [];
    this.textContent = "";
    this.className = "";
    this.hidden = false;
    this.listeners = new Map();
  }
  append(...children) {
    this.children.push(...children);
  }
  replaceChildren(...children) {
    this.children = [...children];
    this.textContent = "";
  }
  addEventListener(type, listener) {
    this.listeners.set(type, listener);
  }
  dispatch(type) {
    this.listeners.get(type)?.({ type, target: this });
  }
}

class FakeFormData {
  constructor() {
    this.fields = [];
  }
  append(name, value) {
    this.fields.push([name, value]);
  }
}

function response(status, payload) {
  return {
    ok: status >= 200 && status < 300,
    status,
    async json() {
      return payload;
    },
  };
}

const document = {
  readyState: "complete",
  querySelectorAll() {
    return [];
  },
  createElement(tag) {
    return new FakeElement(tag);
  },
  addEventListener() {},
};
const window = {
  document,
  location: { href: "http://127.0.0.1:8001/" },
  addEventListener() {},
  setTimeout,
  clearTimeout,
  fetch() {
    throw new Error("unexpected default fetch");
  },
  MissionStore: {
    version: 1,
    getInspectionPoints() {
      return [{ point_id: "IP_00012" }, { point_id: "IP_00007" }];
    },
    getMission() {
      return {
        metadata: { mission_id: MISSION_ID, mission_sha256: MISSION_SHA256 },
        image_inspection_overlay: [{ id: "IP_0012" }],
      };
    },
    subscribeMission() {},
  },
};
const context = {
  window,
  document,
  console,
  URL,
  FormData: FakeFormData,
  Error,
  Set,
  Number,
  String,
  encodeURIComponent,
};
vm.createContext(context);
vm.runInContext(source, context, { filename: "inspection_evidence.js" });
const hooks = window.__PerceptionEvidenceTestHooks;

function timerHarness() {
  let nextId = 1;
  const timers = new Map();
  return {
    timers,
    setTimer(fn) {
      const id = nextId++;
      timers.set(id, fn);
      return id;
    },
    clearTimer(id) {
      timers.delete(id);
    },
  };
}

function flattenText(node) {
  return [node.textContent, ...node.children.flatMap((child) => flattenText(child))]
    .filter(Boolean)
    .join(" ");
}

function findElements(node, tagName) {
  const matches = node.tagName === tagName ? [node] : [];
  return matches.concat(
    ...node.children.map((child) => findElements(child, tagName))
  );
}

async function testAuthoritativePointSource() {
  const points = hooks.getAuthoritativeInspectionPoints();
  assert.deepEqual(
    points.map((point) => point.point_id),
    ["IP_00012", "IP_00007"]
  );
  assert.ok(!points.some((point) => point.point_id === "IP_0012"));
}

async function testPostPayloadTerminalPollingAndResult() {
  const timer = timerHarness();
  const requests = [];
  const results = [];
  let form = null;
  const fetchImpl = async (url, options = {}) => {
    requests.push([url, options]);
    if (url === "/api/v1/perception/jobs") {
      form = options.body;
      return response(202, { workflow_job_id: "perception_job_1", status: "queued" });
    }
    if (url.endsWith("/result")) {
      return response(200, {
        status: "completed",
        inspection_point_id: "IP_00012",
        target_frame_count: 1,
        frames: [],
      });
    }
    return response(200, {
      workflow_job_id: "perception_job_1",
      status: "completed",
      stage: "finished",
      progress: 100,
    });
  };
  const controller = new hooks.PerceptionWorkflowController({
    fetchImpl,
    setTimer: timer.setTimer,
    clearTimer: timer.clearTimer,
    formDataFactory: () => new FakeFormData(),
    onResult: (result) => results.push(result),
  });
  const video = { name: "flight.mp4" };
  await controller.start({
    video,
    inspectionPointId: "IP_00012",
    missionId: MISSION_ID,
    missionSha256: MISSION_SHA256,
    videoId: "VIDEO_1",
  });
  assert.deepEqual(form.fields, [
    ["video", video],
    ["inspection_point_id", "IP_00012"],
    ["mission_id", MISSION_ID],
    ["mission_sha256", MISSION_SHA256],
    ["video_id", "VIDEO_1"],
  ]);
  assert.equal(timer.timers.size, 1);
  await controller.pollNow();
  assert.equal(timer.timers.size, 0, "terminal state must stop polling");
  assert.equal(results.length, 1);
  assert.equal(results[0].inspection_point_id, "IP_00012");
  assert.equal(requests.length, 3);
}

async function testNoDuplicateTimerAndFailedError() {
  const timer = timerHarness();
  const errors = [];
  let statusCalls = 0;
  const controller = new hooks.PerceptionWorkflowController({
    fetchImpl: async (url) => {
      if (url === "/api/v1/perception/jobs") {
        return response(202, { workflow_job_id: "job_2", status: "queued" });
      }
      statusCalls += 1;
      if (statusCalls === 1) {
        return response(200, { status: "running", stage: "video_running", progress: 30 });
      }
      return response(200, {
        status: "failed",
        stage: "finished",
        progress: 100,
        error: { code: "video_job_failed", message: "视频识别失败" },
      });
    },
    setTimer: timer.setTimer,
    clearTimer: timer.clearTimer,
    formDataFactory: () => new FakeFormData(),
    onError: (error) => errors.push(error.message),
  });
  await controller.start({
    video: {}, inspectionPointId: "IP_00012",
    missionId: MISSION_ID, missionSha256: MISSION_SHA256, videoId: "",
  });
  assert.equal(timer.timers.size, 1);
  await controller.pollNow();
  assert.equal(timer.timers.size, 1, "rescheduling must replace the old timer");
  await controller.pollNow();
  assert.equal(timer.timers.size, 0);
  assert.deepEqual(errors, ["视频识别失败"]);
}

async function testUploadFailureRestoresController() {
  const timer = timerHarness();
  const errors = [];
  const controller = new hooks.PerceptionWorkflowController({
    fetchImpl: async () =>
      response(503, {
        detail: { code: "mission_unavailable", message: "当前 Mission 不可用" },
      }),
    setTimer: timer.setTimer,
    clearTimer: timer.clearTimer,
    formDataFactory: () => new FakeFormData(),
    onError: (error) => errors.push(error.message),
  });
  await assert.rejects(
    controller.start({
      video: {}, inspectionPointId: "IP_00012",
      missionId: MISSION_ID, missionSha256: MISSION_SHA256, videoId: "",
    }),
    /当前 Mission 不可用/
  );
  assert.equal(controller.active, false);
  assert.equal(controller.startInFlight, false);
  assert.equal(timer.timers.size, 0);
  assert.deepEqual(errors, ["当前 Mission 不可用"]);
}

async function testCompletedResultRenderingAndAnnotatedUrlPolicy() {
  const container = new FakeElement("div");
  hooks.renderPerceptionResult(container, {
    status: "completed_with_errors",
    inspection_point_id: "IP_00012",
    target_frame_count: 1,
    frames: [
      {
        frame_id: "FRAME_000018",
        timestamp_ms: 750,
        status: "completed",
        defect_detection: {
          detection_count: 1,
          detections: [{ class_name: "bird_nest", confidence: 0.863 }],
          annotated_image_url: "/api/v1/perception/media/defect/annotated.jpg",
        },
      },
    ],
  });
  const text = flattenText(container);
  assert.match(text, /IP_00012/);
  assert.match(text, /FRAME_000018/);
  assert.match(text, /bird_nest 86\.3%/);
  const links = findElements(container, "a");
  const images = findElements(container, "img");
  assert.equal(links.length, 1);
  assert.equal(
    links[0].href,
    "/api/v1/perception/media/defect/annotated.jpg"
  );
  assert.equal(images.length, 1);
  assert.equal(
    images[0].src,
    "/api/v1/perception/media/defect/annotated.jpg"
  );
  images[0].dispatch("error");
  assert.equal(images[0].hidden, true);
  assert.match(flattenText(container), /标注图加载失败/);
  assert.match(flattenText(container), /bird_nest 86\.3%/);
  assert.equal(
    hooks.accessibleAnnotatedUrl(
      "/api/v1/perception/media/defect/a.jpg"
    ),
    "/api/v1/perception/media/defect/a.jpg"
  );
  assert.equal(hooks.accessibleAnnotatedUrl("/results/a.jpg"), null);
  assert.equal(
    hooks.accessibleAnnotatedUrl("http://127.0.0.1:8003/results/a.jpg"),
    null
  );
}

Promise.resolve()
  .then(testAuthoritativePointSource)
  .then(testPostPayloadTerminalPollingAndResult)
  .then(testNoDuplicateTimerAndFailedError)
  .then(testUploadFailureRestoresController)
  .then(testCompletedResultRenderingAndAnnotatedUrlPolicy)
  .then(() => console.log("perception UI behavior tests passed"))
  .catch((error) => {
    console.error(error);
    process.exitCode = 1;
  });
