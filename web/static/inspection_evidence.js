(() => {
  "use strict";

  const ROOT_SELECTOR = "#inspectInfoCard, #inspectInfoCardFs";
  const TAB_SELECTOR = "[data-evidence-tab]";
  const PANEL_SELECTOR = "[data-evidence-panel]";
  let activeTab = "replay";

  function setActiveTab(name) {
    activeTab = name;
    document.querySelectorAll(ROOT_SELECTOR).forEach((root) => {
      root.querySelectorAll(TAB_SELECTOR).forEach((tab) => {
        const selected = tab.dataset.evidenceTab === name;
        tab.setAttribute("aria-selected", String(selected));
        tab.tabIndex = selected ? 0 : -1;
      });
      root.querySelectorAll(PANEL_SELECTOR).forEach((panel) => {
        panel.hidden = panel.dataset.evidencePanel !== name;
      });
    });
  }

  function init() {
    document.querySelectorAll(ROOT_SELECTOR).forEach((root) => {
      root.querySelectorAll(TAB_SELECTOR).forEach((tab) => {
        tab.addEventListener("click", () => setActiveTab(tab.dataset.evidenceTab));
      });
    });
    setActiveTab(activeTab);
  }

  function safeInit() {
    try {
      init();
    } catch (error) {
      console.error("Inspection evidence panel initialization failed", error);
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", safeInit, { once: true });
  } else {
    safeInit();
  }
})();

(() => {
  "use strict";

  const ROOT_SELECTOR = "#inspectInfoCard, #inspectInfoCardFs";
  const TERMINAL_STATES = new Set(["completed", "completed_with_errors", "failed"]);
  const POLL_INTERVAL_MS = 1500;
  const STATUS_LABELS = {
    idle: "待命",
    uploading: "上传中",
    queued: "排队中",
    running: "处理中",
    video_running: "视频识别",
    defect_running: "缺陷检测",
    completed: "已完成",
    completed_with_errors: "部分完成",
    failed: "失败",
  };
  let lastMissionVersion = -1;

  function activateEvidenceTab(name) {
    document.querySelectorAll(ROOT_SELECTOR).forEach((root) => {
      root.querySelectorAll("[data-evidence-tab]").forEach((tab) => {
        const selected = tab.dataset.evidenceTab === name;
        tab.setAttribute("aria-selected", String(selected));
        tab.tabIndex = selected ? 0 : -1;
      });
      root.querySelectorAll("[data-evidence-panel]").forEach((panel) => {
        panel.hidden = panel.dataset.evidencePanel !== name;
      });
    });
  }

  function getAuthoritativeInspectionPoints() {
    const points = window.MissionStore?.getInspectionPoints?.();
    if (!Array.isArray(points)) return [];
    return points.filter(
      (point) => point && typeof point.point_id === "string" && point.point_id.length > 0
    );
  }

  function getCurrentMissionBinding() {
    const metadata = window.MissionStore?.getMission?.()?.metadata || {};
    return {
      missionId: typeof metadata.mission_id === "string" ? metadata.mission_id : "",
      missionSha256:
        typeof metadata.mission_sha256 === "string" ? metadata.mission_sha256 : "",
    };
  }

  function responseError(payload, fallback) {
    const detail = payload?.detail;
    if (detail && typeof detail === "object") {
      return detail.message || detail.code || fallback;
    }
    if (typeof detail === "string" && detail) return detail;
    return payload?.message || fallback;
  }

  async function readJson(response) {
    try {
      return await response.json();
    } catch (_) {
      return null;
    }
  }

  function accessibleAnnotatedUrl(value, pageHref = window.location?.href || "") {
    const raw = typeof value === "string" ? value.trim() : "";
    const proxyPath = /^\/api\/v1\/perception\/media\/defect\/[A-Za-z0-9][A-Za-z0-9._-]*\.(?:jpe?g|png|bmp)$/i;
    if (proxyPath.test(raw)) return raw;
    try {
      const pageUrl = new URL(pageHref);
      const candidate = new URL(raw, pageUrl);
      if (
        candidate.origin !== pageUrl.origin ||
        candidate.search ||
        candidate.hash ||
        !proxyPath.test(candidate.pathname)
      ) return null;
      return candidate.href;
    } catch (_) {
      return null;
    }
  }

  class PerceptionWorkflowController {
    constructor(options = {}) {
      this.fetchImpl = options.fetchImpl || window.fetch.bind(window);
      this.setTimer = options.setTimer || window.setTimeout.bind(window);
      this.clearTimer = options.clearTimer || window.clearTimeout.bind(window);
      this.formDataFactory = options.formDataFactory || (() => new FormData());
      this.pollIntervalMs = options.pollIntervalMs ?? POLL_INTERVAL_MS;
      this.onState = options.onState || (() => {});
      this.onResult = options.onResult || (() => {});
      this.onError = options.onError || (() => {});
      this.timerId = null;
      this.workflowJobId = null;
      this.startInFlight = false;
      this.active = false;
      this.generation = 0;
    }

    async start({ video, inspectionPointId, missionId, missionSha256, videoId }) {
      if (this.startInFlight) throw new Error("识别请求正在提交，请稍候");
      if (!missionId || !missionSha256) {
        throw new Error("当前 Mission 缺少可验证的运行时身份，请重新加载任务");
      }
      this.cancelPolling();
      this.generation += 1;
      const generation = this.generation;
      this.startInFlight = true;
      this.onState({ status: "uploading", stage: "uploading", progress: 0, error: null });
      try {
        const form = this.formDataFactory();
        form.append("video", video);
        form.append("inspection_point_id", inspectionPointId);
        form.append("mission_id", missionId);
        form.append("mission_sha256", missionSha256);
        if (videoId) form.append("video_id", videoId);
        const response = await this.fetchImpl("/api/v1/perception/jobs", {
          method: "POST",
          body: form,
        });
        const payload = await readJson(response);
        if (!response.ok) {
          throw new Error(responseError(payload, `上传失败（HTTP ${response.status}）`));
        }
        if (!payload || typeof payload.workflow_job_id !== "string") {
          throw new Error("8001 返回的 workflow_job_id 无效");
        }
        if (generation !== this.generation) return null;
        this.workflowJobId = payload.workflow_job_id;
        this.active = true;
        this.onState({ ...payload, progress: 0, stage: "queued", error: null });
        this.schedulePoll();
        return payload;
      } catch (error) {
        this.startInFlight = false;
        if (generation === this.generation) this.fail(error);
        throw error;
      } finally {
        this.startInFlight = false;
      }
    }

    schedulePoll() {
      if (!this.active || !this.workflowJobId) return;
      this.stopTimer();
      const generation = this.generation;
      this.timerId = this.setTimer(() => {
        this.timerId = null;
        this.pollNow(generation).catch((error) => this.fail(error));
      }, this.pollIntervalMs);
    }

    async pollNow(expectedGeneration = this.generation) {
      if (!this.active || !this.workflowJobId || expectedGeneration !== this.generation) {
        return null;
      }
      this.stopTimer();
      const response = await this.fetchImpl(
        `/api/v1/perception/jobs/${encodeURIComponent(this.workflowJobId)}`
      );
      const statusPayload = await readJson(response);
      if (!response.ok) {
        throw new Error(responseError(statusPayload, `状态查询失败（HTTP ${response.status}）`));
      }
      if (expectedGeneration !== this.generation) return null;
      const terminal = TERMINAL_STATES.has(statusPayload.status);
      if (terminal) this.active = false;
      this.onState(statusPayload);
      if (!terminal) {
        this.schedulePoll();
        return statusPayload;
      }
      this.stopTimer();
      if (statusPayload.status === "failed") {
        const error = new Error(
          statusPayload.error?.message || statusPayload.error?.code || "感知任务失败"
        );
        this.onError(error, statusPayload);
        return statusPayload;
      }
      await this.loadResult(expectedGeneration);
      return statusPayload;
    }

    async loadResult(expectedGeneration = this.generation) {
      const response = await this.fetchImpl(
        `/api/v1/perception/jobs/${encodeURIComponent(this.workflowJobId)}/result`
      );
      const result = await readJson(response);
      if (!response.ok) {
        throw new Error(responseError(result, `结果读取失败（HTTP ${response.status}）`));
      }
      if (expectedGeneration === this.generation) this.onResult(result);
      return result;
    }

    fail(error) {
      this.active = false;
      this.stopTimer();
      const normalized = error instanceof Error ? error : new Error(String(error));
      this.onState({ status: "failed", stage: "failed", progress: 0, error: normalized.message });
      this.onError(normalized);
    }

    stopTimer() {
      if (this.timerId !== null) {
        this.clearTimer(this.timerId);
        this.timerId = null;
      }
    }

    cancelPolling() {
      this.generation += 1;
      this.active = false;
      this.workflowJobId = null;
      this.stopTimer();
    }
  }

  function mountPanels() {
    document.querySelectorAll(ROOT_SELECTOR).forEach((root) => {
      const videoPanel = root.querySelector('[data-evidence-panel="video"]');
      const defectPanel = root.querySelector('[data-evidence-panel="defect"]');
      if (videoPanel && !videoPanel.dataset.perceptionMounted) {
        videoPanel.dataset.perceptionMounted = "true";
        videoPanel.classList.remove("evidence-empty");
        videoPanel.innerHTML = `
          <div class="perception-form">
            <label><span>巡检点</span><select data-perception-point aria-label="权威巡检点"></select></label>
            <label><span>视频文件</span><input data-perception-video type="file" accept=".mp4,.avi,.mov,.mkv,video/*" /></label>
            <label><span>视频 ID（可选）</span><input data-perception-video-id type="text" placeholder="由服务自动生成" /></label>
            <button type="button" class="btn-primary btn-compact" data-perception-start>开始识别</button>
          </div>
          <div class="perception-runtime" aria-live="polite">
            <div class="perception-status-line"><span class="perception-badge idle" data-perception-status>待命</span><b data-perception-progress-text>0%</b></div>
            <progress data-perception-progress max="100" value="0"></progress>
            <dl class="perception-meta">
              <dt>阶段</dt><dd data-perception-stage>—</dd>
              <dt>Workflow</dt><dd data-perception-workflow>—</dd>
              <dt>Video Job</dt><dd data-perception-video-job>—</dd>
            </dl>
            <p class="perception-error" data-perception-error hidden></p>
          </div>`;
      }
      if (defectPanel && !defectPanel.dataset.perceptionMounted) {
        defectPanel.dataset.perceptionMounted = "true";
        defectPanel.classList.remove("evidence-empty");
        defectPanel.innerHTML = '<div class="perception-results" data-perception-results><p class="perception-placeholder">完成视频识别后显示关键帧与缺陷结果</p></div>';
      }
    });
  }

  function refreshPointOptions() {
    const points = getAuthoritativeInspectionPoints();
    document.querySelectorAll("[data-perception-point]").forEach((select) => {
      const selected = select.value;
      select.replaceChildren();
      const placeholder = document.createElement("option");
      placeholder.value = "";
      placeholder.textContent = points.length ? "选择巡检点" : "请先加载 Mission";
      select.append(placeholder);
      points.forEach((point) => {
        const option = document.createElement("option");
        option.value = point.point_id;
        option.textContent = point.point_id;
        select.append(option);
      });
      if (points.some((point) => point.point_id === selected)) select.value = selected;
    });
    document.querySelectorAll("[data-perception-start]").forEach((button) => {
      if (!controller.active && !controller.startInFlight) button.disabled = points.length === 0;
    });
  }

  function renderWorkflowState(state = {}) {
    const status = state.status || "idle";
    const stage = state.stage || status;
    const progress = Math.max(0, Math.min(100, Number(state.progress) || 0));
    document.querySelectorAll("[data-perception-status]").forEach((element) => {
      element.textContent = STATUS_LABELS[stage] || STATUS_LABELS[status] || status;
      element.className = `perception-badge ${status}`;
    });
    document.querySelectorAll("[data-perception-progress]").forEach((element) => {
      element.value = progress;
    });
    document.querySelectorAll("[data-perception-progress-text]").forEach((element) => {
      element.textContent = `${progress}%`;
    });
    document.querySelectorAll("[data-perception-stage]").forEach((element) => {
      element.textContent = stage || "—";
    });
    document.querySelectorAll("[data-perception-workflow]").forEach((element) => {
      element.textContent = state.workflow_job_id || controller.workflowJobId || "—";
    });
    document.querySelectorAll("[data-perception-video-job]").forEach((element) => {
      element.textContent = state.video_job_id || "—";
    });
    const errorText =
      typeof state.error === "string" ? state.error : state.error?.message || state.error?.code || "";
    document.querySelectorAll("[data-perception-error]").forEach((element) => {
      element.textContent = errorText;
      element.hidden = !errorText;
    });
    const busy = controller.active || controller.startInFlight;
    document.querySelectorAll("[data-perception-start]").forEach((button) => {
      button.disabled = busy || getAuthoritativeInspectionPoints().length === 0;
      button.textContent = busy ? "识别处理中…" : "开始识别";
    });
  }

  function appendText(parent, tag, className, text) {
    const element = document.createElement(tag);
    if (className) element.className = className;
    element.textContent = text;
    parent.append(element);
    return element;
  }

  function renderPerceptionResult(container, result) {
    container.replaceChildren();
    const summary = document.createElement("div");
    summary.className = "perception-result-summary";
    appendText(summary, "strong", "", result.inspection_point_id || "未知巡检点");
    appendText(
      summary,
      "span",
      "",
      `${STATUS_LABELS[result.status] || result.status || "未知状态"} · 关键帧 ${Number(result.target_frame_count) || 0}`
    );
    container.append(summary);

    const frames = Array.isArray(result.frames) ? result.frames : [];
    if (!frames.length) {
      appendText(container, "p", "perception-placeholder", "本次任务没有返回目标关键帧");
      return;
    }
    frames.forEach((frame) => {
      const card = document.createElement("article");
      card.className = `perception-frame-card ${frame.status || "unknown"}`;
      appendText(card, "h4", "", frame.frame_id || "未命名帧");
      appendText(card, "p", "perception-frame-meta", `${Number(frame.timestamp_ms) || 0} ms · ${frame.status || "unknown"}`);
      const defect = frame.defect_detection || {};
      appendText(card, "p", "perception-detection-count", `检测数量：${Number(defect.detection_count) || 0}`);
      const detections = Array.isArray(defect.detections) ? defect.detections : [];
      const list = document.createElement("ul");
      list.className = "perception-detection-list";
      detections.forEach((detection) => {
        const confidence = Number(detection.confidence);
        const confidenceText = Number.isFinite(confidence) ? ` ${(confidence * 100).toFixed(1)}%` : "";
        appendText(list, "li", "", `${detection.class_name || "unknown"}${confidenceText}`);
      });
      if (detections.length) card.append(list);

      const annotated = defect.annotated_image_url;
      const accessible = accessibleAnnotatedUrl(annotated);
      if (accessible) {
        const media = document.createElement("div");
        media.className = "perception-annotated-media";
        const thumbnail = document.createElement("img");
        thumbnail.className = "perception-annotated-thumbnail";
        thumbnail.src = accessible;
        thumbnail.alt = `${frame.frame_id || "关键帧"} 标注结果`;
        thumbnail.loading = "lazy";
        thumbnail.width = 160;
        media.append(thumbnail);
        const link = appendText(media, "a", "perception-annotated-link", "查看标注图");
        link.href = accessible;
        link.target = "_blank";
        link.rel = "noopener";
        const loadError = appendText(media, "p", "perception-media-note", "标注图加载失败，检测结果仍可查看");
        loadError.hidden = true;
        thumbnail.addEventListener("error", () => {
          thumbnail.hidden = true;
          link.hidden = true;
          loadError.hidden = false;
        });
        card.append(media);
      } else if (typeof annotated === "string" && annotated) {
        appendText(card, "p", "perception-media-note", "标注图未提供可访问的 8001 代理地址");
      }
      if (frame.error?.message) appendText(card, "p", "perception-error", frame.error.message);
      container.append(card);
    });
  }

  function renderAllResults(result) {
    document.querySelectorAll("[data-perception-results]").forEach((container) => {
      renderPerceptionResult(container, result);
    });
    activateEvidenceTab("defect");
  }

  function showControllerError(error) {
    renderWorkflowState({ status: "failed", stage: "failed", progress: 0, error: error.message });
  }

  const controller = new PerceptionWorkflowController({
    onState: renderWorkflowState,
    onResult: renderAllResults,
    onError: showControllerError,
  });

  function bindActions() {
    document.querySelectorAll(ROOT_SELECTOR).forEach((root) => {
      root.querySelector("[data-perception-start]")?.addEventListener("click", async () => {
        const inspectionPointId = root.querySelector("[data-perception-point]")?.value || "";
        const video = root.querySelector("[data-perception-video]")?.files?.[0];
        const videoId = root.querySelector("[data-perception-video-id]")?.value?.trim() || "";
        const { missionId, missionSha256 } = getCurrentMissionBinding();
        if (!inspectionPointId) {
          showControllerError(new Error("请选择当前 Mission 中的巡检点"));
          return;
        }
        if (!video) {
          showControllerError(new Error("请选择本地视频文件"));
          return;
        }
        try {
          await controller.start({
            video,
            inspectionPointId,
            missionId,
            missionSha256,
            videoId,
          });
        } catch (_) {
          // Controller has already rendered a structured user-facing error.
        }
      });
    });
  }

  function resetForMissionChange() {
    controller.cancelPolling();
    refreshPointOptions();
    renderWorkflowState({ status: "idle", stage: "idle", progress: 0, error: null });
  }

  function init() {
    mountPanels();
    bindActions();
    refreshPointOptions();
    renderWorkflowState({ status: "idle", stage: "idle", progress: 0, error: null });
    lastMissionVersion = Number(window.MissionStore?.version ?? -1);
    window.MissionStore?.subscribeMission?.(() => {
      const version = Number(window.MissionStore?.version ?? -1);
      if (version !== lastMissionVersion || !window.MissionStore?.getMission?.()) {
        lastMissionVersion = version;
        resetForMissionChange();
      } else {
        refreshPointOptions();
      }
    });
    window.addEventListener("pagehide", () => controller.cancelPolling(), { once: true });
  }

  window.__PerceptionEvidenceTestHooks = {
    PerceptionWorkflowController,
    accessibleAnnotatedUrl,
    getAuthoritativeInspectionPoints,
    getCurrentMissionBinding,
    renderPerceptionResult,
    terminalStates: TERMINAL_STATES,
  };

  function safeInit() {
    try {
      init();
    } catch (error) {
      console.error("Perception workflow panel initialization failed", error);
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", safeInit, { once: true });
  } else {
    safeInit();
  }
})();
