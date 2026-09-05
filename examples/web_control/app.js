"use strict";

const $ = selector => document.querySelector(selector);
let scenes = [], token = "", state = {}, busy = false, connected = false;
let frameId = -1, frameUrl = "", lastError = "";
let headFrameId = -1, headFrameUrl = "";
let camera = { azimuth: 135, elevation: -20, distance: 0.85 };
let sizeCandidate = "", sizeSince = 0, sizeAttempt = "";
let historyKey = "";
const taskPhases = { settling: "等待站稳", waiting: "停步观察", observing: "读取头部图像", thinking: "模型正在判断", moving: "执行移动", skill: "执行原子动作 / 恢复", confirming: "停止后再次观察", completed: "模型视觉确认完成", cancelled: "任务已中止", failed: "任务失败" };
function taskActive() { return Boolean(state.navigation && !["completed", "cancelled", "failed"].includes(state.navigation.phase)); }
taskPhases.head = "调整头部 / 重新观察";
const stages = { idle: "平衡 / 移动", sitting: "坐下中", seated: "保持坐姿", rising: "站起中", executing: "执行动作", recovering: "恢复平衡" };

function feedback(title, detail, mapping = "") {
  $("#feedback-title").textContent = title;
  $("#feedback-detail").textContent = detail;
  $("#feedback-code").textContent = mapping;
}

function explain(message) {
  const messages = {
    "command updated": "移动目标已更新；控制器将持续执行，直到收到下一条有效移动指令。",
    "movement cleared; current action continues": "移动目标已清零；当前动作仍继续。需要立即冻结时请暂停仿真。",
    "standing up": "已开始站起，随后进入恢复平衡阶段。",
    "ignored: action/recovery in progress": "动作或恢复阶段尚未结束，本次请求未执行，也不会排队。",
    "ignored: stop with space/s before starting an action": "请先点击停止运动，等待站稳后再次触发动作。",
    "ignored: wait until upright and settled": "机器人尚未直立或站稳，请稍后重试。",
  };
  if (message.startsWith("started:")) return `已启动 ${message.slice(9)} 策略；请观察实际物理结果。`;
  return messages[message] || message;
}

async function request(path, body) {
  const response = await fetch(path, {
    ...(body ? { method: "POST", headers: { "Content-Type": "application/json", "X-Control-Token": token }, body: JSON.stringify(body) } : {}),
    signal: AbortSignal.timeout(35000), cache: "no-store",
  });
  const data = await response.json();
  if (response.status === 403) token = "";
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function controls() {
  const loaded = Boolean(state.scene);
  const matches = state.scene === $("#scene-select").value;
  const active = taskActive();
  $("#task-start").disabled = busy || !connected || !loaded || !matches || active || Boolean(state.error) || !state.model_config?.configured || !$("#task-input").value.trim();
  $("#task-cancel").disabled = busy || !connected || !active;
  $("#task-cancel-global").hidden = !active;
  $("#task-cancel-global").disabled = busy || !connected;
  $("#manual-task-note").hidden = !active;
  $("#task-input").disabled = active;
  $("#view-select").disabled = busy || !connected || !loaded;
  $("#load-scene").disabled = busy || !connected;
  $("#scene-select").disabled = busy || !connected;
  $("#pause").disabled = busy || !connected || !loaded || Boolean(state.error);
  $("#reset").disabled = busy || !connected || !loaded;
  $("#pause").textContent = state.paused ? "继续仿真" : "暂停仿真";
  document.querySelectorAll(".action-button").forEach(button => {
    button.disabled = busy || !connected || !matches || state.paused || Boolean(state.error) || active;
  });
}

function update(next) {
  const changed = state.generation !== next.generation;
  state = next;
  if (changed) {
    frameId = -1;
    camera = { azimuth: 135, elevation: -20, distance: 0.85 };
    $("#live-frame").hidden = true;
    $("#head-frame").hidden = true;
    $("#head-frame").removeAttribute("src");
    $("#head-stamp").hidden = true;
    headFrameId = -1;
    if (headFrameUrl) URL.revokeObjectURL(headFrameUrl);
    headFrameUrl = "";
    sizeAttempt = "";
    $("#render-size").textContent = "等待画面";
  }
  const status = state.error ? "异常 · 请重置" : !state.scene ? "未加载" : state.paused ? "已暂停" : "运行中";
  $("#connection-text").textContent = connected ? "本机服务已连接" : "连接中断";
  $(".viewport-state").textContent = connected ? status : "连接中断 · 画面可能过期";
  $("#sim-time").textContent = state.time_s == null ? "—" : `${state.time_s.toFixed(2)} s`;
  $("#sim-policy").textContent = state.policy || "—";
  $("#sim-stage").textContent = stages[state.stage] || "未加载";
  $("#viewport-scene").textContent = state.scene ? `MICRODUCK / ${state.scene.toUpperCase()}` : "请选择场景";
  $(".empty-view").hidden = Boolean(state.scene);
  if (state.error && state.error !== lastError) feedback("仿真异常", state.error, "请重置场景");
  lastError = state.error;
  $("#runtime-note").textContent = state.error || explain(state.message || "请选择场景");
  if (!state.scene || !state.head_frame_available) {
    $("#head-status").hidden = false;
    $("#head-status").textContent = state.scene ? "头部画面尚未就绪，请确认后端服务已重启" : "加载场景后显示头部画面";
  }
  $("#view-select").value = state.view || "external";
  const head = state.view === "head";
  $("#viewport").dataset.view = head ? "head" : "external";
  $("#view-help").textContent = head ? "头部固定视角 · 随机器人运动" : "拖动后松开旋转 · 滚轮缩放";
  $("#viewport").setAttribute("aria-label", head ? "机器人头部摄像头画面" : "仿真画面；拖动旋转，滚轮缩放；键盘左右键旋转，加减键缩放");
  const config = state.model_config;
  $("#model-status").textContent = config?.configured ? `${config.provider} / ${config.model} · 开始任务后向该服务发送任务、头部图像及决策历史` : "尚未配置模型：在服务端设置 VLM_PROVIDER 和 VLM_MODEL 后重启服务。密钥仅在本机配置。";
  const navigation = state.navigation;
  $("#task-phase").textContent = navigation ? (taskPhases[navigation.phase] || navigation.phase) : "未开始";
  $("#task-reason").textContent = navigation ? `${navigation.task}：${navigation.reason}` : "尚无模型观察结果";
  const history = navigation?.history || [];
  $("#task-count").textContent = `${history.length} 条`;
  const key = JSON.stringify(history);
  if (key !== historyKey) {
    historyKey = key;
    // 模型文本是不可信输入，只作为文字显示，绝不解析成 HTML 或代码。
    // 仅将展示顺序倒置，不改变状态中的历史顺序和原始观察编号。
    $("#task-history").replaceChildren(...[...history].reverse().map(item => {
      const row = document.createElement("li");
      const description = describeDecision(item);
      for (const [key, tag] of [["title", "strong"], ["evidence", "p"], ["result", "p"], ["assessment", "small"]]) {
        const line = document.createElement(tag);
        line.className = `decision-${key}`;
        line.textContent = description[key];
        row.append(line);
      }
      return row;
    }));
  }
  controls();
}

async function command(op, extra = {}, quiet = false) {
  if (busy) return;
  busy = true;
  if (op === "load" || op === "reset") $("#load-scene").textContent = "正在加载…";
  controls();
  try {
    const result = await request("/api/command", { op, generation: state.generation, ...extra });
    update(result.state);
    if (!quiet) feedback(result.message.startsWith("ignored:") ? "请求未执行" : "操作已响应", explain(result.message), extra.action || op);
  } catch (error) {
    feedback("操作未完成", error.message);
  } finally {
    busy = false;
    $("#load-scene").textContent = "加载场景";
    controls();
  }
}

async function syncRenderSize() {
  if (!state.scene || state.error || busy || document.hidden) return;
  const rect = $("#viewport").getBoundingClientRect();
  const width = renderWidth(rect.width, rect.height, window.devicePixelRatio, $("#quality-select").value);
  const key = `${state.generation}:${width}`;
  if (sizeCandidate !== key) {
    sizeCandidate = key;
    sizeSince = performance.now();
  }
  // 窗口停止变化 350 ms 后才重建；失败请求不反复重试，避免持续占用渲染线程。
  if (width === state.render_width || key === sizeAttempt || performance.now() - sizeSince < 350) return;
  sizeAttempt = key;
  await command("resolution", { width }, true);
}

$("#quality-select").addEventListener("change", () => { sizeAttempt = ""; });
$("#live-frame").addEventListener("load", () => {
  const image = $("#live-frame");
  $("#render-size").textContent = `${image.naturalWidth} × ${image.naturalHeight}`;
});

$(".camera-pip").addEventListener("toggle", () => {
  // 重新展开时获取最新帧；折叠只停止下载小窗图像，不影响仿真与视觉任务。
  if ($(".camera-pip").open) headFrameId = -1;
});

async function refreshHeadFrame() {
  if (!state.scene || !state.head_frame_available || !$(".camera-pip").open || headFrameId === state.frame_id) return;
  const generation = state.generation;
  try {
    const response = await fetch(`/api/head-frame?id=${state.frame_id}`, { cache: "no-store", signal: AbortSignal.timeout(3000) });
    if (!response.ok) throw new Error("头部画面读取失败");
    const blob = await response.blob();
    // 换场景期间迟到的图像直接丢弃，不能盖住新场景的小窗。
    if (generation !== state.generation || Number(response.headers.get("X-Scene-Generation")) !== generation) return;
    const url = URL.createObjectURL(blob);
    const image = $("#head-frame");
    const previous = headFrameUrl;
    image.src = url;
    headFrameUrl = url;
    if (previous) URL.revokeObjectURL(previous);
    image.hidden = false;
    $("#head-status").hidden = true;
    headFrameId = Number(response.headers.get("X-Frame-Id"));
    $("#head-stamp").textContent = `HEAD / 640 × 360 · 帧 ${headFrameId}`;
    $("#head-stamp").hidden = false;
  } catch (error) {
    if (generation !== state.generation) return;
    $("#head-frame").hidden = true;
    $("#head-stamp").hidden = true;
    $("#head-status").hidden = false;
    $("#head-status").textContent = `${error.message}，正在重试`;
  }
}

function actionButton(action) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = `action-button${action.id === "stop" ? " stop" : ""}`;
  button.title = `策略：${action.policy}`;
  const label = document.createElement("span");
  label.textContent = action.label;
  const key = document.createElement("kbd");
  key.textContent = action.key.toUpperCase();
  key.setAttribute("aria-label", `对应终端按键 ${action.key}`);
  button.append(label);
  if (action.key) button.append(key);
  // 页面只发送动作 ID。服务端将 forward 映射为 w，再交给 Behaviors；
  // 浏览器不计算关节角，也不直接写 MuJoCo 的物理状态。
  button.addEventListener("click", () => command("action", { action: action.id }));
  return button;
}

function renderScene() {
  const scene = scenes.find(item => item.id === $("#scene-select").value);
  if (!scene) return;
  $("#scene-title").textContent = scene.title;
  $("#scene-description").textContent = scene.description;
  $("#action-count").textContent = `${scene.actions.length} 个按钮`;
  $("#movement-actions").replaceChildren(...scene.actions.filter(a => a.group === "movement").map(actionButton));
  $("#skill-actions").replaceChildren(...scene.actions.filter(a => a.group === "skills").map(actionButton));
  controls();
}

$("#scene-select").addEventListener("change", () => {
  renderScene();
  feedback("已选择场景", "点击加载场景后生效；当前仿真尚未切换。");
});
$("#load-scene").addEventListener("click", () => command("load", { scene: $("#scene-select").value }));
$("#pause").addEventListener("click", () => command(state.paused ? "resume" : "pause"));
$("#reset").addEventListener("click", () => command("reset"));
$("#task-input").addEventListener("input", controls);
$("#task-start").addEventListener("click", () => command("task_start", { task: $("#task-input").value.trim() }));
$("#task-cancel").addEventListener("click", () => command("task_cancel"));
$("#task-cancel-global").addEventListener("click", () => command("task_cancel"));
const controlTabs = [...document.querySelectorAll('[role="tab"]')];
function selectControlTab(tab) {
  controlTabs.forEach(button => {
    const selected = button === tab;
    button.setAttribute("aria-selected", String(selected));
    button.tabIndex = selected ? 0 : -1;
    document.getElementById(button.getAttribute("aria-controls")).hidden = !selected;
  });
}
controlTabs.forEach((tab, index) => {
  tab.addEventListener("click", () => selectControlTab(tab));
  tab.addEventListener("keydown", event => {
    let next;
    if (event.key === "ArrowRight" || event.key === "ArrowLeft") next = controlTabs[1 - index];
    else if (event.key === "Home") next = controlTabs[0];
    else if (event.key === "End") next = controlTabs[1];
    else return;
    event.preventDefault();
    selectControlTab(next);
    next.focus();
  });
});
$("#view-select").addEventListener("change", event => command("view", { view: event.target.value }));
$("#layout-select").addEventListener("change", event => { document.body.dataset.layout = event.target.value; });
$("#density-select").addEventListener("change", event => { document.body.dataset.density = event.target.value; });

// 相机只改变观察方式，不影响机器人。拖动结束后提交角度，滚轮提交距离。
let drag = null, zoomTimer;
$("#viewport").addEventListener("pointerdown", event => {
  if (!state.scene || !connected || busy || state.view === "head") return;
  drag = { x: event.clientX, y: event.clientY, ...camera };
  event.currentTarget.setPointerCapture(event.pointerId);
});
$("#viewport").addEventListener("pointerup", event => {
  if (!drag) return;
  camera.azimuth = ((drag.azimuth + (event.clientX - drag.x) * 0.4 + 540) % 360) - 180;
  camera.elevation = Math.max(-85, Math.min(-5, drag.elevation + (event.clientY - drag.y) * 0.3));
  drag = null;
  command("camera", camera);
});
$("#viewport").addEventListener("pointercancel", () => { drag = null; });
$("#viewport").addEventListener("wheel", event => {
  if (!state.scene || !connected || busy || state.view === "head") return;
  event.preventDefault();
  camera.distance = Math.max(0.25, Math.min(3, camera.distance * Math.exp(event.deltaY * 0.001)));
  clearTimeout(zoomTimer);
  zoomTimer = setTimeout(() => command("camera", camera), 120);
}, { passive: false });
$("#viewport").addEventListener("keydown", event => {
  if (!state.scene || !connected || busy || state.view === "head") return;
  const deltas = { ArrowLeft: -10, ArrowRight: 10 };
  if (event.key in deltas) camera.azimuth = ((camera.azimuth + deltas[event.key] + 540) % 360) - 180;
  else if (event.key === "+" || event.key === "=") camera.distance = Math.max(0.25, camera.distance * 0.9);
  else if (event.key === "-") camera.distance = Math.min(3, camera.distance * 1.1);
  else return;
  event.preventDefault();
  command("camera", camera);
});

async function poll() {
  try {
    if (!token) {
      const data = await request("/api/catalog");
      scenes = data.scenes;
      token = data.token;
      const selected = $("#scene-select").value;
      $("#scene-select").replaceChildren(...scenes.map(scene => new Option(scene.title, scene.id)));
      if (scenes.some(scene => scene.id === selected)) $("#scene-select").value = selected;
      renderScene();
    }
    const next = await request("/api/state");
    connected = true;
    update(next);
    await syncRenderSize();
    if (state.scene && frameId !== state.frame_id) {
      const generation = state.generation;
      const requestedFrameId = state.frame_id;
      const response = await fetch(`/api/frame?id=${state.frame_id}`, { cache: "no-store", signal: AbortSignal.timeout(3000) });
      if (!response.ok) throw new Error("仿真画面读取失败");
      const blob = await response.blob();
      if (generation !== state.generation) return;
      const returnedGeneration = response.headers.get("X-Scene-Generation");
      if (returnedGeneration !== null && Number(returnedGeneration) !== generation) return;
      const url = URL.createObjectURL(blob);
      $("#live-frame").src = url;
      $("#live-frame").hidden = false;
      if (frameUrl) URL.revokeObjectURL(frameUrl);
      frameUrl = url;
      frameId = requestedFrameId;
    }
    await refreshHeadFrame();
  } catch (error) {
    connected = false;
    token = "";
    headFrameId = -1;
    $("#connection-text").textContent = "连接中断";
    $(".viewport-state").textContent = "连接中断 · 画面可能过期";
    $("#head-status").hidden = false;
    $("#head-status").textContent = "连接中断 · 画面可能过期";
    feedback("无法连接仿真服务", `${error.message}。连接恢复后不会自动继续运动。`);
    controls();
  } finally {
    setTimeout(poll, connected ? 100 : 1500);
  }
}

// 只有前台页面续约。关闭页面或切到后台，服务端 3 秒后暂停并清除移动目标。
setInterval(() => {
  if (token && connected && !document.hidden) request("/api/heartbeat", {}).catch(() => {});
}, 750);
controls();
poll();
