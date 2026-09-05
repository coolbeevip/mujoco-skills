"use strict";

function describeDecision(item) {
  const labels = { forward: "向前走", left: "向左转身", right: "向右转身", wait: "停步观察", done: "视觉判断任务已完成", abort: "停止尝试", sit: "坐下", stand: "站起", ground_pick: "俯身", kick_left: "左脚踢球", kick_right: "右脚踢球", roulade: "单次翻滚", crouch: "蹲伏", stop: "停止运动" };
  const units = { forward: " 厘米", left: "°", right: "°", wait: " 秒" };
  Object.assign(labels, {head_down: "低头", head_up: "抬头", head_reset: "头部回正"});
  const label = labels[item.action] || item.action;
  const legacy = item.amount === undefined;
  const target = legacy ? `${label}持续 ${item.duration_s} 秒（旧版指令）` : `${label}${units[item.action] ? ` ${item.amount}${units[item.action]}` : ""}`;
  const run = item.execution;
  let result = legacy ? "旧记录没有实际运动反馈" : "模型的视觉判断，不代表经过位置测量验证";
  if (run) {
    const status = { running: run.phase === "settling" ? "停步复核中" : "执行中", reached: "动作已完成", incomplete: "未达到动作目标", cancelled: "动作已中止" }[run.status] || run.status;
    let actual = "";
    if (item.action.startsWith("head_") && Number.isFinite(run.camera_pitch_deg)) {
      actual = `镜头实际${run.camera_pitch_deg < 0 ? "向下" : "向上"} ${Math.abs(run.camera_pitch_deg)}°（相对水平）`;
    } else if (item.action === "left" || item.action === "right") {
      actual = `实际${run.actual_angle_deg < 0 ? "右转" : "左转"} ${Math.abs(run.actual_angle_deg)}° · 位置变化 ${run.drift_cm} 厘米`;
    } else if (item.action === "forward") {
      actual = `实际${run.actual_distance_cm < 0 ? "后退" : "前进"} ${Math.abs(run.actual_distance_cm)} 厘米`;
    } else if (run.action_success_verified === false) {
      actual = "策略执行进度不等于动作效果验证";
    } else if (run.elapsed_s !== undefined) {
      actual = `已观察 ${run.elapsed_s} 秒`;
    }
    const checkpoint = run.status === "checkpoint";
    result = [checkpoint ? "中途复查，等待新决策" : status, actual,
      checkpoint ? `原计划尚余 ${run.remaining_plan_cm} 厘米（非目标距离）` : ""].filter(Boolean).join(" · ");
  }
  const range = item.proximity;
  const distanceText = range ? (range.visible ? `深度测距 ${range.surface_distance_cm} 厘米 · 方向偏差 ${range.bearing_deg}° · ${range.seated_arrival_verified ? "坐姿到达复核有效" : range.near ? "已进入近距离" : "未满足站姿到达门槛"}` : "深度测距：目标不可见") : "";
  return {
    title: `第 ${item.observation} 次观察 · ${target}`,
    evidence: item.evidence,
    result,
    assessment: [`${item.target_visible ? "已看到目标" : "未看到目标"} · 模型自评 ${Math.round(item.confidence * 100)}%`, distanceText].filter(Boolean).join(" · "),
  };
}

if (typeof module !== "undefined") module.exports = { describeDecision };
