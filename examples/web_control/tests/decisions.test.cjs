const {test} = require('node:test');
const assert = require('node:assert/strict');
const {describeDecision} = require('../decisions.js');

test('高层搜索显示观察点和控制阶段，不冒充任务完成', () => {
  const row=describeDecision({action:'search_103',amount:0,observation:1,confidence:.8,target_visible:false,execution:{status:'reached',search_phase:'target_visible',point:1,segments:[{},{}]}});
  assert.match(row.title,/搜索会议室 103/);
  assert.match(row.result,/发现目标，交回模型判断/);
  assert.match(row.result,/观察点 2/);
  assert.doesNotMatch(row.result,/undefined|任务已完成/);
});

test('区分模型自评与深度测距', () => {
  const row=describeDecision({action:'sit',amount:0,observation:2,confidence:.99,target_visible:true,proximity:{visible:true,surface_distance_cm:25.1,bearing_deg:-2,near:true}});
  assert.match(row.assessment,/深度测距 25.1 厘米/);
  assert.match(row.assessment,/已进入近距离/);
  assert.match(row.assessment,/模型自评 99%/);
});

test('长前进中途复查不宣称整段完成', () => {
  const row=describeDecision({observation:1,action:'forward',amount:50,confidence:0.9,execution:{status:'checkpoint',actual_distance_cm:29,remaining_plan_cm:21}});
  assert.match(row.title,/50 厘米/);
  assert.match(row.result,/中途复查/);
  assert.match(row.result,/实际前进 29 厘米/);
  assert.match(row.result,/非目标距离/);
  assert.doesNotMatch(row.result,/动作已完成/);
});

test('原子动作显示中文名称且不伪造动作效果', () => {
  for (const [action, label] of Object.entries({sit:'坐下',stand:'站起',ground_pick:'俯身',kick_left:'左脚踢球',kick_right:'右脚踢球',roulade:'单次翻滚',crouch:'蹲伏'})) {
    const item=describeDecision({observation:1,action,amount:0,confidence:0.9,execution:{status:'reached',action_success_verified:false}});
    assert.match(item.title, new RegExp(label));
    assert.match(item.result, /不等于动作效果验证/);
    assert.doesNotMatch(item.title, /undefined|秒/);
  }
});

test('转身记录分别说明计划角度、实际结果和模型自评', () => {
  const item = describeDecision({observation: 2, action:'left', amount:30, target_visible:false, confidence:0.1, evidence:'寻找蓝色圆柱', execution:{status:'reached',actual_angle_deg:26.2,drift_cm:3.1}});
  assert.match(item.title, /向左转身 30°/);
  assert.match(item.result, /实际左转 26.2°/);
  assert.match(item.result, /3.1 厘米/);
  assert.match(item.assessment, /模型自评 10%/);
  assert.doesNotMatch(item.title, /秒/);
});
test('前进目标使用厘米并区分未完成', () => {
  const item=describeDecision({observation:1,action:'forward',amount:10,confidence:0.5,execution:{status:'incomplete',actual_distance_cm:0.8}});
  assert.match(item.title, /10 厘米/);
  assert.match(item.result, /未达到动作目标/);
  assert.match(item.result, /0.8 厘米/);
});
test('旧服务记录不捏造实际角度', () => {
  const item=describeDecision({observation:1,action:'left',duration_s:0.5,confidence:0.1});
  assert.match(item.title, /0.5 秒（旧版指令）/);
  assert.match(item.result, /没有实际运动反馈/);
});
