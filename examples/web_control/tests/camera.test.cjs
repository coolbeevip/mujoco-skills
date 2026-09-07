const { test } = require("node:test");
const assert = require("node:assert/strict");
const { draggedCamera } = require("../camera.js");
const start = { azimuth: 0, elevation: -45, distance: 2, pan: [0, 0, 0] };
test("右键平移改变中心但保留角度和距离，不修改原始相机", () => {
  const moved = draggedCamera(start, 100, 0, 600, true);
  assert.equal(moved.azimuth, 0);
  assert.equal(moved.elevation, -45);
  assert.equal(moved.distance, 2);
  assert.ok(moved.pan[1] > 0);
  assert.deepEqual(start.pan, [0, 0, 0]);
});
test("平移方向随视角变化，垂直拖动沿屏幕上方向，远景平移更大", () => {
  const moved = draggedCamera({ ...start, azimuth: 90 }, 100, 0, 600, true);
  assert.ok(moved.pan[0] < 0);
  const down = draggedCamera(start, 0, 100, 600, true);
  assert.ok(down.pan[2] > 0);
  const far = draggedCamera({ ...start, distance: 4 }, 0, 100, 600, true);
  assert.equal(far.pan[2], 2 * down.pan[2]);
});
test("左键旋转保留已有平移，平移数值限制在服务端允许范围", () => {
  const moved = draggedCamera(start, 10, 10, 600, false);
  assert.equal(moved.azimuth, 4);
  assert.deepEqual(moved.pan, start.pan);
  assert.ok(draggedCamera(start, 1e8, 1e8, 600, true).pan.every(v => Math.abs(v) <= 20));
});
