const { test } = require("node:test");
const assert = require("node:assert/strict");
const { renderWidth } = require("../display.js");

test("自动档按图像实际区域与像素比选择上档分辨率", () => {
  assert.equal(renderWidth(1000, 562.5, 1), 1280);
  assert.equal(renderWidth(1000, 562.5, 2), 1920);
  assert.equal(renderWidth(350, 300, 2), 960);
  assert.equal(renderWidth(350, 300, 3), 1280);
  assert.equal(renderWidth(1400, 400, 1), 960); // 宽屏两侧留白不计入图像宽度。
  assert.equal(renderWidth(5000, 3000, 3), 1920);
});

test("固定画质不受窗口尺寸或像素比影响", () => {
  assert.equal(renderWidth(300, 200, 1, "1080"), 1920);
  assert.equal(renderWidth(2000, 1200, 2, "720"), 1280);
});
