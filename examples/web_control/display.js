"use strict";

// object-fit: contain 下，先求出 16:9 图片实际占用的 CSS 宽度，再乘屏幕像素比。
// 例如 1000 CSS 像素、2 倍屏需要约 2000 像素的原图；这里封顶 1920，限制开销。
function renderWidth(cssWidth, cssHeight, pixelRatio, quality = "auto") {
  if (quality === "720") return 1280;
  if (quality === "1080") return 1920;
  const fittedWidth = Math.min(cssWidth, cssHeight * 16 / 9);
  const needed = fittedWidth * Math.max(1, pixelRatio || 1);
  return [640, 960, 1280, 1600, 1920].find(width => width >= needed) || 1920;
}

if (typeof module !== "undefined") module.exports = { renderWidth };
