// 把屏幕拖动换成观察中心的世界坐标偏移。距离越远，同样像素对应的平移越大。
function draggedCamera(start, dx, dy, height, pan) {
  if (!pan) return {
    ...start,
    azimuth: ((start.azimuth + dx * 0.4 + 540) % 360) - 180,
    elevation: Math.max(-85, Math.min(-5, start.elevation + dy * 0.3)),
  };
  const a = start.azimuth * Math.PI / 180, e = start.elevation * Math.PI / 180;
  const scale = 2 * start.distance * Math.tan(Math.PI / 8) / Math.max(height, 1);
  const right = [Math.sin(a), -Math.cos(a), 0];
  const up = [-Math.sin(e) * Math.cos(a), -Math.sin(e) * Math.sin(a), Math.cos(e)];
  return {
    ...start,
    pan: (start.pan || [0, 0, 0]).map((v, i) =>
      Math.max(-20, Math.min(20, v + scale * (-dx * right[i] + dy * up[i])))),
  };
}
if (typeof module !== "undefined") module.exports = { draggedCamera };
