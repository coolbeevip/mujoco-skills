# 场景图片

[chi-welcome-logo.svg](chi-welcome-logo.svg) 由项目维护者提供，用于办公区沙发上方的 60 × 60 厘米挂画；版权归原权利人所有，不声明额外授权。

[chi-welcome-logo.png](chi-welcome-logo.png) 是原 SVG 的 1024 × 1024 渲染副本，供 MuJoCo 加载。运行样例不需要 SVG 转换工具。外部预览的墙壁半透明不影响画框和 Logo 的不透明度；头部相机仍使用真实遮挡。

更新 SVG 后，安装了 librsvg 的开发环境可从仓库根目录重新生成纹理，再重启服务并重新加载办公场景：

```sh
rsvg-convert -w 1024 -h 1024 examples/web_control/assets/chi-welcome-logo.svg -o examples/web_control/assets/chi-welcome-logo.png
```
