# Learning-page checks / 学习页面检查

From the repository root / 在仓库根目录运行：

```bash
MPLCONFIGDIR=/tmp/irsim-doc-mpl .venv/bin/pytest docs/tests/ -o addopts=""
NODE_PATH=/path/to/node_modules node --test docs/tests/test_playground_ui.cjs
```

The Python checks execute the documented example, the quick-start scene, and the playground adapter against the real simulator. They check native Matplotlib pixels, compound geometry, both LiDAR models, multi-robot kinematics, batching without changing integration or sampling, collision handling, input limits, reset, and reproducible source bundles.

Python 检查会使用真实仿真器执行文档示例、快速入门场景和在线实验适配器，检查原生 Matplotlib 像素、复合几何、两类 LiDAR、多机器人运动学、分组运行不改变积分与采样、碰撞处理、输入限制、重置及源码包的可复现性。

DOM checks require `jsdom`, available through `NODE_PATH`. They use a mock worker to check both languages, lazy loading, display of returned images and states, bounded replay and image URL cleanup, YAML behavior controls, reset, cancellation, and recovery from invalid YAML. These tests do not execute Python and do not replace visual inspection at mobile and desktop widths in both themes.

DOM 检查需要通过 `NODE_PATH` 提供 `jsdom`，使用模拟 worker 检查双语界面、按需加载、返回的图片与状态显示、有容量限制的回放及图片 URL 释放、YAML 行为控制、重置、取消和错误 YAML 恢复。这些测试不执行 Python，也不能代替在移动端、桌面端以及明暗主题下的视觉检查。

For a real WebAssembly smoke test, install the `pyodide` npm package at the version pinned in `docs/playground_build.py`, build the HTML docs, then run:

如需执行真实 WebAssembly 冒烟测试，请安装 `docs/playground_build.py` 中锁定版本的 `pyodide` npm 包，构建 HTML 文档后运行：

```bash
NODE_PATH=/path/to/node_modules \
IRSIM_BUNDLE=docs/build/html/_static/playground \
PYODIDE_CACHE=/tmp/irsim-pyodide-cache \
node docs/tests/pyodide_smoke.cjs
```

This test needs network access to download Pyodide packages and `loguru`. It loads the exact source ZIP emitted by the documentation build through the same runtime module used by the browser worker, then checks real `make()` / `step()` results, native PNG rendering, compound geometry, both LiDAR models, collision, and invalid YAML. This is separate from browser-specific worker/CDN and visual checks.

此测试需要联网下载 Pyodide 依赖与 `loguru`。它使用浏览器 worker 共用的运行时模块，加载文档构建生成的源码 ZIP，检查真实 `make()` / `step()` 结果、原生 PNG 渲染、复合几何、两类 LiDAR、碰撞及无效 YAML；浏览器 worker、CDN 和视觉检查仍需单独进行。
