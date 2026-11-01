# 通过 MCP 让本地 Gemma 控制 IR-SIM 小机器人

[English](README.md) · [架构与一致性](architecture.zh-CN.md) · [性能实测](https://github.com/hanruihua/ir-sim/pull/386)

这是可选的真实 Model Context Protocol 服务和一个本地模型 Host。Host 使用官方 Python MCP 客户端完成握手、发现工具、读取观测，将任务交给开启 thinking 的 Gemma，再把模型原生返回的工具调用发送给仿真。机器人实际运动由 IR-SIM 现有控制器完成，工具返回真实观测。

示例任务是绕开障碍，先到 blue，再到 green。中间标有 **Obstacle / no entry** 的浅灰矩形是有意设置的碰撞几何体，不是渲染错误。原来的默认纯黑填充已换成浅灰并添加标注。

## 安装与运行

在包含本次改动的仓库根目录执行：

```powershell
uv sync --locked --extra mcp --group test --group lint
.\.venv\Scripts\python.exe usage/27mcp_robot/demo.py --display --realtime --record-window --output artifacts/mcp-demo/my-run
```

默认模型接口为 `http://127.0.0.1:8000/v1`，模型名为 `gemma-4-e2b-it`。这里使用 OpenAI 兼容的 HTTP 协议，不调用 OpenAI 云 API，也不需要 OpenAI 密钥。可通过 `--base-url`、`--model` 或 `IRSIM_GEMMA_URL`、`IRSIM_GEMMA_MODEL` 修改；如本地服务要求认证，设置 `IRSIM_GEMMA_API_KEY`，不要把凭据写进 URL。

实测复用了本机已有的 Gemma 4 E2B 和 vLLM 0.22.1。原容器配置未开启原生工具调用，测试临时容器增加了以下启动参数：

```text
--enable-auto-tool-choice --tool-call-parser gemma4 --reasoning-parser gemma4
```

这些是模型服务启动参数，客户端不能仅靠请求字段替服务启用。不同 vLLM 版本支持的 parser 名称需以安装版本为准。Host 要求原生 `tool_calls`，并检查解析后的 `reasoning` 或 `reasoning_content`。每次模型请求都发送 `chat_template_kwargs: {enable_thinking: true}`；超出 token 上限、缺少工具调用、一次返回多个调用或选择未暴露工具都会使该次运行失败，不会静默关闭 thinking 或换成脚本路线。参考 [Gemma thinking 文档](https://ai.google.dev/gemma/docs/capabilities/thinking)及 [vLLM 工具调用文档](https://docs.vllm.ai/en/latest/features/tool_calling/)。

可视化使用 Matplotlib 的 TkAgg 后端，需要桌面与 Tk。`--record-window` 使用附带 ffmpeg 的 Windows `gdigrab`，只采集带唯一标题的仿真窗口，完成 MP4 后进行完整解码验证，并提取终点截图。该录像选项目前支持 Windows；不加录像选项可在其他平台运行无窗口的 MCP/模型测试。录制前关闭其他示例窗口。视频中的渲染、雷达和轨迹均来自 IR-SIM，并保留实际模型等待时间。

提交的录像、截图和可移植验证报告位于[示例 assets 目录](../../usage/27mcp_robot/assets/)。本地重跑会在输出目录生成 `report.json`、`trace.jsonl`、模型响应、MCP 握手/工具 schema 和 stderr 日志；这些临时产物被 Git 忽略。不提交模型权重或凭据。

## LLM 如何控制机器人

1. Host 通过官方 MCP SDK 启动服务，完成 `initialize`、`tools/list` 和资源读取。
2. Host 把用户任务、真实世界摘要和工具定义交给本地 Gemma，开启 thinking。
3. Gemma 原生返回下一次 `navigate_to_waypoint` 调用，自行选择目标与顺序。
4. Host 仅补充观测对应的版本和唯一命令 ID，通过 MCP 发送模型所选参数。
5. 仿真进程运行现有 dash 控制器、完整 step、传感器更新和原生渲染，再返回结果。
6. Host 把实际结果反馈给模型，形成下一轮闭环；英文版有对应时序图。

模型推理要数十秒，因此 LLM 负责目标决策，本地控制器负责 10 Hz 运动。具名路点来自 YAML，没有在 Host 中写死运动路线。`--expect blue green` 只用于独立检查结果及结束运行，不生成模型动作。

可以改变任务：

```powershell
.\.venv\Scripts\python.exe usage/27mcp_robot/demo.py --mission "Visit orange, then green." --expect orange green --output artifacts/mcp-demo/orange-green
```

导航工具检查带机器人半径缓冲的直线路段是否与障碍相交，受阻时在设置目标之前拒绝命令。它不是全局路径规划器，也不保证动态障碍、车辆曲线轨迹或真实硬件安全。本次验证范围为平面 omni 机器人、静态障碍、dash 控制和 lidar2d。

## 接入其他 MCP Host

安装仓库后，可使用常见的本地 Host 配置格式：

```json
{
  "mcpServers": {
    "irsim": {
      "command": "C:/path/to/ir-sim/.venv/Scripts/python.exe",
      "args": [
        "-m", "irsim_mcp.server",
        "--world", "C:/path/to/ir-sim/usage/27mcp_robot/robot.yaml",
        "--allow-control", "--display"
      ]
    }
  }
}
```

请替换两处绝对路径。去掉 `--allow-control` 就是默认只读模式：工具发现中不暴露写工具，运行时也会拒绝写命令。去掉 `--display` 不创建图窗。当前传输为本地 stdio，Host 启动并管理服务生命周期；本次不暴露 HTTP 端口，也不自动附着到任意已有 Python 环境。

| MCP 接口 | 行为 |
| --- | --- |
| `irsim://world/latest` 资源 | 最近可用的完整 tick 或已完成命令的 JSON 副本 |
| `get_world_state()` | 以结构化工具结果读取相同观测 |
| `set_robot_goal(robot_id, x, y, expected_version, command_id)` | 验证有限值和边界后设置目标，不推进时间 |
| `step_simulation(steps, expected_version, command_id)` | 推进 1～200 步，碰撞或取消时停止 |
| `navigate_to_waypoint(robot_id, waypoint, max_steps, expected_version, command_id)` | 运行现有控制器，单次最多 200 步 |
| `reset_simulation(expected_version, command_id)` | 重置位置与时钟，并增加 epoch |
| `cancel_motion()` | 请求在完整 tick 边界停下，由运行中的命令报告实际停止帧 |

观测包含 schema/session/epoch/revision/version、发布序号、帧号、仿真时间、采集时间/年龄、命令执行中标志、机器人位置/速度/目标/状态、雷达距离及**逐束 valid 标志**、障碍 bounds、具名路点和完成的访问。没有有效回波不能解释为障碍。本次是精简的平面读模型，不声称完整导出了地图、所有传感器、接触报告或插件内部状态。

服务在修改前验证边界、身份和数值。版本包含随机 session ID 与重置 epoch。相同命令 ID、相同参数的重试返回回执，不重复运动；同一 ID 配不同参数会失败。回执缓存保留 256 条，淘汰后的旧版本仍会阻止过期重放。超时或异常失败会关闭会话，不能把结果未知误报成“未执行”。具体见[一致性契约](architecture.zh-CN.md)。

## 测试与证据

```powershell
# 真实仿真进程 + 官方 MCP stdio 客户端，不需要模型：
.\.venv\Scripts\python.exe -m pytest tests/test_mcp.py --no-cov -q

# 显式启用真实本地模型测试，需先启动带上述 parser 的模型服务：
$env:IRSIM_LIVE_GEMMA = '1'
.\.venv\Scripts\python.exe -m pytest tests/test_mcp.py::test_live_gemma_thinking_controls_robot --no-cov -q
Remove-Item Env:\IRSIM_LIVE_GEMMA
```

普通测试默认跳过 live model，跳过不代表模型集成成功。本次提交的 demo 证据来自真实 Gemma 调用。独立检查器验证模型原生选择的命令、thinking 输出、访问顺序、无碰撞、终点距离和到达状态；另有真实命令执行期间持续读取帧与取消动作的测试。[验证记录](validation.json)区分了本地测试、模型运行和录像检查。

## 相关性能工作与贡献流程

本集成从上游 `main` 独立构建，不依赖[调度优化 PR #386](https://github.com/hanruihua/ir-sim/pull/386)。性能基准和加速比属于那项独立改造，使用 MCP 包不要求先合入性能改动。

贡献时只需要 fork 一次，从当前上游 main 建立聚焦的功能分支，再以 `hanruihua/ir-sim:main` 为 base、`你的fork:功能分支` 为 head 提 PR。先在自己的 fork 内审阅是可选步骤。将不相关的性能改造与集成工作分开，有利于独立审阅和合并。参考 [GitHub 官方贡献流程](https://docs.github.com/en/get-started/exploring-projects-on-github/contributing-to-a-project)。
