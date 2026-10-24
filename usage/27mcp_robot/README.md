# Local Gemma + MCP robot demo

[中文](README.zh-CN.md) · [Full setup, tool reference and architecture](../../docs/mcp/README.md)

From the repository root, install the optional integration and start a local Gemma server with native tool and reasoning parsers enabled:

```powershell
uv sync --locked --extra mcp --group test --group lint
.\.venv\Scripts\python.exe usage/27mcp_robot/demo.py --display --realtime --record-window --output artifacts/mcp-demo/my-run
```

Defaults: `http://127.0.0.1:8000/v1`, `gemma-4-e2b-it`, mission `Visit blue, then green.` The validated vLLM 0.22.1 startup flags were `--enable-auto-tool-choice --tool-call-parser gemma4 --reasoning-parser gemma4`. `enable_thinking=true` is sent for every model turn. Windows recording captures the native Matplotlib window, including real thinking delays. For headless execution omit `--display --realtime --record-window`.

Gemma selects the waypoint through a native tool call; the official MCP client forwards it and the existing controller moves the robot. The grey **Obstacle / no entry** rectangle is an intentional 2 × 2 m wall. It blocks the direct initial route to green, so the robot first follows the clear route to blue. The host does not generate the route.

See [the recorded video](assets/gemma-thinking-window.mp4), [final window screenshot](assets/gemma-thinking-window.png), [portable E2E report](assets/e2e-report.json) and [compact trace](assets/trace.jsonl). The report records model-selected commands, thinking evidence, observed visit order, collision status, endpoint error and video integrity.

![Native IR-SIM window after arrival](assets/gemma-thinking-window.png)
