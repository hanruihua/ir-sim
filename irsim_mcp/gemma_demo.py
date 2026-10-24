"""A real MCP host: local Gemma thinking + native tool calls control IR-SIM.

The model chooses the waypoint. The host supplies only request identity and
the exact observed version, then forwards the call through the MCP SDK.
"""

import argparse
import asyncio
import copy
import hashlib
import importlib.metadata
import json
import os
import sys
import time
import uuid
from datetime import timedelta
from pathlib import Path
from urllib.parse import urlsplit

import httpx
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


def local_endpoint(value):
    """Do not accidentally send local simulation observations to a remote URL."""
    parsed = urlsplit(value)
    if (
        parsed.hostname not in ("127.0.0.1", "localhost", "::1")
        or parsed.scheme != "http"
    ):
        raise ValueError("Gemma demo requires an http loopback endpoint")
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError("Use IRSIM_GEMMA_API_KEY for optional authentication")
    return value.rstrip("/")


def model_tools(tools):
    result = []
    for tool in tools:
        if tool.name not in ("get_world_state", "navigate_to_waypoint"):
            continue
        schema = copy.deepcopy(tool.inputSchema)
        for field in ("expected_version", "command_id"):
            schema.get("properties", {}).pop(field, None)
            if field in schema.get("required", []):
                schema["required"].remove(field)
        result.append(
            {
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description,
                    "parameters": schema,
                },
            }
        )
    return result


def compact_state(state):
    """Send a bounded spatial summary, retaining the full MCP state in evidence."""
    result = {
        k: state[k]
        for k in (
            "version",
            "frame",
            "sim_time",
            "world_bounds",
            "waypoints",
            "obstacles",
            "visits",
        )
    }
    result["robots"] = []
    for robot in state["robots"]:
        item = {k: robot[k] for k in ("id", "pose", "goal", "arrived", "collision")}
        item["nearest_valid_lidar_range"] = (
            robot["scan"]["nearest_valid_range"] if robot["scan"] else None
        )
        result["robots"].append(item)
    return result


def evaluate(state, expected_visits):
    actual = [visit["waypoint"] for visit in state["visits"]]
    no_collision = all(not r["collision"] for r in state["robots"])
    last = state["robots"][0]
    target = state["waypoints"][expected_visits[-1]]
    distance = sum((last["pose"][i] - target[i]) ** 2 for i in range(2)) ** 0.5
    return {
        "visited_in_order": actual == expected_visits,
        "no_collision": no_collision,
        "final_distance": distance,
        "arrived": last["arrived"],
        "physical_goal_passed": actual == expected_visits
        and no_collision
        and distance <= 0.16
        and last["arrived"],
    }


async def run_demo(
    *,
    world,
    output,
    base_url,
    model,
    mission,
    expected_visits,
    display=False,
    realtime=False,
    record_window=False,
    max_turns=8,
    max_tokens=768,
):
    base_url = local_endpoint(base_url)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not expected_visits:
        raise ValueError("expected_visits is required for independent E2E assertions")
    if record_window and not display:
        raise ValueError("--record-window requires --display")
    params = StdioServerParameters(
        command=sys.executable,
        args=[
            "-m",
            "irsim_mcp.server",
            "--world",
            str(Path(world).resolve()),
            "--allow-control",
            *(["--display"] if display else []),
            *(["--realtime"] if realtime else []),
        ],
    )
    report = {
        "passed": False,
        "model": model,
        "base_url": base_url,
        "mission": mission,
        "expected_visits": expected_visits,
        "enable_thinking": True,
        "thinking_verified": False,
        "model_selected_commands": 0,
        "turns": [],
        "started_at_unix": time.time(),
        "python": sys.version,
        "dependencies": {
            name: importlib.metadata.version(name)
            for name in ("mcp", "httpx", "numpy", "matplotlib", "shapely")
        },
        "source_sha256": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in Path(__file__).parent.glob("*.py")
        },
        "world_sha256": hashlib.sha256(Path(world).read_bytes()).hexdigest(),
    }
    recorder = None
    trace_file = output / "trace.jsonl"

    def trace(event, **data):
        with trace_file.open("a", encoding="utf-8") as stream:
            stream.write(
                json.dumps(
                    {"event": event, "at_unix": time.time(), **data},
                    ensure_ascii=False,
                    allow_nan=False,
                )
                + "\n"
            )
        print(
            json.dumps(
                {
                    "event": event,
                    **{
                        k: v
                        for k, v in data.items()
                        if k
                        in (
                            "turn",
                            "tool",
                            "status",
                            "frame",
                            "seconds",
                            "thinking_chars",
                        )
                    },
                },
                ensure_ascii=False,
            ),
            flush=True,
        )

    try:
        with (output / "server.stderr.log").open("w", encoding="utf-8") as server_log:
            async with (
                stdio_client(params, errlog=server_log) as (reader, writer),
                ClientSession(
                    reader, writer, read_timeout_seconds=timedelta(seconds=90)
                ) as client,
                httpx.AsyncClient(timeout=httpx.Timeout(600, connect=10)) as http,
            ):
                initialized = await client.initialize()
                tools = (await client.list_tools()).tools
                (output / "mcp-handshake.json").write_text(
                    json.dumps(
                        {
                            "initialize": initialized.model_dump(mode="json"),
                            "tools": [t.model_dump(mode="json") for t in tools],
                        },
                        indent=2,
                    ),
                    encoding="utf-8",
                )
                resource = await client.read_resource("irsim://world/latest")
                state = json.loads(resource.contents[0].text)
                trace("initial_observation", state=state, frame=state["frame"])
                if record_window:
                    from irsim_mcp.recording import WindowRecording

                    recorder = await asyncio.to_thread(
                        WindowRecording, output / "window.mp4", state["window_title"]
                    )
                    trace(
                        "window_recording_started",
                        capture="gdigrab",
                        window_title=state["window_title"],
                    )
                messages = [
                    {
                        "role": "system",
                        "content": "You control one simulated robot with real MCP tools. Follow the user's waypoint order. Use navigate_to_waypoint with robot_id=0, the next waypoint name, and max_steps=150. It follows a straight line; choose intermediate waypoints if a wall blocks it. Use exactly one tool per turn, after thinking briefly. Tool feedback contains the actual observed world; never claim movement before a successful tool result.",
                    },
                    {
                        "role": "user",
                        "content": mission
                        + "\nActual MCP observation:\n"
                        + json.dumps(compact_state(state)),
                    },
                ]
                headers = {}
                if os.getenv("IRSIM_GEMMA_API_KEY"):
                    headers["Authorization"] = (
                        f"Bearer {os.environ['IRSIM_GEMMA_API_KEY']}"
                    )
                for turn in range(1, max_turns + 1):
                    request = {
                        "model": model,
                        "messages": messages,
                        "tools": model_tools(tools),
                        "tool_choice": "auto",
                        "temperature": 0,
                        "max_tokens": max_tokens,
                        "chat_template_kwargs": {"enable_thinking": True},
                    }
                    trace(
                        "model_request",
                        turn=turn,
                        enable_thinking=True,
                        observed_version=state["version"],
                    )
                    started = time.monotonic()
                    response = await http.post(
                        f"{base_url}/chat/completions", json=request, headers=headers
                    )
                    response.raise_for_status()
                    payload = response.json()
                    (output / f"model-response-{turn}.json").write_text(
                        json.dumps(payload, indent=2, ensure_ascii=False),
                        encoding="utf-8",
                    )
                    choice = payload["choices"][0]
                    message = choice["message"]
                    reasoning = (
                        message.get("reasoning")
                        or message.get("reasoning_content")
                        or ""
                    )
                    thought = bool(reasoning.strip())
                    report["thinking_verified"] |= thought
                    turn_evidence = {
                        "turn": turn,
                        "seconds": round(time.monotonic() - started, 3),
                        "finish_reason": choice["finish_reason"],
                        "usage": payload.get("usage"),
                        "thinking_chars": len(reasoning),
                        "reasoning_sha256": hashlib.sha256(
                            reasoning.encode()
                        ).hexdigest()
                        if thought
                        else None,
                    }
                    report["turns"].append(turn_evidence)
                    trace("model_response", **turn_evidence)
                    if choice["finish_reason"] == "length":
                        raise RuntimeError(
                            "Gemma exhausted its token budget; no truncated tool call is executed"
                        )
                    calls = message.get("tool_calls") or []
                    if len(calls) != 1:
                        raise RuntimeError(
                            f"Expected one native model tool call; received {len(calls)}"
                        )
                    call = calls[0]
                    name = call["function"]["name"]
                    if name not in {t["function"]["name"] for t in request["tools"]}:
                        raise RuntimeError(f"Model selected an unexposed tool: {name}")
                    arguments = json.loads(call["function"]["arguments"])
                    if (
                        not isinstance(arguments, dict)
                        or {"expected_version", "command_id"} & arguments.keys()
                    ):
                        raise RuntimeError(
                            "Model cannot supply host-owned request identity/version"
                        )
                    # Native model-selected arguments are forwarded unchanged.
                    if name != "get_world_state":
                        arguments |= {
                            "expected_version": state["version"],
                            "command_id": uuid.uuid4().hex,
                        }
                    trace("mcp_call", turn=turn, tool=name, arguments=arguments)
                    result = await client.call_tool(name, arguments)
                    if result.isError:
                        feedback = {
                            "error": [
                                c.text for c in result.content if c.type == "text"
                            ]
                        }
                    else:
                        data = result.structuredContent
                        if not isinstance(data, dict):
                            raise RuntimeError(
                                "MCP tool returned no structured observation"
                            )
                        state = data.get("state", data)
                        if name == "navigate_to_waypoint":
                            report["model_selected_commands"] += 1
                        feedback = {
                            "outcome": data.get("outcome"),
                            "observation": compact_state(state),
                        }
                    trace(
                        "mcp_result",
                        turn=turn,
                        tool=name,
                        result=result.model_dump(mode="json"),
                        frame=state["frame"],
                    )
                    # Keep native tool-call history; do not replay provider-only fields.
                    messages.append(
                        {
                            "role": "assistant",
                            "content": message.get("content"),
                            "tool_calls": calls,
                        }
                    )
                    messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": call["id"],
                            "content": json.dumps(feedback),
                        }
                    )
                    checks = evaluate(state, expected_visits)
                    if checks["physical_goal_passed"]:
                        break
                report["checks"] = evaluate(state, expected_visits)
                report["final_state"] = state
                # A successful motion trace alone cannot satisfy the model E2E.
                report["passed"] = bool(
                    report["checks"]["physical_goal_passed"]
                    and report["thinking_verified"]
                    and report["model_selected_commands"] >= len(expected_visits)
                )
                trace(
                    "e2e_assertions",
                    status="passed" if report["passed"] else "failed",
                    checks=report["checks"],
                )
                if recorder:
                    await asyncio.sleep(2)
                    report["recording"] = await asyncio.to_thread(recorder.close)
                    recorder = None
    except BaseException as exc:
        report["passed"] = False
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            if recorder:
                report["recording"] = await asyncio.to_thread(recorder.close)
        except Exception as exc:
            report["passed"] = False
            report["recording_error"] = f"{type(exc).__name__}: {exc}"
        report["elapsed_seconds"] = round(time.time() - report["started_at_unix"], 3)
        (output / "report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--output", default="artifacts/mcp-demo")
    parser.add_argument(
        "--base-url", default=os.getenv("IRSIM_GEMMA_URL", "http://127.0.0.1:8000/v1")
    )
    parser.add_argument(
        "--model", default=os.getenv("IRSIM_GEMMA_MODEL", "gemma-4-e2b-it")
    )
    parser.add_argument(
        "--mission", default="Visit blue, then green. Stop after reaching green."
    )
    parser.add_argument("--expect", nargs="+", default=["blue", "green"])
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--record-window", action="store_true")
    parser.add_argument("--max-tokens", type=int, default=768)
    args = parser.parse_args()
    report = asyncio.run(
        run_demo(
            world=args.world,
            output=args.output,
            base_url=args.base_url,
            model=args.model,
            mission=args.mission,
            expected_visits=args.expect,
            display=args.display,
            realtime=args.realtime,
            record_window=args.record_window,
            max_tokens=args.max_tokens,
        )
    )
    print(
        json.dumps(
            {"passed": report["passed"], "output": str(Path(args.output).resolve())}
        )
    )
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
