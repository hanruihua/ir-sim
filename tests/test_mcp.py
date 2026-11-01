"""Contracts for the optional MCP bridge, using real simulation and stdio IPC."""

import asyncio
import json
import os
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip("mcp")
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from irsim_mcp.runtime import BridgeError, SimulationBridge

WORLD = Path(__file__).resolve().parents[1] / "usage/27mcp_robot/robot.yaml"


@pytest.fixture
def bridge():
    with SimulationBridge(WORLD, allow_control=True) as instance:
        yield instance


def command(bridge, name, **arguments):
    return bridge.command(
        name,
        arguments,
        expected_version=bridge.read_world()["version"],
        command_id=f"test-{bridge.read_world()['revision']}-{name}",
    )


def test_copied_read_model_has_finite_sensor_data_and_complete_ticks(bridge):
    before = bridge.read_world()
    assert before["frame"] == 0
    assert before["robots"][0]["scan"]["beam_count"] == 60
    json.dumps(before, allow_nan=False)
    before["robots"][0]["pose"][0] = -100
    assert bridge.read_world()["robots"][0]["pose"][0] == 1
    command(bridge, "set_robot_goal", robot_id=0, x=2, y=7)
    result = command(bridge, "step_simulation", steps=5)
    assert result["state"]["frame"] == 5
    assert result["state"]["sim_time"] == 0.5
    assert result["state"]["robots"][0]["pose"][1] > 1
    assert before["frame"] == 0
    assert bridge.read_world()["frame"] == 5


def test_read_only_default_rejects_control():
    with SimulationBridge(WORLD) as instance:
        with pytest.raises(BridgeError, match="read-only"):
            command(instance, "step_simulation", steps=1)
        assert instance.read_world()["frame"] == 0


def test_stale_version_rejected_without_mutation(bridge):
    old = bridge.read_world()["version"]
    command(bridge, "step_simulation", steps=2)
    with pytest.raises(BridgeError, match="stale"):
        bridge.command("step_simulation", {"steps": 3}, old, "stale")
    assert bridge.read_world()["frame"] == 2


def test_duplicate_command_is_not_reapplied_or_published_backwards(bridge):
    version = bridge.read_world()["version"]
    first = bridge.command("step_simulation", {"steps": 2}, version, "same")
    command(bridge, "step_simulation", steps=3)
    duplicate = bridge.command("step_simulation", {"steps": 2}, version, "same")
    assert first["duplicate"] is False
    assert duplicate["duplicate"] is True
    assert duplicate["state"]["frame"] == bridge.read_world()["frame"] == 5
    with pytest.raises(BridgeError, match="different"):
        bridge.command("step_simulation", {"steps": 3}, version, "same")


@pytest.mark.parametrize("steps", [0, -1, 201, 1.5, True])
def test_step_bounds_are_validated_before_mutation(bridge, steps):
    version = bridge.read_world()["version"]
    with pytest.raises(BridgeError, match="steps"):
        command(bridge, "step_simulation", steps=steps)
    assert bridge.read_world()["version"] == version


@pytest.mark.parametrize(
    ("x", "y"), [(float("nan"), 2), (2, float("inf")), (-1, 2), (11, 2)]
)
def test_invalid_goals_do_not_partially_mutate(bridge, x, y):
    before = bridge.read_world()
    with pytest.raises(BridgeError, match="goal"):
        command(bridge, "set_robot_goal", robot_id=0, x=x, y=y)
    assert bridge.read_world()["version"] == before["version"]
    assert bridge.read_world()["robots"] == before["robots"]


def test_invalid_identity_and_unknown_tools_are_rejected(bridge):
    with pytest.raises(BridgeError, match="robot"):
        command(bridge, "set_robot_goal", robot_id=99, x=2, y=2)
    with pytest.raises(BridgeError, match="unknown"):
        command(bridge, "execute_python", code="print(1)")
    assert bridge.read_world()["frame"] == 0


def test_reset_changes_epoch_and_invalidates_old_version(bridge):
    initial = bridge.read_world()
    command(bridge, "step_simulation", steps=3)
    reset = command(bridge, "reset_simulation")["state"]
    assert reset["frame"] == 0
    assert reset["epoch"] == initial["epoch"] + 1
    assert reset["version"] != initial["version"]
    with pytest.raises(BridgeError, match="stale"):
        bridge.command("step_simulation", {"steps": 1}, initial["version"], "old")


def test_waypoint_navigation_uses_real_kinematics_and_sensor_updates(bridge):
    for waypoint in ("blue", "green"):
        result = command(
            bridge, "navigate_to_waypoint", robot_id=0, waypoint=waypoint, max_steps=150
        )
        assert result["outcome"]["status"] == "arrived"
        assert result["outcome"]["advanced_steps"] > 20
        assert result["state"]["robots"][0]["collision"] is False
        assert result["state"]["robots"][0]["scan"]["beam_count"] == 60
    state = bridge.read_world()
    assert [v["waypoint"] for v in state["visits"]] == ["blue", "green"]
    assert state["robots"][0]["pose"][:2] == pytest.approx([8, 7], abs=0.16)


def test_blocked_segment_is_rejected_before_setting_goal(bridge):
    before = bridge.read_world()
    # Orange is initially clear, but the diagonal to green crosses the wall.
    with pytest.raises(BridgeError, match="blocked"):
        command(
            bridge, "navigate_to_waypoint", robot_id=0, waypoint="green", max_steps=150
        )
    assert bridge.read_world()["version"] == before["version"]


def test_collision_stops_bounded_motion_and_is_reported(bridge):
    command(bridge, "set_robot_goal", robot_id=0, x=5, y=4.5)
    result = command(bridge, "step_simulation", steps=200)
    assert result["outcome"]["status"] == "collision"
    assert 0 < result["outcome"]["advanced_steps"] < 200
    assert result["state"]["robots"][0]["collision"] is True


def test_dead_worker_is_not_advertised_as_a_fresh_observation(bridge):
    bridge._process.terminate()
    bridge._process.join(timeout=3)
    with pytest.raises(BridgeError, match="unavailable"):
        bridge.read_world()


def test_reads_during_mutation_never_expose_half_a_tick(bridge):
    async def exercise():
        write = asyncio.create_task(
            asyncio.to_thread(command, bridge, "step_simulation", steps=100)
        )
        for _ in range(30):
            state = bridge.read_world()
            assert state["sim_time"] == pytest.approx(state["frame"] * 0.1)
            assert 0 <= state["frame"] <= 100
            await asyncio.sleep(0)
        await write

    asyncio.run(exercise())


def test_live_reads_and_cancellation_during_a_real_command():
    async def exercise(instance):
        task = asyncio.create_task(
            asyncio.to_thread(command, instance, "step_simulation", steps=200)
        )
        observed = []
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            state = instance.read_world()
            observed.append(state["frame"])
            assert state["sim_time"] == pytest.approx(state["frame"] * 0.1)
            if state["frame"] >= 3:
                instance.cancel_motion()
                break
            await asyncio.sleep(0.02)
        result = await task
        assert any(0 < frame < 200 for frame in observed)
        assert result["outcome"]["status"] == "cancelled"
        assert 3 <= result["outcome"]["advanced_steps"] < 200
        assert instance.read_world()["frame"] == result["state"]["frame"]

    with SimulationBridge(WORLD, allow_control=True, realtime=True) as instance:
        asyncio.run(exercise(instance))


@pytest.mark.parametrize("allow_control", [False, True])
def test_real_mcp_stdio_handshake_resources_and_control(allow_control):
    async def exercise():
        arguments = ["-m", "irsim_mcp.server", "--world", str(WORLD)]
        if allow_control:
            arguments.append("--allow-control")
        params = StdioServerParameters(command=sys.executable, args=arguments)
        async with (
            stdio_client(params) as (reader, writer),
            ClientSession(reader, writer) as client,
        ):
            initialized = await client.initialize()
            assert initialized.serverInfo.name == "ir-sim"
            tools = {tool.name: tool for tool in (await client.list_tools()).tools}
            assert "get_world_state" in tools
            assert ("step_simulation" in tools) is allow_control
            resources = await client.list_resources()
            assert str(resources.resources[0].uri) == "irsim://world/latest"
            state = (await client.call_tool("get_world_state", {})).structuredContent
            assert state["frame"] == 0
            if allow_control:
                invalid = await client.call_tool(
                    "step_simulation",
                    {
                        "steps": True,
                        "expected_version": state["version"],
                        "command_id": "bad-steps",
                    },
                )
                assert invalid.isError is True
                result = await client.call_tool(
                    "step_simulation",
                    {
                        "steps": 3,
                        "expected_version": state["version"],
                        "command_id": "e2e",
                    },
                )
                assert result.isError is False
                assert result.structuredContent["state"]["frame"] == 3
                stale = await client.call_tool(
                    "step_simulation",
                    {
                        "steps": 3,
                        "expected_version": state["version"],
                        "command_id": "stale",
                    },
                )
                assert stale.isError is True
            resource = await client.read_resource("irsim://world/latest")
            assert json.loads(resource.contents[0].text)["frame"] == (
                3 if allow_control else 0
            )

    asyncio.run(exercise())


@pytest.mark.parametrize(
    "url",
    [
        "https://example.com/v1",
        "http://192.168.1.2/v1",
        "http://user:password@localhost/v1",
    ],
)
def test_model_host_rejects_nonlocal_or_embedded_credentials(url):
    from irsim_mcp.gemma_demo import local_endpoint

    with pytest.raises(ValueError, match=r"loopback|authentication"):
        local_endpoint(url)


def test_e2e_oracle_rejects_wrong_visit_order_and_collision(bridge):
    from irsim_mcp.gemma_demo import evaluate

    for waypoint in ("blue", "green"):
        command(
            bridge, "navigate_to_waypoint", robot_id=0, waypoint=waypoint, max_steps=150
        )
    state = bridge.read_world()
    assert evaluate(state, ["blue", "green"])["physical_goal_passed"] is True
    assert evaluate(state, ["green", "blue"])["physical_goal_passed"] is False
    state["robots"][0]["collision"] = True
    assert evaluate(state, ["blue", "green"])["physical_goal_passed"] is False


@pytest.mark.skipif(
    os.getenv("IRSIM_LIVE_GEMMA") != "1", reason="explicit live local model opt-in"
)
def test_live_gemma_thinking_controls_robot(tmp_path):
    from irsim_mcp.gemma_demo import run_demo

    report = asyncio.run(
        run_demo(
            world=WORLD,
            output=tmp_path,
            base_url=os.getenv("IRSIM_GEMMA_URL", "http://127.0.0.1:8000/v1"),
            model=os.getenv("IRSIM_GEMMA_MODEL", "gemma-4-e2b-it"),
            mission="Visit blue, then green. Stop after reaching green.",
            expected_visits=["blue", "green"],
            display=False,
        )
    )
    assert report["passed"] is True
    assert report["thinking_verified"] is True
    assert report["model_selected_commands"] >= 2
