"""IR-SIM MCP stdio server. Run ``python -m irsim_mcp.server --help``."""

import argparse
import asyncio
import json
from typing import Annotated, Any

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import Field

from irsim_mcp.runtime import SimulationBridge

Version = Annotated[
    str,
    Field(
        min_length=1,
        max_length=128,
        description="Exact version from the latest world state",
    ),
]
CommandID = Annotated[
    str,
    Field(
        min_length=1,
        max_length=128,
        description="Unique ID; reuse with identical arguments only when retrying",
    ),
]
RobotID = Annotated[int, Field(strict=True, ge=0)]
Steps = Annotated[int, Field(strict=True, ge=1, le=200)]
Coordinate = Annotated[float, Field(strict=True, allow_inf_nan=False)]


def create_server(bridge):
    server = FastMCP(
        "ir-sim",
        instructions="Read irsim://world/latest or get_world_state. Only complete command boundaries are published. Re-read and replan after stale-version errors. Units: metres, radians, seconds. Navigation follows straight segments, not a global planner.",
    )
    read = ToolAnnotations(
        readOnlyHint=True, destructiveHint=False, openWorldHint=False
    )
    write = ToolAnnotations(
        readOnlyHint=False,
        destructiveHint=False,
        idempotentHint=True,
        openWorldHint=False,
    )

    @server.resource(
        "irsim://world/latest",
        mime_type="application/json",
        description="Copied latest completed command/tick boundary, including sensor data and freshness",
    )
    def latest() -> str:
        return json.dumps(bridge.read_world(), allow_nan=False)

    @server.tool(annotations=read)
    def get_world_state() -> dict[str, Any]:
        """Read version, frame, poses, goals, scan ranges/validity, obstacle bounds and named waypoints."""
        return bridge.read_world()

    if bridge.allow_control:

        @server.tool(annotations=write)
        async def set_robot_goal(
            robot_id: RobotID,
            x: Coordinate,
            y: Coordinate,
            expected_version: Version,
            command_id: CommandID,
        ) -> dict[str, Any]:
            """Set a bounded 2D goal. Does not advance simulation time."""
            return await asyncio.to_thread(
                bridge.command,
                "set_robot_goal",
                {"robot_id": robot_id, "x": x, "y": y},
                expected_version,
                command_id,
            )

        @server.tool(annotations=write)
        async def step_simulation(
            steps: Steps, expected_version: Version, command_id: CommandID
        ) -> dict[str, Any]:
            """Advance 1..200 complete IR-SIM ticks, stopping at collision or cancellation."""
            return await asyncio.to_thread(
                bridge.command,
                "step_simulation",
                {"steps": steps},
                expected_version,
                command_id,
            )

        @server.tool(annotations=write)
        async def navigate_to_waypoint(
            robot_id: RobotID,
            waypoint: str,
            max_steps: Steps,
            expected_version: Version,
            command_id: CommandID,
        ) -> dict[str, Any]:
            """Set a named waypoint and run the existing local controller until arrival/collision/step limit. Reject a blocked straight segment; choose intermediate waypoints to go around obstacles."""
            return await asyncio.to_thread(
                bridge.command,
                "navigate_to_waypoint",
                {"robot_id": robot_id, "waypoint": waypoint, "max_steps": max_steps},
                expected_version,
                command_id,
            )

        @server.tool(annotations=write)
        async def reset_simulation(
            expected_version: Version, command_id: CommandID
        ) -> dict[str, Any]:
            """Reset the simulation and start a new epoch, invalidating prior versions."""
            return await asyncio.to_thread(
                bridge.command, "reset_simulation", {}, expected_version, command_id
            )

        @server.tool(
            annotations=ToolAnnotations(
                readOnlyHint=False,
                destructiveHint=False,
                idempotentHint=True,
                openWorldHint=False,
            )
        )
        def cancel_motion() -> dict[str, Any]:
            """Request a running command to stop between ticks; its receipt reports the actual completed frame."""
            bridge.cancel_motion()
            return {"cancel_requested": True}

    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--world", required=True, help="Trusted local YAML configuration"
    )
    parser.add_argument(
        "--allow-control",
        action="store_true",
        help="Expose bounded mutation tools; default is read-only",
    )
    parser.add_argument(
        "--display", action="store_true", help="Open native TkAgg rendering window"
    )
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Pace commanded ticks to simulation time",
    )
    args = parser.parse_args()
    with SimulationBridge(
        args.world,
        allow_control=args.allow_control,
        display=args.display,
        realtime=args.realtime,
    ) as bridge:
        create_server(bridge).run(transport="stdio")


if __name__ == "__main__":
    main()
