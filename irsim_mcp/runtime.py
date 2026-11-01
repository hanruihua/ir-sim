"""One simulation writer process and a copied, versioned JSON read model.

No IR-SIM/Matplotlib imports occur in the MCP process. This both isolates global
simulation parameters and keeps library diagnostics away from JSON-RPC stdout.
"""

import contextlib
import copy
import hashlib
import json
import math
import multiprocessing as mp
import queue
import sys
import threading
import time
import uuid
from collections import OrderedDict
from pathlib import Path

WINDOW_TITLE = "IR-SIM MCP Robot Demo"
MAX_STEPS = 200


class BridgeError(ValueError):
    """A rejected command, or an unavailable simulation worker."""


def _integer(value, label, minimum=0, maximum=MAX_STEPS):
    if type(value) is not int or not minimum <= value <= maximum:
        raise BridgeError(f"{label} must be an integer in [{minimum}, {maximum}]")
    return value


class _Simulation:
    """Private worker state. Only the worker's main thread may access this."""

    def __init__(self, config, display, realtime, cancel, snapshots):
        import matplotlib

        matplotlib.use("TkAgg" if display else "Agg")
        import irsim

        self.env = irsim.make(
            str(config),
            display=display,
            headless=not display,
            seed=7,
            log_level="ERROR",
        )
        if self.env.world_param.step_mode != "internal":
            raise BridgeError("MCP control requires internal stepping")
        self.session = uuid.uuid4().hex
        self.window_title = f"{WINDOW_TITLE} [{self.session[:8]}]" if display else None
        self.epoch = 0
        self.revision = 0
        self.publication = 0
        self.busy = False
        self.visits = []
        self.receipts = OrderedDict()
        self.display = display
        self.realtime = realtime
        self.cancel = cancel
        self.snapshots = snapshots
        self.waypoints = self.env.config.get("custom", {}).get("waypoints", {})
        for name, point in self.waypoints.items():
            if not isinstance(name, str) or len(point) != 2:
                raise BridgeError("waypoints must map names to [x, y]")
            self._goal(*point)
        if display:
            plot = self.env._env_plot
            plot.fig.canvas.manager.set_window_title(self.window_title)
            labels = self.env.config.get("custom", {}).get("labels", {})
            for obj in self.env.objects:
                if obj.name in labels:
                    obj.set_text(str(labels[obj.name]))
            for name, (x, y) in self.waypoints.items():
                color = name if name in ("blue", "green", "orange") else "purple"
                plot.ax.scatter([x], [y], c=color, s=100, marker="s", zorder=5)
                plot.ax.text(x + 0.12, y + 0.12, name, color=color, fontsize=12)
            self.title("Waiting for Gemma thinking / MCP commands")
            self.render()

    def title(self, text):
        if self.display:
            self.env.set_title(text)

    def render(self):
        if self.display:
            self.env.render(interval=0.001)
            self.env._env_plot.fig.canvas.draw_idle()
            self.env._env_plot.fig.canvas.flush_events()

    def pump(self):
        if self.display:
            import matplotlib.pyplot as plt

            if not plt.fignum_exists(self.env._env_plot.fig.number):
                raise EOFError("simulation window was closed")
            plt.pause(0.01)

    def _goal(self, x, y):
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
            raise BridgeError("goal coordinates must be finite numbers")
        world = self.env._world
        if not (
            world.x_range[0] <= x <= world.x_range[1]
            and world.y_range[0] <= y <= world.y_range[1]
        ):
            raise BridgeError("goal is outside world bounds")
        return [float(x), float(y)]

    def _robot(self, identity):
        if type(identity) is not int:
            raise BridgeError("robot_id must be an integer")
        for obj in self.env.robot_list:
            if obj.id == identity:
                return obj
        raise BridgeError(f"unknown robot_id: {identity}")

    def snapshot(self):
        import numpy as np

        def vector(value):
            return None if value is None else np.asarray(value).ravel().tolist()

        robots = []
        for robot in self.env.robot_list:
            scan = next((s for s in robot.sensors if s.sensor_type == "lidar2d"), None)
            summary = None
            if scan is not None:
                valid = np.asarray(scan.valid, dtype=bool)
                ranges = np.asarray(scan.range_data)
                summary = {
                    "beam_count": int(ranges.size),
                    "valid_count": int(valid.sum()),
                    "nearest_valid_range": float(ranges[valid].min())
                    if valid.any()
                    else None,
                    "ranges": vector(ranges),
                    "valid": vector(valid),
                    "range_max": float(scan.range_max),
                }
            robots.append(
                {
                    "id": robot.id,
                    "name": robot.name,
                    "pose": vector(robot.state),
                    "velocity": vector(robot.velocity),
                    "goal": vector(robot.goal),
                    "arrived": bool(robot.arrive),
                    "collision": bool(robot.collision),
                    "scan": summary,
                }
            )
        world = self.env._world
        self.publication += 1
        result = {
            "schema_version": 1,
            "publication_sequence": self.publication,
            "command_in_progress": self.busy,
            "session": self.session,
            "epoch": self.epoch,
            "revision": self.revision,
            "version": f"{self.session}:{self.epoch}:{self.revision}",
            "frame": world.count,
            "sim_time": world.time,
            "step_time": world.step_time,
            "captured_at_unix": time.time(),
            "worker_pid": mp.current_process().pid,
            "window_title": self.window_title,
            "world_bounds": [*world.x_range, *world.y_range],
            "robots": robots,
            "obstacles": [
                {"id": o.id, "bounds": list(o.geometry.bounds)}
                for o in self.env.obstacle_list
            ],
            "waypoints": copy.deepcopy(self.waypoints),
            "visits": copy.deepcopy(self.visits),
        }
        # Fail closed if an unsupported world produces NaN/Infinity in its read model.
        return json.loads(json.dumps(result, allow_nan=False))

    def publish(self):
        """Latest-only telemetry: never block physics on a slow reader."""
        state = self.snapshot()
        try:
            self.snapshots.put_nowait(state)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                self.snapshots.get_nowait()
            with contextlib.suppress(queue.Full):
                self.snapshots.put_nowait(state)

    def execute(self, name, arguments, expected_version, command_id):
        if not isinstance(command_id, str) or not 1 <= len(command_id) <= 128:
            raise BridgeError("command_id must contain 1..128 characters")
        try:
            fingerprint = hashlib.sha256(
                json.dumps(
                    [name, arguments, expected_version], sort_keys=True, allow_nan=False
                ).encode()
            ).hexdigest()
        except (ValueError, TypeError) as exc:
            raise BridgeError(
                "goal/command arguments must be finite JSON values"
            ) from exc
        if command_id in self.receipts:
            previous, outcome = self.receipts[command_id]
            if previous != fingerprint:
                raise BridgeError(
                    "command_id was already used with different arguments"
                )
            return {
                "command_id": command_id,
                "duplicate": True,
                "outcome": outcome,
                "state": self.snapshot(),
            }
        if expected_version != f"{self.session}:{self.epoch}:{self.revision}":
            raise BridgeError(
                "stale expected_version; read the latest world and replan"
            )

        # Validate every argument before the first state change.
        if name == "set_robot_goal":
            if set(arguments) != {"robot_id", "x", "y"}:
                raise BridgeError("set_robot_goal requires robot_id, x, y")
            robot = self._robot(arguments["robot_id"])
            goal = self._goal(arguments["x"], arguments["y"])
        elif name == "navigate_to_waypoint":
            if set(arguments) != {"robot_id", "waypoint", "max_steps"}:
                raise BridgeError(
                    "navigate_to_waypoint requires robot_id, waypoint, max_steps"
                )
            robot = self._robot(arguments["robot_id"])
            waypoint = arguments["waypoint"]
            if not isinstance(waypoint, str) or waypoint not in self.waypoints:
                raise BridgeError("unknown waypoint")
            goal = self._goal(*self.waypoints[waypoint])
            steps = _integer(arguments["max_steps"], "max_steps", 1)
            # The demo primitive is a straight segment, not a global path planner.
            # Reject blocked static geometry before changing the robot's goal.
            from shapely.geometry import LineString

            corridor = LineString([robot.state[:2, 0], goal]).buffer(
                float(robot.radius) + 0.02
            )
            if any(o.geometry.intersects(corridor) for o in self.env.obstacle_list):
                raise BridgeError("segment is blocked; choose an intermediate waypoint")
        elif name == "step_simulation":
            if set(arguments) != {"steps"}:
                raise BridgeError("step_simulation requires steps")
            steps = _integer(arguments["steps"], "steps", 1)
        elif name == "reset_simulation":
            if arguments:
                raise BridgeError("reset_simulation takes no action arguments")
        else:
            raise BridgeError(f"unknown command: {name}")

        self.cancel.clear()
        self.busy = True
        if name in ("set_robot_goal", "navigate_to_waypoint"):
            robot.set_goal(goal)
            self.env.refresh()
            self.revision += 1
            self.publish()
        outcome = {"status": "accepted", "advanced_steps": 0}
        if name == "reset_simulation":
            self.env.reset()
            self.epoch += 1
            self.visits.clear()
            self.title("Reset / waiting for MCP commands")
        elif name in ("step_simulation", "navigate_to_waypoint"):
            if name == "navigate_to_waypoint":
                self.title(f"Gemma chose {waypoint} / MCP navigation")
            for _ in range(steps):
                if self.cancel.is_set():
                    outcome["status"] = "cancelled"
                    break
                started = time.monotonic()
                # EnvBase performs motion, geometry, sensors and status in its
                # established order. No simulator fields are written in parallel.
                self.env.step()
                self.revision += 1
                outcome["advanced_steps"] += 1
                self.publish()
                self.render()
                if any(obj.collision for obj in self.env.robot_list):
                    outcome["status"] = "collision"
                    break
                if name == "navigate_to_waypoint" and robot.arrive:
                    outcome["status"] = "arrived"
                    self.visits.append(
                        {
                            "waypoint": waypoint,
                            "robot_id": robot.id,
                            "frame": self.env._world.count,
                        }
                    )
                    break
                if self.realtime:
                    # Keep the GUI responsive while matching the simulation clock.
                    deadline = started + self.env.step_time
                    while time.monotonic() < deadline:
                        self.pump()
                        time.sleep(min(0.005, max(0, deadline - time.monotonic())))
            else:
                outcome["status"] = (
                    "step_limit" if name == "navigate_to_waypoint" else "completed"
                )
        self.revision += 1
        self.busy = False
        self.title(
            f"{outcome['status']} / frame {self.env._world.count} / waiting for Gemma"
        )
        self.render()
        self.receipts[command_id] = (fingerprint, copy.deepcopy(outcome))
        if len(self.receipts) > 256:
            self.receipts.popitem(last=False)
        return {
            "command_id": command_id,
            "duplicate": False,
            "outcome": outcome,
            "state": self.snapshot(),
        }


def _worker(connection, config, display, realtime, cancel, snapshots):
    # Must precede importing the simulator: it may print library diagnostics.
    sys.stdout = sys.stderr
    simulation = None
    try:
        simulation = _Simulation(config, display, realtime, cancel, snapshots)
        connection.send({"state": simulation.snapshot()})
        while True:
            simulation.pump()
            if not connection.poll(0.02):
                continue
            request = connection.recv()
            if request is None:
                break
            try:
                result = simulation.execute(**request)
                connection.send({"result": result})
            except BridgeError as exc:
                connection.send({"error": str(exc)})
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        # A partially applied unexpected simulator failure is never advertised
        # as a successful command, and cannot be retried on this worker.
        connection.send({"fatal": f"{type(exc).__name__}: {exc}"})
    finally:
        if simulation is not None:
            simulation.env.end(0)
        connection.close()
        snapshots.cancel_join_thread()
        snapshots.close()


class SimulationBridge:
    """Process-isolated command gateway. Reads return owned copies immediately.

    ``timeout`` covers one IPC exchange, including bounded real-time motion.
    Unexpected timeout/crash invalidates the worker; callers must not blindly
    retry a mutation with an unknown result. Restart creates a new session ID.
    """

    def __init__(
        self, config, *, allow_control=False, display=False, realtime=False, timeout=60
    ):
        self.config = str(Path(config).resolve(strict=True))
        self.allow_control = allow_control
        self.timeout = timeout
        self._lock = threading.Lock()
        self._state_lock = threading.Lock()
        context = mp.get_context("spawn")
        self._connection, child = context.Pipe()
        self._cancel = context.Event()
        self._snapshots = context.Queue(maxsize=2)
        self._process = context.Process(
            target=_worker,
            args=(child, self.config, display, realtime, self._cancel, self._snapshots),
            daemon=True,
        )
        self._state = None
        self._closed = False
        self._process.start()
        child.close()
        try:
            self._state = self._receive()["state"]
        except BaseException:
            self.close()
            raise

    def _receive(self):
        if not self._connection.poll(self.timeout):
            self.close()
            raise BridgeError("worker timeout; command outcome unknown, session closed")
        try:
            message = self._connection.recv()
        except (EOFError, OSError) as exc:
            self.close()
            raise BridgeError("worker unavailable; command outcome unknown") from exc
        if "fatal" in message:
            self.close()
            raise BridgeError(f"worker failed: {message['fatal']}")
        if "error" in message:
            raise BridgeError(message["error"])
        return message

    def read_world(self):
        if self._closed or not self._process.is_alive():
            raise BridgeError("worker is closed or unavailable")
        with self._state_lock:
            while True:
                try:
                    incoming = self._snapshots.get_nowait()
                except queue.Empty:
                    break
                self._accept_state(incoming)
            state = copy.deepcopy(self._state)
        state["age_seconds"] = max(0, time.time() - state["captured_at_unix"])
        return state

    def _accept_state(self, state):
        if state["publication_sequence"] > self._state["publication_sequence"]:
            self._state = state

    def command(self, name, arguments, expected_version, command_id):
        if not self.allow_control:
            raise BridgeError("server is read-only; restart with --allow-control")
        # The lock belongs to the blocking operation, so cancelling an async
        # tool handler cannot release it while IPC is still in progress.
        with self._lock:
            self.read_world()
            try:
                self._connection.send(
                    {
                        "name": name,
                        "arguments": arguments,
                        "expected_version": expected_version,
                        "command_id": command_id,
                    }
                )
            except (OSError, BrokenPipeError) as exc:
                self.close()
                raise BridgeError("worker unavailable") from exc
            result = self._receive()["result"]
            with self._state_lock:
                self._accept_state(result["state"])
            return copy.deepcopy(result)

    def cancel_motion(self):
        """Request cancellation between complete ticks, without waiting on IPC."""
        if not self.allow_control:
            raise BridgeError("server is read-only")
        self._cancel.set()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._cancel.set()
        with contextlib.suppress(OSError, BrokenPipeError):
            self._connection.send(None)
        self._process.join(timeout=3)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=3)
        self._connection.close()
        self._snapshots.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
