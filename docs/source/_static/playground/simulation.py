"""Small browser adapter. All simulation updates are delegated to IR-SIM."""

import json
import math
from io import BytesIO
from pathlib import Path

import yaml
from matplotlib import pyplot as plt

import irsim


class Playground:
    def __init__(self):
        self.env = None
        self.steps = 0
        self.frame = b""
        self.frame_time = 0

    def create(self, source, seed=0):
        if self.env is not None:
            self.env.end(0)
            self.env = None
        if len(source) > 64000:
            raise ValueError("Playground YAML is limited to 64,000 characters.")
        config = yaml.safe_load(source)
        if not isinstance(config, dict):
            raise ValueError("The scene must be a YAML mapping.")
        world = config.setdefault("world", {})
        if not isinstance(world, dict):
            raise ValueError("world must be a mapping.")
        dt = world.get("step_time", 0.1)
        if (
            not isinstance(dt, (int, float))
            or not math.isfinite(dt)
            or not 0.01 <= dt <= 0.5
        ):
            raise ValueError("Playground step_time must be between 0.01 and 0.5 s.")
        if world.get("step_mode", "internal") != "internal":
            raise ValueError("This playground supports internal step mode only.")
        if world.get("obstacle_map"):
            raise ValueError(
                "File and grid maps are not supported in this first playground."
            )
        if world.get("control_mode", "auto") != "auto":
            raise ValueError("Use control_mode: auto; desktop input is unavailable.")
        for name, limit in [("robot", 10), ("obstacle", 50)]:
            entries = config.get(name) or []
            if isinstance(entries, dict):
                entries = [entries]
            if not isinstance(entries, list) or any(
                not isinstance(o, dict) for o in entries
            ):
                raise ValueError(f"{name} must be a mapping or list of mappings.")
            counts = [o.get("number", 1) for o in entries]
            if (
                any(type(n) is not int or not 1 <= n <= limit for n in counts)
                or sum(counts) > limit
            ):
                raise ValueError(f"Playground is limited to {limit} {name} objects.")
            if name == "robot" and not entries:
                raise ValueError("Provide at least one robot mapping.")
        plot = world.setdefault("plot", {})
        pixels = plot.setdefault("figure_pixels", [720, 576])
        if (
            not isinstance(pixels, list)
            or len(pixels) != 2
            or any(type(n) is not int or not 240 <= n <= 1000 for n in pixels)
        ):
            raise ValueError(
                "figure_pixels must contain two integers from 240 to 1000."
            )
        # Use normal figure callbacks, never a desktop/global keyboard hook.
        config.setdefault("gui", {}).setdefault("keyboard", {})["backend"] = "mpl"
        Path("/tmp/playground.yaml").write_text(yaml.safe_dump(config))
        figures = set(plt.get_fignums())
        try:
            self.env = irsim.make(
                "/tmp/playground.yaml", display=False, seed=int(seed), log_level="ERROR"
            )
        except Exception:
            # Invalid plot options can fail after a figure has been allocated.
            for figure in set(plt.get_fignums()) - figures:
                plt.close(figure)
            raise
        self.steps = 0
        self.frame_time = 0
        return self.snapshot()

    def step(self, action=None, count=1):
        if self.env is None:
            raise ValueError("Create a scene first.")
        if self.steps >= 2000:
            raise ValueError(
                "Playground step budget reached (2000). Reset to continue."
            )
        if type(count) is not int or not 1 <= count <= 20:
            raise ValueError("Request between 1 and 20 simulation steps per frame.")
        if action is not None and not self.manual_control:
            raise ValueError(
                "Slider commands require one diff robot. Use YAML behavior."
            )
        for _ in range(min(count, 2000 - self.steps)):
            self.env.step(action=action)
            self.env.render()
            if self.env._world.sampling:
                self.frame_time = self.env.time
            self.steps += 1
            if self.env.done():
                break
        return self.snapshot()

    @property
    def manual_control(self):
        robots = self.env.robot_list
        return len(robots) == 1 and robots[0].kinematics == "diff"

    def snapshot(self):
        # The native IR-SIM plot owns every artist, including sensors and trails.
        # Encode only once per displayed frame, not once per integration step.
        buffer = BytesIO()
        self.env._env_plot.fig.canvas.print_png(buffer)
        self.frame = buffer.getvalue()
        objects = []
        for obj in self.env.objects:
            goal = obj.goal
            objects.append(
                {
                    "id": obj.id,
                    "name": obj.name,
                    "role": obj.role,
                    "kinematics": obj.kinematics,
                    "shape": obj.shape,
                    "state": obj.state.ravel().tolist(),
                    "velocity": obj.velocity.ravel().tolist(),
                    "goal": None if goal is None else goal.ravel().tolist(),
                    "collision": bool(obj.collision_flag),
                    "arrived": bool(obj.arrive_flag),
                    "stopped": bool(obj.stop_flag),
                    "static": bool(obj.static),
                }
            )
        return {
            "version": irsim.__version__,
            "time": self.env.time,
            "dt": self.env.step_time,
            "steps": self.steps,
            "frame_time": self.frame_time,
            "manual_control": self.manual_control,
            "done": bool(self.env.done()),
            "objects": objects,
        }

    def dispatch(self, request):
        if request["method"] == "create":
            return self.create(request["yaml"], request.get("seed", 0))
        if request["method"] == "step":
            return self.step(request.get("action"), request.get("count", 1))
        raise ValueError("Unknown playground command.")


playground = Playground()


def handle_request(raw):
    return json.dumps(playground.dispatch(json.loads(raw)), allow_nan=False)
