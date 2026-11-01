"""Measure dispatch changes against an explicit source checkout.

Run from the repository root with the same interpreter and thread limits for
both versions. Each measurement excludes scene creation, warm-up, trajectory
hashing and message serialization. Timing is reported, never asserted in pytest.
"""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import math
import os
import platform
import statistics
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

SCENARIOS = {
    "dash_1": (1, "dash"),
    "dash_10": (10, "dash"),
    "dash_100": (100, "dash"),
    "dash_1000": (1000, "dash"),
    "id_actions_1000": (1000, "explicit"),
    "sfm_group_200": (200, "group_sfm"),
}


def scene(number: int, behavior: str) -> dict:
    columns = math.ceil(math.sqrt(number))
    states = [
        [2.0 + (i % columns) * 1.2, 2.0 + (i // columns) * 1.2, 0.0]
        for i in range(number)
    ]
    robots = {
        "number": number,
        "distribution": {"name": "manual"},
        "kinematics": {"name": "diff"},
        "shape": {"name": "circle", "radius": 0.15},
        "state": states,
        "goal": [[x + 200.0, y, heading] for x, y, heading in states],
        "vel_max": [1.0, 1.0],
    }
    if behavior == "group_sfm":
        robots["group_behavior"] = {
            "name": "sfm",
            "vmax": 1.0,
            "neighbor_threshold": 5.0,
        }
    elif behavior == "dash":
        robots["behavior"] = {"name": "dash"}
    return {
        "world": {
            "width": 400,
            "height": 400,
            "step_time": 0.1,
            "sample_time": 0.1,
            "collision_mode": "unobstructed",
        },
        "robot": robots,
    }


def run_case(irsim, np, yaml, directory, name, warmup, steps):
    number, behavior = SCENARIOS[name]
    config = scene(number, behavior)
    config_json = json.dumps(config, sort_keys=True, separators=(",", ":"))
    path = directory / f"{name}.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    gc.collect()
    env = irsim.make(str(path), headless=True, seed=42, log_level="ERROR")
    try:
        if behavior == "explicit":
            actions = [np.array([[0.3], [0.0]]) for _ in range(number)]
            object_ids = [obj.id for obj in env.robot_list]

            def step():
                env.step(actions, action_id=object_ids)

        else:
            step = env.step
        for _ in range(warmup):
            step()
        samples = []
        for _ in range(steps):
            start = time.perf_counter_ns()
            step()
            samples.append((time.perf_counter_ns() - start) / 1e6)

        # Includes every trajectory state, so identical end points alone do not
        # hide differences in intermediate motion. This work is outside timing.
        trajectory = hashlib.sha256()
        for obj in env.objects:
            trajectory.update(np.asarray(obj.trajectory, dtype=np.float64).tobytes())
        final_snapshot = json.dumps(
            env.get_msg().to_dict(),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode()
        return {
            "samples_ms": samples,
            "median_ms": statistics.median(samples),
            "p95_ms": float(np.percentile(samples, 95)),
            "trajectory_sha256": trajectory.hexdigest(),
            "final_snapshot_sha256": hashlib.sha256(final_snapshot).hexdigest(),
            "scene_sha256": hashlib.sha256(config_json.encode()).hexdigest(),
        }
    finally:
        env.end(0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root", type=Path, default=Path(__file__).resolve().parents[1]
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument(
        "--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS)
    )
    args = parser.parse_args()
    if args.repeats < 1 or args.steps < 1 or args.warmup < 0:
        parser.error("repeats and steps must be positive; warmup must be non-negative")

    # Limit numerical libraries before importing them, including via irsim.
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    source = args.source_root.resolve()
    sys.path.insert(0, str(source))

    import matplotlib

    matplotlib.use("Agg")

    import numpy as np
    import shapely
    import yaml

    import irsim

    if not Path(irsim.__file__).resolve().is_relative_to(source):
        raise RuntimeError(
            f"Imported irsim from outside requested source: {irsim.__file__}"
        )

    result = {
        "label": args.label,
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "platform": platform.platform(),
            "python": sys.version,
            "numpy": np.__version__,
            "shapely": shapely.__version__,
            "matplotlib": matplotlib.__version__,
            "openblas_threads": 1,
            "omp_threads": 1,
        },
        "method": {
            "repeats": args.repeats,
            "steps_per_repeat": args.steps,
            "warmup_per_repeat": args.warmup,
            "seed": 42,
            "headless": True,
            "noise": False,
            "notes": "Fresh environment per repeat; JSON and hashing excluded from step timing. Desktop load and thermal state are uncontrolled.",
        },
        "source_sha256": {
            path: hashlib.sha256((source / path).read_bytes()).hexdigest()
            for path in (
                "irsim/lib/behavior/behavior_registry.py",
                "irsim/lib/behavior/behavior.py",
                "irsim/lib/behavior/behavior_methods.py",
                "irsim/world/object_base.py",
                "irsim/util/decorator.py",
            )
        },
        "scenarios": {},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    with tempfile.TemporaryDirectory(prefix="irsim-dispatch-") as directory:
        for name in args.scenarios:
            runs = []
            result["scenarios"][name] = {"runs": runs}
            for repeat in range(args.repeats):
                run = run_case(
                    irsim, np, yaml, Path(directory), name, args.warmup, args.steps
                )
                runs.append(run)
                save()
                print(
                    f"{args.label}: {name} {repeat + 1}/{args.repeats} "
                    f"median={run['median_ms']:.4f} ms p95={run['p95_ms']:.4f} ms",
                    flush=True,
                )
            result["scenarios"][name]["median_of_run_medians_ms"] = statistics.median(
                run["median_ms"] for run in runs
            )
            save()


if __name__ == "__main__":
    main()
