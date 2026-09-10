"""Test the actual adapter and exact-source bundling without a browser."""

import hashlib
import importlib.util
import json
import zipfile
from io import BytesIO
from pathlib import Path

import numpy as np
import pytest
import yaml
from matplotlib import pyplot as plt
from PIL import Image

DOCS = Path(__file__).resolve().parents[1]
ASSETS = DOCS / "source/_static/playground"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def simulator():
    simulator = load_module("browser_simulation", ASSETS / "simulation.py").Playground()
    yield simulator
    if simulator.env is not None:
        simulator.env.end(0)


def test_default_model(simulator):
    simulator.create((ASSETS / "kinematics.yaml").read_text())
    for _ in range(20):
        snapshot = simulator.step([0.8, 0.6])
    np.testing.assert_allclose(
        snapshot["objects"][0]["state"], [1.03852807, 2.25970740, 2.4]
    )
    assert snapshot["time"] == 4
    assert snapshot["frame_time"] == 4
    assert snapshot["manual_control"]
    image = Image.open(BytesIO(simulator.frame))
    assert image.size == (720, 576)
    np.testing.assert_array_equal(
        np.asarray(image), np.asarray(simulator.env._env_plot.fig.canvas.buffer_rgba())
    )
    json.dumps(snapshot, allow_nan=False)


def test_batch_keeps_physics_and_sample_time(simulator):
    config = yaml.safe_load((ASSETS / "kinematics.yaml").read_text())
    config["world"].update(step_time=0.05, sample_time=0.2)
    simulator.create(yaml.safe_dump(config))
    snapshot = simulator.step([0.8, 0.6], count=5)
    assert snapshot["time"] == 0.25
    assert snapshot["frame_time"] == 0.2
    assert snapshot["steps"] == 5
    simulator.create(yaml.safe_dump(config))
    for _ in range(5):
        single = simulator.step([0.8, 0.6])
    assert single["objects"] == snapshot["objects"]


@pytest.mark.parametrize("scene", ["kinematics", "obstacle", "lidar"])
def test_object_inspector_uses_native_attributes(simulator, scene):
    source = (ASSETS / f"{scene}.yaml").read_text()
    initial = simulator.create(source)
    initial_objects = json.dumps(initial["objects"])
    snapshot = simulator.step([0.8, 0.6], count=2)
    assert len(snapshot["objects"]) == len(simulator.env.objects)
    for data, obj in zip(snapshot["objects"], simulator.env.objects, strict=True):
        assert data == {
            "id": obj.id,
            "name": obj.name,
            "role": obj.role,
            "kinematics": obj.kinematics,
            "shape": obj.shape,
            "state": obj.state.ravel().tolist(),
            "velocity": obj.velocity.ravel().tolist(),
            "goal": None if obj.goal is None else obj.goal.ravel().tolist(),
            "collision": bool(obj.collision_flag),
            "arrived": bool(obj.arrive_flag),
            "stopped": bool(obj.stop_flag),
            "static": bool(obj.static),
        }
        if data["static"]:
            before = next(item for item in initial["objects"] if item["id"] == obj.id)
            assert data["state"] == before["state"]
    assert json.dumps(initial["objects"]) == initial_objects
    json.dumps(snapshot, allow_nan=False)


@pytest.mark.parametrize("sensor", ["lidar2d", "fmcw_lidar2d"])
def test_native_compound_sensors_and_trajectory(simulator, sensor):
    source = (
        (ASSETS / "lidar.yaml").read_text().replace("name: lidar2d", f"name: {sensor}")
    )
    simulator.create(source)
    before = simulator.frame
    simulator.step(count=5)
    robot = simulator.env.robot
    assert robot.shape == "compound"
    assert np.isfinite(robot.get_lidar_scan()["ranges"]).all()
    assert len(simulator.env._env_plot.ax.collections) >= 1
    assert len(robot.trajectory_line[0].get_xdata()) >= 5
    assert simulator.frame != before


def test_object_inspector_static_model_without_goal(simulator):
    config = yaml.safe_load((ASSETS / "kinematics.yaml").read_text())
    config["robot"]["kinematics"] = {"name": "static"}
    simulator.create(yaml.safe_dump(config))
    simulator.env.robot.set_goal(None)
    snapshot = simulator.snapshot()
    assert snapshot["objects"][0]["goal"] is None
    assert snapshot["objects"][0]["kinematics"] == "static"
    assert snapshot["objects"][0]["static"]
    json.dumps(snapshot, allow_nan=False)


def test_multiple_models_use_yaml_behaviors(simulator):
    config = yaml.safe_load((ASSETS / "kinematics.yaml").read_text())
    config["robot"] = [
        {
            "kinematics": {"name": kind},
            "state": [0, y, 0],
            "goal": [4, y, 0],
            "shape": {
                "name": "rectangle",
                "length": 0.6,
                "width": 0.3,
                "wheelbase": 0.4,
            },
            "behavior": {"name": "dash"},
        }
        for y, kind in enumerate(["diff", "omni", "acker"])
    ]
    snapshot = simulator.create(yaml.safe_dump(config))
    assert not snapshot["manual_control"]
    snapshot = simulator.step(count=3)
    assert all(obj["state"][0] > 0 for obj in snapshot["objects"])
    for data, obj in zip(snapshot["objects"], simulator.env.objects, strict=True):
        assert data["kinematics"] == obj.kinematics
        assert data["state"] == obj.state.ravel().tolist()
        assert data["velocity"] == obj.velocity.ravel().tolist()
        assert data["goal"] == obj.goal.ravel().tolist()
    with pytest.raises(ValueError, match="Slider commands"):
        simulator.step([0.8, 0.6])


@pytest.mark.parametrize("pixels", [[2000, 2000], [0, 500], [500], "large"])
def test_browser_frame_size_limit(simulator, pixels):
    config = yaml.safe_load((ASSETS / "kinematics.yaml").read_text())
    config["world"]["plot"] = {"figure_pixels": pixels}
    with pytest.raises(ValueError, match="figure_pixels"):
        simulator.create(yaml.safe_dump(config))


def test_failed_plot_leaves_no_figure_and_can_recover(simulator):
    config = yaml.safe_load((ASSETS / "kinematics.yaml").read_text())
    figures = plt.get_fignums()
    config["robot"]["color"] = "not-a-real-color"
    with pytest.raises(ValueError, match="color"):
        simulator.create(yaml.safe_dump(config))
    assert plt.get_fignums() == figures
    simulator.create((ASSETS / "kinematics.yaml").read_text())
    assert simulator.step([0.8, 0.6])["steps"] == 1


def test_collisions_and_reset(simulator):
    source = (ASSETS / "obstacle.yaml").read_text()
    simulator.create(source)
    for _ in range(100):
        snapshot = simulator.step([0.8, 0])
        if snapshot["done"]:
            break
    assert snapshot["done"]
    assert snapshot["objects"][0]["collision"]
    assert snapshot["objects"][0]["stopped"]
    reset = simulator.create(source)
    assert reset["time"] == 0
    assert not reset["objects"][0]["collision"]
    assert not reset["objects"][0]["stopped"]
    assert reset["objects"][0]["state"] == [1, 5, 0]


@pytest.mark.parametrize("dt", [0, -0.1, float("nan"), 1, "slow"])
def test_bounded_step_size(simulator, dt):
    config = yaml.safe_load((ASSETS / "kinematics.yaml").read_text())
    config["world"]["step_time"] = dt
    with pytest.raises(ValueError, match="step_time"):
        simulator.create(yaml.safe_dump(config))


@pytest.mark.parametrize(
    "source", ["[]", "null", "world: []", "robot: []", "x" * 64001]
)
def test_invalid_scene(simulator, source):
    with pytest.raises(ValueError, match=r"mapping|64,000"):
        simulator.create(source)


def test_step_budget(simulator):
    simulator.create((ASSETS / "kinematics.yaml").read_text())
    simulator.steps = 2000
    with pytest.raises(ValueError, match="budget"):
        simulator.step([0.8, 0.6])


def test_bundle_is_exact_and_reproducible(tmp_path):
    build = load_module("playground_build", DOCS / "playground_build.py").build_bundle
    first = build(DOCS.parent, tmp_path / "a")
    second = build(DOCS.parent, tmp_path / "b")
    assert first == second
    archive = tmp_path / "a/irsim-source.zip"
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == first["sha256"]
    with zipfile.ZipFile(archive) as bundle:
        for name in bundle.namelist():
            if name.startswith("irsim/"):
                assert bundle.read(name) == (DOCS.parent / name).read_bytes()
        assert f"ir_sim-{first['version']}.dist-info/METADATA" in bundle.namelist()
