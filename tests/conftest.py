"""
Shared pytest fixtures for ir-sim test suite.

This module provides common fixtures for environment setup, matplotlib cleanup,
dummy objects, and other shared utilities used across test files.
"""

import contextlib
from collections.abc import Generator
from typing import Any

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pytest

import irsim
from irsim.config import env_param

# Force non-GUI backend for testing
matplotlib.use("Agg")


# ---------------------------------------------------------------------------
# Matplotlib cleanup fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def cleanup_matplotlib() -> Generator[None, None, None]:
    """Automatically close all matplotlib figures before and after each test."""
    plt.close("all")
    yield
    plt.close("all")


# ---------------------------------------------------------------------------
# Dummy world fixtures for testing plot and geometry components
# ---------------------------------------------------------------------------


class DummyWorld2D:
    """Minimal 2D world mock for testing EnvPlot without full environment."""

    def __init__(self) -> None:
        self.grid_map = np.zeros((10, 10))
        self.x_range = (0, 10)
        self.y_range = (0, 10)
        self.time = 0.0
        self.status = "ok"
        self.plot_parse = {}


class DummyWorld3D(DummyWorld2D):
    """Minimal 3D world mock for testing EnvPlot3D."""

    def __init__(self) -> None:
        super().__init__()
        self.z_range = (0, 5)


@pytest.fixture
def dummy_world_2d() -> DummyWorld2D:
    """Provide a minimal 2D world mock."""
    return DummyWorld2D()


@pytest.fixture
def dummy_world_3d() -> DummyWorld3D:
    """Provide a minimal 3D world mock."""
    return DummyWorld3D()


# ---------------------------------------------------------------------------
# Logger fixtures
# ---------------------------------------------------------------------------


class DummyLogger:
    """Silent logger that ignores all log messages for testing."""

    def info(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def warning(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def error(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def debug(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def critical(self, *_args: Any, **_kwargs: Any) -> None:
        pass


@pytest.fixture
def dummy_logger() -> Generator[DummyLogger, None, None]:
    """Install a dummy logger for testing and restore after."""
    original_logger = env_param.logger
    logger = DummyLogger()
    env_param.logger = logger
    yield logger
    env_param.logger = original_logger


def install_dummy_logger() -> None:
    """Install a dummy logger globally (for use in non-fixture contexts)."""
    env_param.logger = DummyLogger()


# ---------------------------------------------------------------------------
# Environment factory fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def env_factory():
    """Factory fixture to create environments with automatic cleanup."""
    created_envs: list[Any] = []

    def _make(yaml_file: str, **kwargs: Any) -> Any:
        defaults = {"save_ani": False, "display": False}
        defaults.update(kwargs)
        env = irsim.make(yaml_file, **defaults)
        created_envs.append(env)
        return env

    yield _make

    # Cleanup all created environments
    for env in created_envs:
        with contextlib.suppress(Exception):
            env.end()


# ---------------------------------------------------------------------------
# Common YAML configuration paths (relative to tests directory)
# ---------------------------------------------------------------------------


YAML_CONFIGS = {
    "collision_avoidance": "test_collision_avoidance.yaml",
    "all_objects": "test_all_objects.yaml",
    "collision_world": "test_collision_world.yaml",
    "multi_objects": "test_multi_objects_world.yaml",
    "grid_map": "test_grid_map.yaml",
    "keyboard_control": "test_keyboard_control.yaml",
    "keyboard_control2": "test_keyboard_control2.yaml",
    "fov_world": "test_fov_world.yaml",
    "render": "test_render.yaml",
    "custom_behavior": "custom_behavior.yaml",
    "duplicate_names": "test_duplicate_names.yaml",
}


# ---------------------------------------------------------------------------
# Keyboard mock helpers
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_keyboard_key():
    """Factory fixture to create mock keyboard keys."""
    with contextlib.suppress(ImportError):
        from unittest.mock import Mock

        from pynput import keyboard

        def _create_key(char: str) -> Mock:
            return Mock(spec=keyboard.Key, char=char)

        return _create_key

    # Fallback if pynput not available
    from unittest.mock import Mock

    def _create_key(char: str) -> Mock:
        key = Mock()
        key.char = char
        return key

    return _create_key


class MockMplEvent:
    """Mock matplotlib keyboard event."""

    def __init__(self, key: str) -> None:
        self.key = key


@pytest.fixture
def mock_mpl_event():
    """Factory fixture to create mock matplotlib events."""

    def _create(key: str) -> MockMplEvent:
        return MockMplEvent(key)

    return _create


# ---------------------------------------------------------------------------
# Parametrization helpers
# ---------------------------------------------------------------------------


PROJECTIONS = ["2d", "3d"]


def parametrize_projection(test_func):
    """Decorator to run a test with both 2D and 3D projections."""
    return pytest.mark.parametrize("projection", PROJECTIONS)(test_func)
