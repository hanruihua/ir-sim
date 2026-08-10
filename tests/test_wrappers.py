"""Tests for environment wrappers without requiring PyTorch in the test suite."""

from __future__ import annotations

from unittest.mock import patch

import numpy as np
import pytest

from irsim.wrappers import TorchWrapper, Wrapper
from irsim.wrappers.torch_wrapper import _load_torch


class FakeTensor:
    """Small tensor double covering the methods used by TorchWrapper."""

    def __init__(self, value, device="cpu", dtype=None):
        self.array = np.asarray(value)
        self.device = device
        self.dtype = dtype

    @property
    def ndim(self):
        return self.array.ndim

    @property
    def shape(self):
        return self.array.shape

    def is_floating_point(self):
        return self.array.dtype.kind in "fc"

    def to(self, *, device, dtype=None):
        return FakeTensor(self.array, device=device, dtype=dtype)

    def detach(self):
        return self

    def cpu(self):
        return FakeTensor(self.array, device="cpu", dtype=self.dtype)

    def numpy(self):
        return self.array

    def item(self):
        return self.array.item()


class FakeTorch:
    float32 = object()
    float64 = object()

    @staticmethod
    def device(device):
        return device

    @staticmethod
    def is_tensor(value):
        return isinstance(value, FakeTensor)

    @staticmethod
    def as_tensor(value, *, device, dtype=None):
        return FakeTensor(value, device=device, dtype=dtype)


class FakeRobot:
    def __init__(
        self,
        state,
        goal,
        velocity,
        *,
        arrive=False,
        collision=False,
    ):
        self.state = np.asarray(state)
        self.goal = None if goal is None else np.asarray(goal)
        self.velocity = np.asarray(velocity)
        self.vel_dim = self.velocity.shape[0]
        self.arrive = arrive
        self.collision = collision


class FakeEnv:
    def __init__(self):
        self.name = "fake"
        self.last_action = None
        self.last_action_id = None
        self.reset_random = None
        self.robot_list = [
            FakeRobot(
                [[1.0], [2.0], [0.5]],
                [[5.0], [6.0], [0.0]],
                [[0.2], [0.1]],
                arrive=True,
            ),
            FakeRobot(
                [[3.0], [4.0], [1.0]],
                [[7.0], [8.0], [0.0]],
                [[0.4], [0.3]],
                collision=True,
            ),
        ]

    def step(self, action=None, action_id=0):
        self.last_action = action
        self.last_action_id = action_id
        return np.array([3.0])

    def reset(self, random=False):
        self.reset_random = random
        return np.array([1.0, 2.0])

    @staticmethod
    def get_robot_state():
        return np.array([[1.0], [2.0], [0.5]])

    @staticmethod
    def get_lidar_scan(id=0):
        return {
            "id": id,
            "ranges": np.array([1.0, 2.0]),
            "valid": np.array([True, False]),
            "intensities": None,
        }

    @staticmethod
    def get_lidar_offset(id=0):
        return [float(id), 0.0, 0.5]


@pytest.fixture
def torch_wrapper():
    with patch("irsim.wrappers.torch_wrapper._load_torch", return_value=FakeTorch):
        yield TorchWrapper(FakeEnv(), device="cuda")


def test_wrapper_proxies_attributes_and_unwraps_nested_wrappers():
    env = FakeEnv()
    wrapper = Wrapper(Wrapper(env))

    assert wrapper.name == "fake"
    assert wrapper.unwrapped is env


def test_torch_wrapper_requires_pytorch():
    with (
        patch(
            "irsim.wrappers.torch_wrapper.importlib.import_module",
            side_effect=ImportError("missing"),
        ),
        pytest.raises(ImportError, match="pip install torch"),
    ):
        _load_torch()


def test_load_torch_returns_imported_module():
    with patch(
        "irsim.wrappers.torch_wrapper.importlib.import_module",
        return_value=FakeTorch,
    ):
        assert _load_torch() is FakeTorch


def test_torch_wrapper_rejects_unknown_dtype():
    with (
        patch("irsim.wrappers.torch_wrapper._load_torch", return_value=FakeTorch),
        pytest.raises(ValueError, match="Unknown torch dtype"),
    ):
        TorchWrapper(FakeEnv(), dtype="not_a_dtype")


def test_step_converts_tensor_action_and_result(torch_wrapper):
    action = FakeTensor([[0.5], [0.1]], device="cuda")
    action_id = FakeTensor(2, device="cuda")

    result = torch_wrapper.step(action, action_id)

    assert isinstance(torch_wrapper.env.last_action, np.ndarray)
    assert np.array_equal(torch_wrapper.env.last_action, action.array)
    assert torch_wrapper.env.last_action_id == 2
    assert isinstance(result, FakeTensor)
    assert result.device == "cuda"


def test_step_converts_tensor_action_id_list(torch_wrapper):
    action_ids = FakeTensor([0, 1], device="cuda")

    torch_wrapper.step(None, action_ids)

    assert torch_wrapper.env.last_action_id == [0, 1]


def test_step_splits_leading_robot_action_batch(torch_wrapper):
    actions = FakeTensor([[0.5, 0.1], [0.2, -0.1]], device="cuda")

    torch_wrapper.step(actions)

    converted = torch_wrapper.env.last_action
    assert isinstance(converted, list)
    assert len(converted) == 2
    assert converted[0].shape == (2, 1)
    assert np.array_equal(converted[1], np.array([[0.2], [-0.1]]))


def test_step_splits_batch_for_tensor_robot_ids(torch_wrapper):
    actions = FakeTensor([[[0.5], [0.1]], [[0.2], [-0.1]]], device="cuda")
    robot_ids = FakeTensor([1, 0], device="cuda")

    torch_wrapper.step(actions, action_id=robot_ids)

    assert isinstance(torch_wrapper.env.last_action, list)
    assert torch_wrapper.env.last_action_id == [1, 0]


def test_step_splits_batch_for_tuple_robot_ids(torch_wrapper):
    actions = FakeTensor([[0.5, 0.1], [0.2, -0.1]], device="cuda")

    torch_wrapper.step(actions, action_id=(1, 0))

    assert isinstance(torch_wrapper.env.last_action, list)
    assert torch_wrapper.env.last_action_id == [1, 0]


def test_step_validates_batch_shape_and_robot_ids(torch_wrapper):
    with pytest.raises(ValueError, match="same leading size"):
        torch_wrapper.step(FakeTensor([[0.5, 0.1], [0.2, -0.1]]), action_id=[0])

    with pytest.raises(IndexError, match="invalid robot id"):
        torch_wrapper.step(FakeTensor([[0.5, 0.1], [0.2, -0.1]]), action_id=1)

    with pytest.raises(ValueError, match="expects action_dim=2"):
        torch_wrapper.step(FakeTensor([[0.5, 0.1, 0.2]]))


def test_step_preserves_non_batch_action_shapes(torch_wrapper):
    malformed = FakeTensor(np.zeros((2, 2, 2)))

    torch_wrapper.step(malformed)

    assert isinstance(torch_wrapper.env.last_action, np.ndarray)
    assert torch_wrapper.env.last_action.shape == (2, 2, 2)


def test_reset_and_state_getters_return_tensors(torch_wrapper):
    reset_result = torch_wrapper.reset(random=True)
    state = torch_wrapper.get_robot_state()
    offset = torch_wrapper.get_lidar_offset(2)

    assert torch_wrapper.env.reset_random is True
    assert isinstance(reset_result, FakeTensor)
    assert isinstance(state, FakeTensor)
    assert state.array.shape == (3, 1)
    assert state.dtype is FakeTorch.float32
    assert isinstance(offset, FakeTensor)
    assert offset.array.shape == (3,)


def test_batched_robot_getters_match_training_layout(torch_wrapper):
    states = torch_wrapper.get_robot_states()
    selected_state = torch_wrapper.get_robot_states(1)
    goals = torch_wrapper.get_robot_goals()
    velocities = torch_wrapper.get_robot_velocities(FakeTensor([1]))
    status = torch_wrapper.get_robot_status()

    assert states.array.shape == (2, 3, 1)
    assert selected_state.array.shape == (1, 3, 1)
    assert goals.array.shape == (2, 3, 1)
    assert velocities.array.shape == (1, 2, 1)
    assert np.array_equal(status["arrive"].array, [True, False])
    assert np.array_equal(status["collision"].array, [False, True])


def test_batched_robot_getters_support_empty_and_heterogeneous_lists(torch_wrapper):
    empty = torch_wrapper.get_robot_states([])
    torch_wrapper.env.robot_list[1].state = np.zeros((4, 1))

    with pytest.raises(ValueError, match="stack=False"):
        torch_wrapper.get_robot_states()

    states = torch_wrapper.get_robot_states(stack=False)

    assert empty.array.shape == (0,)
    assert [state.array.shape for state in states] == [(3, 1), (4, 1)]


def test_batched_goals_reject_missing_goal_when_stacking(torch_wrapper):
    torch_wrapper.env.robot_list[1].goal = None

    with pytest.raises(ValueError, match="containing None"):
        torch_wrapper.get_robot_goals()

    goals = torch_wrapper.get_robot_goals(stack=False)

    assert isinstance(goals[0], FakeTensor)
    assert goals[1] is None


def test_lidar_scan_preserves_metadata_and_integer_dtypes(torch_wrapper):
    scan = torch_wrapper.get_lidar_scan(4)

    assert scan["id"] == 4
    assert isinstance(scan["ranges"], FakeTensor)
    assert scan["ranges"].dtype is FakeTorch.float32
    assert isinstance(scan["valid"], FakeTensor)
    assert scan["valid"].dtype is None
    assert scan["intensities"] is None


def test_recursive_conversions_preserve_container_structure(torch_wrapper):
    values = (
        np.array([1.0]),
        [np.float64(2.0), {"flag": np.array([True])}],
    )

    tensors = torch_wrapper.to_tensor(values)
    arrays = torch_wrapper.to_numpy(tensors)

    assert isinstance(tensors, tuple)
    assert isinstance(tensors[1], list)
    assert isinstance(tensors[1][1], dict)
    assert isinstance(arrays[0], np.ndarray)
    assert isinstance(arrays[1][0], float)
    assert arrays[1][1]["flag"].dtype == np.bool_
    assert torch_wrapper.to_numpy("unchanged") == "unchanged"


def test_existing_tensors_move_to_device_and_convert_floating_dtype(torch_wrapper):
    floating = torch_wrapper.to_tensor(FakeTensor([1.0], device="cpu"))
    integer = torch_wrapper.to_tensor(FakeTensor([1], device="cpu"))

    assert floating.device == "cuda"
    assert floating.dtype is FakeTorch.float32
    assert integer.device == "cuda"
    assert integer.dtype is None


def test_dtype_none_preserves_floating_dtype():
    with patch("irsim.wrappers.torch_wrapper._load_torch", return_value=FakeTorch):
        wrapper = TorchWrapper(FakeEnv(), dtype=None)

    tensor = wrapper.to_tensor(np.array([1.0], dtype=np.float64))

    assert tensor.dtype is None
