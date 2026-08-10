"""PyTorch input/output adapter for an IR-SIM environment."""

from __future__ import annotations

import importlib
from typing import Any

import numpy as np

from .wrapper import Wrapper


def _load_torch() -> Any:
    """Import PyTorch only when the wrapper is instantiated."""
    try:
        return importlib.import_module("torch")
    except ImportError as e:
        raise ImportError(
            "TorchWrapper requires PyTorch. Install it with `pip install torch`."
        ) from e


class TorchWrapper(Wrapper):
    """Adapt NumPy-based IR-SIM input and output to PyTorch tensors.

    The wrapped simulation continues to run on the CPU. Tensor actions are
    detached and converted to NumPy before :meth:`step`; NumPy observations from
    state and sensor getters are moved to ``device``. Singular getters preserve
    their original shape; plural robot getters add a leading robot dimension.

    Args:
        env: IR-SIM environment to wrap.
        device: PyTorch device receiving output tensors. Default is ``"cpu"``.
        dtype: Floating-point output dtype. A string such as ``"float32"``, a
            ``torch.dtype``, or ``None`` to preserve NumPy floating dtypes.
            Integer and boolean arrays always preserve their dtype.
    """

    def __init__(
        self,
        env: Any,
        device: Any = "cpu",
        dtype: Any = "float32",
    ) -> None:
        super().__init__(env)
        self._torch = _load_torch()
        self.device = self._torch.device(device)
        self.dtype = self._resolve_dtype(dtype)

    def _resolve_dtype(self, dtype: Any) -> Any:
        if not isinstance(dtype, str):
            return dtype
        try:
            return getattr(self._torch, dtype)
        except AttributeError as e:
            raise ValueError(f"Unknown torch dtype: {dtype!r}") from e

    def _dtype_for_numpy(self, value: np.ndarray) -> Any:
        if self.dtype is not None and value.dtype.kind in "fc":
            return self.dtype
        return None

    def to_tensor(self, value: Any) -> Any:
        """Recursively move NumPy arrays and tensors to the configured device."""
        if self._torch.is_tensor(value):
            dtype = self.dtype if value.is_floating_point() else None
            kwargs = {"device": self.device}
            if dtype is not None:
                kwargs["dtype"] = dtype
            return value.to(**kwargs)

        if isinstance(value, np.ndarray):
            return self._torch.as_tensor(
                value,
                device=self.device,
                dtype=self._dtype_for_numpy(value),
            )

        if isinstance(value, np.generic):
            array = np.asarray(value)
            return self._torch.as_tensor(
                array,
                device=self.device,
                dtype=self._dtype_for_numpy(array),
            )

        if isinstance(value, dict):
            return {key: self.to_tensor(item) for key, item in value.items()}
        if isinstance(value, list):
            return [self.to_tensor(item) for item in value]
        if isinstance(value, tuple):
            return tuple(self.to_tensor(item) for item in value)
        return value

    def to_numpy(self, value: Any) -> Any:
        """Recursively detach tensors and convert them to CPU NumPy values."""
        if self._torch.is_tensor(value):
            tensor = value.detach().cpu()
            return tensor.item() if tensor.ndim == 0 else tensor.numpy()
        if isinstance(value, dict):
            return {key: self.to_numpy(item) for key, item in value.items()}
        if isinstance(value, list):
            return [self.to_numpy(item) for item in value]
        if isinstance(value, tuple):
            return tuple(self.to_numpy(item) for item in value)
        return value

    def _normalize_robot_ids(self, ids: Any = None) -> list[int]:
        if ids is None:
            return list(range(len(self.env.robot_list)))

        ids = self.to_numpy(ids)
        if isinstance(ids, np.ndarray):
            ids = ids.tolist()
        if isinstance(ids, (int, np.integer)):
            ids = [ids]
        return [int(robot_id) for robot_id in ids]

    def _get_robot_attribute(
        self,
        attribute: str,
        ids: Any = None,
        *,
        stack: bool = True,
    ) -> Any:
        robot_ids = self._normalize_robot_ids(ids)
        values = [
            getattr(self.env.robot_list[robot_id], attribute) for robot_id in robot_ids
        ]
        if not stack:
            return [self.to_tensor(value) for value in values]
        if not values:
            return self.to_tensor(np.empty((0,), dtype=np.float32))
        if any(value is None for value in values):
            raise ValueError(
                f"Cannot stack robot {attribute!r} values containing None; "
                "pass stack=False to preserve them as a list."
            )

        shapes = {np.shape(value) for value in values}
        if len(shapes) != 1:
            raise ValueError(
                f"Cannot stack robot {attribute!r} values with different shapes; "
                "pass stack=False for heterogeneous robots."
            )
        return self.to_tensor(np.stack(values, axis=0))

    def _split_batch_action(self, action: Any, action_id: Any) -> Any:
        """Turn a homogeneous leading action batch into IR-SIM's action list."""
        if not isinstance(action, np.ndarray) or action.ndim not in (2, 3):
            return action
        if action.ndim == 2 and action.shape[1] == 1:
            return action
        if action.ndim == 3 and action.shape[2] != 1:
            return action

        batch_size = action.shape[0]
        if isinstance(action_id, list):
            if len(action_id) != batch_size:
                raise ValueError(
                    "Batched action and action_id must have the same leading size."
                )
            robot_ids = [int(robot_id) for robot_id in action_id]
        else:
            start = int(action_id)
            robot_ids = list(range(start, start + batch_size))

        if not robot_ids or not all(
            0 <= robot_id < len(self.env.robot_list) for robot_id in robot_ids
        ):
            raise IndexError("Batched action contains an invalid robot id.")

        split_actions = []
        for item, robot_id in zip(action, robot_ids, strict=True):
            robot = self.env.robot_list[robot_id]
            action_dim = item.shape[0]
            if action_dim != robot.vel_dim:
                raise ValueError(
                    f"Robot {robot_id} expects action_dim={robot.vel_dim}, "
                    f"but received {action_dim}."
                )
            split_actions.append(item.reshape(action_dim, 1))
        return split_actions

    def step(self, action: Any = None, action_id: Any = 0) -> Any:
        """Advance the CPU environment using singular or batched actions."""
        action = self.to_numpy(action)
        action_id = self.to_numpy(action_id)
        if isinstance(action_id, np.ndarray):
            action_id = action_id.tolist()
        if isinstance(action_id, tuple):
            action_id = list(action_id)
        action = self._split_batch_action(action, action_id)
        return self.to_tensor(self.env.step(action, action_id))

    def reset(self, *args: Any, **kwargs: Any) -> Any:
        """Reset the environment and adapt any future reset return value."""
        return self.to_tensor(self.env.reset(*args, **kwargs))

    def get_robot_state(self) -> Any:
        """Return the first robot state as a tensor without a batch dimension."""
        return self.to_tensor(self.env.get_robot_state())

    def get_robot_states(self, ids: Any = None, *, stack: bool = True) -> Any:
        """Return selected robot states with a leading robot dimension."""
        return self._get_robot_attribute("state", ids, stack=stack)

    def get_robot_goals(self, ids: Any = None, *, stack: bool = True) -> Any:
        """Return selected robot goals with a leading robot dimension."""
        return self._get_robot_attribute("goal", ids, stack=stack)

    def get_robot_velocities(self, ids: Any = None, *, stack: bool = True) -> Any:
        """Return selected robot velocities with a leading robot dimension."""
        return self._get_robot_attribute("velocity", ids, stack=stack)

    def get_robot_status(self, ids: Any = None) -> dict[str, Any]:
        """Return per-robot arrival and collision flags as boolean tensors."""
        robot_ids = self._normalize_robot_ids(ids)
        robots = [self.env.robot_list[robot_id] for robot_id in robot_ids]
        return self.to_tensor(
            {
                "arrive": np.asarray([robot.arrive for robot in robots], dtype=bool),
                "collision": np.asarray(
                    [robot.collision for robot in robots], dtype=bool
                ),
            }
        )

    def get_lidar_scan(self, id: int = 0) -> dict[str, Any]:
        """Return LiDAR array fields as tensors on the configured device."""
        return self.to_tensor(self.env.get_lidar_scan(id))

    def get_lidar_offset(self, id: int = 0) -> Any:
        """Return the LiDAR pose offset as a tensor."""
        return self.to_tensor(np.asarray(self.env.get_lidar_offset(id)))
