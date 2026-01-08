from typing import Any, Optional

import numpy as np

from irsim.lib.behavior.behavior_registry import register_group_behavior_class
from irsim.world.object_base import ObjectBase


@register_group_behavior_class("omni", "orca")
def beh_omni_orca(members: list[ObjectBase], **kwargs: Any):
    """
    Registered initializer returning a class-based handler for ORCA.
    """
    return OrcaGroupBehavior(members, **kwargs)


class OrcaGroupBehavior:
    """
    Class-based ORCA group behavior with one-time initialization.
    """

    def __init__(
        self,
        members: list[ObjectBase],
        neighborDist: float = 15.0,
        maxNeighbors: int = 10,
        timeHorizon: float = 20.0,
        timeHorizonObst: float = 10.0,
        safe_radius: float = 0.1,
        maxSpeed: Optional[float] = None,
        **kwargs: Any,
    ) -> None:
        self._neighborDist = neighborDist
        self._maxNeighbors = maxNeighbors
        self._timeHorizon = timeHorizon
        self._timeHorizonObst = timeHorizonObst
        self._safe_radius = safe_radius
        self._maxSpeed = maxSpeed
        self._sim = self._build_sim(members, **kwargs)

    def _ensure_pyrvo(self):
        try:
            import pyrvo  # type: ignore

            return pyrvo
        except ImportError as e:
            raise ImportError(
                "pyrvo is not installed. Please install it using `pip install pyrvo`."
            ) from e

    def _build_sim(self, members: list[ObjectBase], **kwargs: Any):
        """
        Add members to the simulator.
        Args:
            members: the members of the group
            kwargs: the keyword arguments
        Returns:
            pyrvo.RVOSimulator: the simulator
        """

        pyrvo = self._ensure_pyrvo()
        sim = pyrvo.RVOSimulator()
        # Get step_time from first member's world_param
        step_time = members[0]._world_param.step_time if members else 0.1
        sim.set_time_step(step_time)

        for member in members:
            agent_max_speed = (
                float(self._maxSpeed)
                if self._maxSpeed is not None
                else member.max_speed
            )
            sim.add_agent(
                member.state[:2, 0].tolist(),
                self._neighborDist,
                self._maxNeighbors,
                self._timeHorizon,
                self._timeHorizonObst,
                float(member.radius + self._safe_radius),
                agent_max_speed,
            )

        return sim

    def __call__(self, members: list[ObjectBase], **kwargs: Any) -> list[np.ndarray]:
        """
        Generate the velocity for the group.
        Args:
            members: the members of the group
            kwargs: the keyword arguments
        Returns:
            list[np.ndarray]: the velocities of the members
        """

        # If agent count mismatches, rebuild
        try:
            if self._sim.get_num_agents() != len(members):
                self._sim = self._build_sim(members, **kwargs)
        except Exception:
            self._sim = self._build_sim(members, **kwargs)

        for i, member in enumerate(members):
            self._sim.set_agent_pref_velocity(
                i, member.get_desired_omni_vel(normalized=True).flatten().tolist()
            )
            self._sim.set_agent_position(i, member.state[:2, 0].tolist())

        self._sim.do_step()

        return [
            np.c_[list(self._sim.get_agent_velocity(i).to_tuple())]
            for i in range(self._sim.get_num_agents())
        ]
