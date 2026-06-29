from math import cos, sin
from typing import Any

import numpy as np

from irsim.lib.behavior.behavior_registry import register_group_behavior_class
from irsim.util.util import omni_to_diff, relative_position
from irsim.world.object_base import ObjectBase


@register_group_behavior_class("omni", "orca")
def beh_omni_orca(members: list[ObjectBase], **kwargs: Any):
    """
    Registered initializer returning a class-based handler for ORCA.
    """
    return OrcaGroupBehavior(members, **kwargs)


@register_group_behavior_class("diff", "orca")
def beh_diff_orca(members: list[ObjectBase], **kwargs: Any):
    """
    Registered initializer returning a class-based ORCA handler for
    differential-drive robots.
    """
    return OrcaGroupBehavior(members, **kwargs)


class OrcaGroupBehavior:
    """
    Class-based ORCA group behavior with one-time initialization.

    ORCA plans a collision-free holonomic velocity ``(vx, vy)`` for every
    member. ``omni`` members use it directly; ``diff`` members map it to a
    ``(linear, angular)`` command via :func:`omni_to_diff`.
    """

    def __init__(
        self,
        members: list[ObjectBase],
        neighborDist: float = 15.0,
        maxNeighbors: int = 10,
        timeHorizon: float = 20.0,
        timeHorizonObst: float = 10.0,
        safe_radius: float = 0.1,
        maxSpeed: float | None = None,
        **kwargs: Any,
    ) -> None:
        self._neighborDist = neighborDist
        self._maxNeighbors = maxNeighbors
        self._timeHorizon = timeHorizon
        self._timeHorizonObst = timeHorizonObst
        self._safe_radius = safe_radius
        self._maxSpeed = maxSpeed
        self._kinematics = members[0].kinematics if members else None
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

    def _pref_velocity(self, member: ObjectBase) -> list[float]:
        """ORCA preferred velocity for one member as world-frame ``[vx, vy]``.

        ``diff`` ``vel_max`` is ``[linear, angular]``, so the raw desired omni
        velocity would skew the preferred direction toward the x-axis. Build it
        from the true goal bearing scaled by the translational speed limit
        instead; ``omni`` members keep using the desired omni velocity.
        """
        if self._kinematics == "diff":
            if member.goal is None:
                return [0.0, 0.0]
            _, radian = relative_position(member.state, member.goal)
            speed = member.max_speed
            return [speed * cos(radian), speed * sin(radian)]
        return member.get_desired_omni_vel(normalized=True).flatten().tolist()

    def _to_action(self, member: ObjectBase, vel_xy: tuple[float, float]) -> np.ndarray:
        """Convert an ORCA world-frame velocity into a member control input.

        ``omni`` members consume ``(vx, vy)`` directly; ``diff`` members get the
        holonomic velocity mapped to ``(linear, angular)``.
        """
        if self._kinematics == "diff":
            return omni_to_diff(
                member.state[2, 0],
                [vel_xy[0], vel_xy[1]],
                w_max=float(member.vel_max[1, 0]),
                guarantee_time=member._world_param.step_time,
            )
        return np.c_[list(vel_xy)]

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
            self._sim.set_agent_pref_velocity(i, self._pref_velocity(member))
            self._sim.set_agent_position(i, member.state[:2, 0].tolist())

        self._sim.do_step()

        return [
            self._to_action(members[i], self._sim.get_agent_velocity(i).to_tuple())
            for i in range(self._sim.get_num_agents())
        ]
