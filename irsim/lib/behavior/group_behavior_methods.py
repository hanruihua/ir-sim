from math import cos, sin
from typing import Any

import numpy as np

from irsim.lib.algorithm.social_force_model import SocialForceModelBatch
from irsim.lib.behavior.behavior_registry import register_group_behavior_class
from irsim.util.util import relative_position, vel_world2diff, vel_world2omni
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

    ORCA plans a collision-free holonomic velocity ``(vx, vy)`` in the world
    frame for every member. ``omni`` members have it rotated into their body
    frame; ``diff`` members map it to a ``(linear, angular)`` command via
    :func:`vel_world2diff`.
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

        ``diff`` members get the holonomic velocity mapped to ``(linear,
        angular)``; every other model converts it through its own kinematics,
        which for ``omni`` means rotating it into the body frame the model is
        commanded in.
        """
        if self._kinematics == "diff":
            return vel_world2diff(
                member.state[2, 0],
                [vel_xy[0], vel_xy[1]],
                w_max=float(member.vel_max[1, 0]),
                guarantee_time=member._world_param.step_time,
            )

        if self._kinematics == "omni":
            return vel_world2omni(member.state[2, 0], [vel_xy[0], vel_xy[1]])

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

        # Keep the kinematics mapping in sync with the call-time members, in
        # case the group was rebuilt or its members changed since __init__.
        if members:
            self._kinematics = members[0].kinematics

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


@register_group_behavior_class("omni", "sfm")
def beh_omni_sfm_group(members: list[ObjectBase], **kwargs: Any):
    """
    Registered initializer returning a class-based SFM handler for
    omnidirectional robots.
    """
    return SfmGroupBehavior(members, **kwargs)


@register_group_behavior_class("diff", "sfm")
def beh_diff_sfm_group(members: list[ObjectBase], **kwargs: Any):
    """
    Registered initializer returning a class-based SFM handler for
    differential-drive robots.
    """
    return SfmGroupBehavior(members, **kwargs)


class SfmGroupBehavior:
    """
    Class-based Social Force Model group behavior.

    Same physics and parameter names as the per-object ``sfm`` behavior,
    evaluated for every member at once by
    :class:`~irsim.lib.algorithm.social_force_model.SocialForceModelBatch`.
    All members read one state snapshot, so the crowd does not depend on the
    order objects are stepped in, and the pairwise social force is NumPy
    array math instead of a Python loop per member.

    Objects outside the group still take part: linestring obstacles enter as
    walls, every other object as a circular neighbour that pushes members
    but is not moved by them. ``target_roles`` filters those outside objects
    exactly as it does for the per-object behavior.

    ``social_groups`` optionally partitions the members into pedestrians
    walking together (Moussaid et al. 2010): those receive coherence,
    intra-group repulsion and gaze forces on top of the three base forces.
    Indices refer to the member order of the YAML group.

    A member without a goal has zero desired velocity: it stands still but
    still yields to neighbours. ``omni`` members receive the planned world
    velocity rotated into their body frame; ``diff`` members get it mapped
    to ``(linear, angular)`` via :func:`vel_world2diff`.
    """

    def __init__(
        self,
        members: list[ObjectBase],
        vmax: float = 1.5,
        neighbor_threshold: float = 10.0,
        relaxation_time: float = 0.5,
        force_factor_desired: float = 1.0,
        force_factor_social: float = 2.1,
        force_factor_obstacle: float = 10.0,
        sigma_obstacle: float = 0.8,
        lambda_importance: float = 2.0,
        gamma: float = 0.35,
        n_angular: float = 2.0,
        n_velocity: float = 3.0,
        safety_radius: float = 0.0,
        social_groups: list[list[int]] | None = None,
        force_factor_coherence: float = 2.0,
        force_factor_group_repulsion: float = 1.0,
        group_repulsion_threshold: float | None = None,
        force_factor_gaze: float = 3.0,
        gaze_vision_angle: float = 90.0,
        target_roles: str = "all",
        **kwargs: Any,
    ) -> None:
        self._vmax = float(vmax)
        self._target_roles = target_roles
        self._kinematics = members[0].kinematics if members else None
        self._step_time = members[0]._world_param.step_time if members else 0.1
        self._sfm = SocialForceModelBatch(
            states=np.zeros((len(members), 8)),
            social_groups=social_groups,
            vmax=self._vmax,
            step_time=self._step_time,
            relaxation_time=relaxation_time,
            force_factor_desired=force_factor_desired,
            force_factor_social=force_factor_social,
            force_factor_obstacle=force_factor_obstacle,
            sigma_obstacle=sigma_obstacle,
            lambda_importance=lambda_importance,
            gamma=gamma,
            n_angular=n_angular,
            n_velocity=n_velocity,
            neighbor_range=neighbor_threshold,
            safety_radius=safety_radius,
            force_factor_coherence=force_factor_coherence,
            force_factor_group_repulsion=force_factor_group_repulsion,
            group_repulsion_threshold=group_repulsion_threshold,
            force_factor_gaze=force_factor_gaze,
            gaze_vision_angle=gaze_vision_angle,
        )

    @property
    def sfm(self) -> SocialForceModelBatch:
        """The batch solver holding the tuned parameters."""
        return self._sfm

    def _collect_inputs(
        self, members: list[ObjectBase]
    ) -> tuple[np.ndarray, list[list[float]], list[list[float]]]:
        """Snapshot member states and the outside objects they react to.

        Returns:
            tuple: ``(states, circles, segments)`` where ``states`` is the
            ``(N, 8)`` member array with the desired velocity renormalised to
            ``vmax`` along the goal bearing (``rvo_state`` scales it by the
            kinematics-specific ``vel_max``), ``circles`` the neighbour
            states of non-member objects and ``segments`` the line obstacles.
        """
        states = np.array([m.rvo_state for m in members], dtype=float).reshape(-1, 8)
        v_des = states[:, 5:7]
        norm = np.hypot(v_des[:, 0], v_des[:, 1])
        moving = norm > 1e-9
        states[moving, 5:7] = self._vmax * v_des[moving] / norm[moving, None]

        circles: list[list[float]] = []
        segments: list[list[float]] = []
        if not members:
            return states, circles, segments

        member_ids = {id(m) for m in members}
        externals = [
            obj for obj in members[0].external_objects if id(obj) not in member_ids
        ]
        if self._target_roles in ("robot", "obstacle"):
            externals = [obj for obj in externals if obj.role == self._target_roles]

        for obj in externals:
            segs = obj.rvo_line_segments
            if segs:
                segments.extend(segs)
            else:
                circles.append(obj.rvo_neighbor_state)

        return states, circles, segments

    def _to_action(self, member: ObjectBase, vel_xy: np.ndarray) -> np.ndarray:
        """Convert a world-frame SFM velocity into a member control input."""
        vx, vy = float(vel_xy[0]), float(vel_xy[1])

        if self._kinematics == "diff":
            # Track the holonomic velocity tightly so the small sideways
            # component of the anisotropic forces survives the conversion.
            _, vmax_pair = member.get_vel_range()
            return vel_world2diff(
                member.state[2, 0],
                [vx, vy],
                w_max=float(vmax_pair[1, 0]),
                guarantee_time=self._step_time,
                tolerance=0.02,
            )

        if self._kinematics == "omni":
            return vel_world2omni(member.state[2, 0], np.array([[vx], [vy]]))

        return np.array([[vx], [vy]])

    def __call__(self, members: list[ObjectBase], **kwargs: Any) -> list[np.ndarray]:
        """
        Generate the velocity for every member from one shared snapshot.

        Args:
            members: the members of the group
            kwargs: unused; the parameters are fixed at construction

        Returns:
            list[np.ndarray]: one control input per member, in member order
        """
        if members:
            self._kinematics = members[0].kinematics

        states, circles, segments = self._collect_inputs(members)
        self._sfm.update(states, circles, segments)
        vel = self._sfm.cal_vel()

        return [self._to_action(m, vel[i]) for i, m in enumerate(members)]
