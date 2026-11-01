from collections.abc import Sequence
from itertools import pairwise
from math import cos, sin
from typing import Any

import numpy as np
from shapely.geometry.polygon import orient

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
    :func:`vel_world2diff`. The objects outside the group that
    ``target_roles`` selects, handed in as ``external_objects`` by
    :class:`~irsim.lib.behavior.group_behavior.GroupBehavior`, are avoided
    too: the obstacles by default, so a crowd does not make way for a robot
    crossing it, every outside object with ``target_roles: 'all'``. Static
    ones (walls, polygon and circle obstacles, the grid map) are obstacles to
    the simulator, added from their geometry when the group first moves
    (:meth:`_add_obstacles`) and again whenever the surroundings change
    (:meth:`_scene`); ``timeHorizonObst`` sets how far ahead they are
    considered. Moving ones (dynamic obstacles, and robots when selected) are
    agents of the simulator (:meth:`_moving_objects`): their position and
    velocity are fed in every step, so members avoid them as they avoid each
    other, and the velocity the simulator plans for them is discarded.
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
        self._ensure_pyrvo()
        self._sim = None
        self._loaded = None  # the scene the simulator was built for (:meth:`_scene`)

    def _ensure_pyrvo(self):
        try:
            import pyrvo  # type: ignore

            return pyrvo
        except ImportError as e:
            raise ImportError(
                "pyrvo is not installed. Please install it using `pip install pyrvo`."
            ) from e

    def _build_sim(self, members: list[ObjectBase], moving: Sequence[ObjectBase] = ()):
        """
        Add the members and the moving objects around them to the simulator.
        Args:
            members: the members of the group
            moving: the objects outside the group that move (:meth:`_moving_objects`)
        Returns:
            pyrvo.RVOSimulator: the simulator
        """

        pyrvo = self._ensure_pyrvo()
        sim = pyrvo.RVOSimulator()
        # Get step_time from first member's world_param
        step_time = members[0]._world_param.step_time if members else 0.1
        sim.set_time_step(step_time)

        for member in members:
            agent_max_speed = self._agent_speed(member)
            sim.add_agent(
                member.state[:2, 0].tolist(),
                self._neighborDist,
                self._maxNeighbors,
                self._timeHorizon,
                self._timeHorizonObst,
                float(member.radius + self._safe_radius),
                agent_max_speed,
            )

        for obj in moving:
            # seen by the members but never planned for: no neighbors of its own
            sim.add_agent(
                obj.state[:2, 0].tolist(),
                self._neighborDist,
                0,
                self._timeHorizon,
                self._timeHorizonObst,
                float(obj.radius + self._safe_radius),
                float(obj.max_speed),
            )

        return sim

    @staticmethod
    def _moving_objects(outside: Sequence[ObjectBase]) -> list[ObjectBase]:
        """The outside objects that move.

        They are agents of the simulator (:meth:`_build_sim`) placed at their
        position and velocity in the scene every step (:meth:`__call__`), so
        the members avoid them; what the simulator plans for them is ignored.
        """
        return [obj for obj in outside if not obj.static]

    @staticmethod
    def _scene(
        members: list[ObjectBase],
        moving: Sequence[ObjectBase],
        outside: Sequence[ObjectBase],
    ) -> tuple:
        """What the simulator holds: the agents in order, the static objects in place.

        The simulator is built again when this changes (:meth:`__call__`):
        an agent came, went or was replaced by another object, or a static
        object was added, removed or moved.
        """
        agents = tuple(obj.id for obj in (*members, *moving))
        walls = tuple((obj.id, obj.state.tobytes()) for obj in outside if obj.static)
        return agents, walls

    def _add_obstacles(self, outside: Sequence[ObjectBase]) -> int:
        """Hand the static outside objects to the simulator.

        Every static object among ``outside`` enters pyrvo through its
        geometry: a polygon (circle, rectangle and
        polygon obstacles) as its outer ring in counter-clockwise order plus
        one clockwise ring per hole, a line (linestring obstacles) as one
        two-vertex obstacle per segment, which pyrvo treats as a wall facing
        both ways. The grid map enters as its occupied region
        (:meth:`~irsim.world.map.obstacle_map.ObstacleMap.occupied_polygons`),
        polygons whose holes are the free pockets;
        a map without a grid enters through its geometry like any other
        object. Moving objects are agents instead (:meth:`_moving_objects`).

        Args:
            outside: the objects outside the group (:meth:`__call__`)

        Returns:
            int: the number of obstacles added
        """
        count = 0
        for obj in outside:
            if not obj.static:
                continue
            if obj.shape == "map" and obj.grid_map is not None:
                parts = obj.occupied_polygons()
            else:
                parts = list(getattr(obj.geometry, "geoms", [obj.geometry]))
            for part in parts:
                if part.geom_type == "Polygon":
                    # pyrvo: a solid is counter-clockwise, the pocket it bounds clockwise
                    oriented = orient(part, sign=1.0)
                    for ring in (oriented.exterior, *oriented.interiors):
                        coords = [
                            [float(x), float(y)]
                            for x, y in ring.simplify(0).coords[:-1]
                        ]
                        if len(coords) >= 2:
                            self._sim.add_obstacle(coords)
                            count += 1
                elif part.geom_type in ("LineString", "LinearRing"):
                    coords = list(part.simplify(0).coords)
                    for (x1, y1), (x2, y2) in pairwise(coords):
                        self._sim.add_obstacle(
                            [[float(x1), float(y1)], [float(x2), float(y2)]]
                        )
                        count += 1
        if count:
            self._sim.process_obstacles()
        return count

    @staticmethod
    def _realizable_speed(member: ObjectBase) -> float:
        """Fastest speed ``member`` can hold in every direction.

        ``max_speed`` of an ``omni`` member is the norm of its component
        limits, which is reachable only on the diagonal; a plan above the
        smaller component would be clipped per component on execution and
        leave ORCA's collision-free direction. ``diff`` members are bounded by
        their linear limit, which ``max_speed`` already is.
        """
        if member.kinematics in ("omni", "omni_angular"):
            return float(np.min(np.asarray(member.vel_max, dtype=float)[:2, 0]))
        return float(member.max_speed)

    def _agent_speed(self, member: ObjectBase) -> float:
        """Speed cap of one agent: its realizable speed, lowered to ``maxSpeed``."""
        realizable = self._realizable_speed(member)
        if self._maxSpeed is not None:
            return min(float(self._maxSpeed), realizable)
        return realizable

    def _pref_velocity(self, member: ObjectBase) -> list[float]:
        """ORCA preferred velocity for one member as world-frame ``[vx, vy]``.

        The direction is the goal bearing and the magnitude the agent's speed
        cap (:meth:`_agent_speed`), the same cap the simulator enforces, so
        members head for their goals at full speed when nothing blocks them
        and the plan stays within what the member can execute. ``diff``
        members take the bearing from the goal directly; ``omni`` members take
        it from the desired omni velocity, whose magnitude is discarded (it
        used to be clipped to 1 m/s, which held omni members at that speed).
        """
        speed = self._agent_speed(member)
        if self._kinematics == "diff":
            if member.goal is None:
                return [0.0, 0.0]
            _, radian = relative_position(member.state, member.goal)
            return [speed * cos(radian), speed * sin(radian)]

        vx, vy = member.get_desired_omni_vel().flatten()[:2]
        norm = float(np.hypot(vx, vy))
        if norm < 1e-9:
            return [0.0, 0.0]
        return [speed * vx / norm, speed * vy / norm]

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

    def __call__(
        self,
        members: list[ObjectBase],
        external_objects: Sequence[ObjectBase] = (),
        **kwargs: Any,
    ) -> list[np.ndarray]:
        """
        Generate the velocity for the group.
        Args:
            members: the members of the group
            external_objects: the objects outside the group that
                ``target_roles`` selects, gathered by
                :class:`~irsim.lib.behavior.group_behavior.GroupBehavior`
            kwargs: unused; the parameters are fixed at construction
        Returns:
            list[np.ndarray]: the velocities of the members
        """

        # Keep the kinematics mapping in sync with the call-time members, in
        # case the group was rebuilt or its members changed since __init__.
        if members:
            self._kinematics = members[0].kinematics

        # Start over when the surroundings changed: an agent came, went or was
        # replaced, or a static object was added, removed or moved
        moving = self._moving_objects(external_objects)
        scene = self._scene(members, moving, external_objects)
        if scene != self._loaded:
            self._sim = self._build_sim(members, moving)
            self._add_obstacles(external_objects)
            self._loaded = scene

        for i, member in enumerate(members):
            self._sim.set_agent_pref_velocity(i, self._pref_velocity(member))
            self._sim.set_agent_position(i, member.state[:2, 0].tolist())

        for i, obj in enumerate(moving, start=len(members)):
            velocity = obj.velocity_xy[:2, 0].tolist()
            self._sim.set_agent_position(i, obj.state[:2, 0].tolist())
            self._sim.set_agent_velocity(i, velocity)
            self._sim.set_agent_pref_velocity(i, velocity)

        self._sim.do_step()

        return [
            self._to_action(member, self._sim.get_agent_velocity(i).to_tuple())
            for i, member in enumerate(members)
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
    but is not moved by them. They are the ``external_objects`` that
    :class:`~irsim.lib.behavior.group_behavior.GroupBehavior` hands in, the
    ones ``target_roles`` selects, the obstacles by default.

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
        **kwargs: Any,
    ) -> None:
        self._vmax = float(vmax)
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
        self, members: list[ObjectBase], external_objects: Sequence[ObjectBase] = ()
    ) -> tuple[np.ndarray, list[list[float]], list[list[float]]]:
        """Snapshot member states and the outside objects they react to.

        Args:
            members: the members of the group
            external_objects: the objects outside the group (:meth:`__call__`)

        Returns:
            tuple: ``(states, circles, segments)`` where ``states`` is the
            ``(N, 8)`` member array with the desired velocity renormalised to
            ``vmax`` along the goal bearing (``rvo_state`` scales it by the
            kinematics-specific ``vel_max``), ``circles`` the neighbour
            states of the outside objects and ``segments`` their line
            obstacles.
        """
        states = np.array([m.rvo_state for m in members], dtype=float).reshape(-1, 8)
        v_des = states[:, 5:7]
        norm = np.hypot(v_des[:, 0], v_des[:, 1])
        moving = norm > 1e-9
        states[moving, 5:7] = self._vmax * v_des[moving] / norm[moving, None]

        circles: list[list[float]] = []
        segments: list[list[float]] = []
        for obj in external_objects:
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

    def __call__(
        self,
        members: list[ObjectBase],
        external_objects: Sequence[ObjectBase] = (),
        **kwargs: Any,
    ) -> list[np.ndarray]:
        """
        Generate the velocity for every member from one shared snapshot.

        Args:
            members: the members of the group
            external_objects: the objects outside the group that
                ``target_roles`` selects, gathered by
                :class:`~irsim.lib.behavior.group_behavior.GroupBehavior`
            kwargs: unused; the parameters are fixed at construction

        Returns:
            list[np.ndarray]: one control input per member, in member order
        """
        if members:
            self._kinematics = members[0].kinematics

        states, circles, segments = self._collect_inputs(members, external_objects)
        self._sfm.update(states, circles, segments)
        vel = self._sfm.cal_vel()

        return [self._to_action(m, vel[i]) for i, m in enumerate(members)]
