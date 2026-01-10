# custom_group_methods.py
"""
Custom group behavior examples for IR-SIM.

This module demonstrates two patterns for custom group behaviors:
1. Class-based (register_group_behavior_class) - for behaviors with initialization
2. Function-based (register_group_behavior) - for simple stateless behaviors
"""
from typing import Any

import numpy as np

from irsim.lib.behavior.behavior_registry import (
    register_group_behavior,
    register_group_behavior_class,
)
from irsim.world.object_base import ObjectBase

# =============================================================================
# Class-based Group Behavior Example (with initialization)
# =============================================================================


@register_group_behavior_class("omni", "formation")
def init_formation_behavior(members: list[ObjectBase], **kwargs: Any):
    """Registered initializer returning a class-based handler."""
    return FormationGroupBehavior(members, **kwargs)


class FormationGroupBehavior:
    """
    Class-based formation group behavior.
    Maintains formation shape while moving toward goals.
    """

    def __init__(
        self,
        members: list[ObjectBase],
        formation_gain: float = 1.0,
        goal_gain: float = 0.5,
        **kwargs: Any,
    ) -> None:
        """
        Initialize formation behavior.

        Args:
            members: List of group members
            formation_gain: Weight for formation keeping
            goal_gain: Weight for goal reaching
        """
        self._formation_gain = formation_gain
        self._goal_gain = goal_gain

        # Calculate initial formation offsets (relative positions)
        if members:
            centroid = np.mean([m.state[:2, 0] for m in members], axis=0)
            self._offsets = [m.state[:2, 0] - centroid for m in members]
        else:
            self._offsets = []

    def __call__(self, members: list[ObjectBase], **kwargs: Any) -> list[np.ndarray]:
        """
        Generate velocities for all members to maintain formation.

        Args:
            members: Current group members
            **kwargs: Additional parameters

        Returns:
            List of velocity arrays for each member
        """
        if not members:
            return []

        velocities = []

        # Calculate current centroid
        centroid = np.mean([m.state[:2, 0] for m in members], axis=0)

        # Calculate target centroid (average of all goals)
        goals = [m.goal[:2, 0] for m in members if m.goal is not None]
        target_centroid = np.mean(goals, axis=0) if goals else centroid

        for i, member in enumerate(members):
            # Desired position in formation
            desired_pos = (
                centroid + self._offsets[i]
                if i < len(self._offsets)
                else member.state[:2, 0]
            )

            # Formation keeping velocity
            formation_vel = self._formation_gain * (desired_pos - member.state[:2, 0])

            # Goal reaching velocity
            goal_vel = self._goal_gain * (target_centroid - centroid)

            # Combined velocity
            vel = formation_vel + goal_vel

            # Clip to max speed
            speed = np.linalg.norm(vel)
            if speed > member.max_speed:
                vel = vel / speed * member.max_speed

            velocities.append(np.c_[vel])

        return velocities


# =============================================================================
# Function-based Group Behavior Example (stateless)
# =============================================================================


@register_group_behavior("omni", "swarm")
def beh_omni_swarm(members: list[ObjectBase], **kwargs: Any) -> list[np.ndarray]:
    """
    Simple swarm behavior - members move toward group centroid while pursuing goals.

    This is a stateless behavior called every simulation step.
    Use this pattern for simple behaviors without initialization requirements.

    Args:
        members: List of group members
        **kwargs: Additional parameters (cohesion_gain, goal_gain)

    Returns:
        List of velocity arrays for each member
    """
    if not members:
        return []

    cohesion_gain = kwargs.get("cohesion_gain", 0.5)
    goal_gain = kwargs.get("goal_gain", 1.0)

    # Calculate centroid
    centroid = np.mean([m.state[:2, 0] for m in members], axis=0)

    velocities = []
    for member in members:
        # Cohesion: move toward centroid
        cohesion_vel = cohesion_gain * (centroid - member.state[:2, 0])

        # Goal: move toward individual goal
        if member.goal is not None:
            goal_vel = goal_gain * (member.goal[:2, 0] - member.state[:2, 0])
        else:
            goal_vel = np.zeros(2)

        vel = cohesion_vel + goal_vel

        # Clip to max speed
        speed = np.linalg.norm(vel)
        if speed > member.max_speed:
            vel = vel / speed * member.max_speed

        velocities.append(np.c_[vel])

    return velocities
