"""
Utility functions for IR-SIM simulation.

This package contains helper functions for:
- Mathematical operations
- Coordinate transformations
- File operations
- Geometry utilities
"""

from .decorator import normalize_actions, plot_only, time_it, time_it2
from .message import resolve_message_targets
from .random import random_uniform, rng, set_seed
from .util import (
    WrapToPi,
    WrapToRegion,
    cross_product,
    diff_to_omni,
    dist_hypot,
    distance,
    file_check,
    find_duplicates,
    find_file,
    find_object_by_identity,
    gen_inequal_from_vertex,
    geometry_transform,
    is_2d_list,
    is_convex_and_ordered,
    omni_to_diff,
    random_point_range,
    relative_position,
    to_numpy,
    transform_point_with_state,
    vel_diff2world,
    vel_omni2world,
    vel_world2diff,
    vel_world2omni,
    vertices_transform,
)

__all__ = [
    "WrapToPi",
    "WrapToRegion",
    "cross_product",
    "diff_to_omni",
    "dist_hypot",
    "distance",
    "file_check",
    "find_duplicates",
    "find_file",
    "find_object_by_identity",
    "gen_inequal_from_vertex",
    "geometry_transform",
    "is_2d_list",
    "is_convex_and_ordered",
    "normalize_actions",
    "omni_to_diff",
    "plot_only",
    "random_point_range",
    "random_uniform",
    "relative_position",
    "resolve_message_targets",
    "rng",
    "set_seed",
    "time_it",
    "time_it2",
    "to_numpy",
    "transform_point_with_state",
    "vel_diff2world",
    "vel_omni2world",
    "vel_world2diff",
    "vel_world2omni",
    "vertices_transform",
]
