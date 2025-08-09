"""
Utility functions for IR-SIM simulation.

This package contains helper functions for:
- Mathematical operations
- Coordinate transformations
- File operations
- Geometry utilities
"""

from .util import (
    WrapToPi,
    WrapToRegion,
    cross_product,
    diff_to_omni,
    dist_hypot,
    distance,
    file_check,
    find_file,
    gen_inequal_from_vertex,
    geometry_transform,
    is_2d_list,
    is_convex_and_ordered,
    omni_to_diff,
    random_point_range,
    relative_position,
    time_it,
    time_it2,
    transform_point_with_state,
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
    "find_file",
    "gen_inequal_from_vertex",
    "geometry_transform",
    "is_2d_list",
    "is_convex_and_ordered",
    "omni_to_diff",
    "random_point_range",
    "relative_position",
    "time_it",
    "time_it2",
    "transform_point_with_state",
    "vertices_transform",
]
