"""
Utility functions for IR-SIM simulation.

This package contains helper functions for:
- Mathematical operations
- Coordinate transformations  
- File operations
- Geometry utilities
"""

from .util import *

__all__ = [
    'WrapToPi', 'WrapToRegion', 'relative_position', 'distance', 'dist_hypot',
    'diff_to_omni', 'omni_to_diff', 'vertices_transform', 'geometry_transform',
    'transform_point_with_state', 'file_check', 'find_file', 'is_2d_list',
    'random_point_range', 'cross_product', 'is_convex_and_ordered',
    'gen_inequal_from_vertex', 'time_it', 'time_it2'
] 