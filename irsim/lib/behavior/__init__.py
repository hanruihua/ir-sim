"""
Behavior system for IR-SIM simulation.

This package contains:
- behavior: Base behavior class
- behavior_registry: Behavior registration system
- behavior_methods: Predefined behavior methods
"""

from .behavior import Behavior
from .behavior_registry import register_behavior

__all__ = ["Behavior", "register_behavior"]
