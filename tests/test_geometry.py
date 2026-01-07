"""
Tests for geometry handler functionality.

Covers GeometryFactory and various geometry types.
"""

import numpy as np
import pytest

from irsim.lib.handler.geometry_handler import GeometryFactory


class TestGeometryFactory:
    """Tests for GeometryFactory and geometry creation."""

    def test_create_circle(self):
        """Test creating circle geometry."""
        circle = GeometryFactory.create_geometry(
            "circle", center=[0.0, 0.0], radius=1.0
        )
        assert circle is not None

    def test_circle_get_gh(self):
        """Test getting G/h matrices for circle."""
        circle = GeometryFactory.create_geometry(
            "circle", center=[0.0, 0.0], radius=1.0
        )
        G, h, cone, convex = circle.get_circle_Gh(np.array([[0.0], [0.0]]), 1.0)
        assert G.shape == (3, 2)
        assert h.shape == (3, 1)
        assert cone == "norm2"
        assert convex is True

    def test_create_polygon(self):
        """Test creating polygon geometry."""
        polygon = GeometryFactory.create_geometry(
            "polygon", vertices=[(0, 0), (1, 0), (0, 1)]
        )
        assert polygon is not None

    def test_polygon_get_gh_none(self):
        """Test getting G/h matrices for polygon with None vertices."""
        polygon = GeometryFactory.create_geometry(
            "polygon", vertices=[(0, 0), (1, 0), (0, 1)]
        )
        G, h, cone, convex = polygon.get_polygon_Gh(None)
        assert G is None
        assert h is None
        assert cone is None
        assert convex is False

    def test_create_rectangle(self):
        """Test creating rectangle geometry."""
        rect = GeometryFactory.create_geometry("rectangle", length=2.0, width=1.0)
        assert rect is not None

    def test_invalid_geometry_name_raises(self):
        """Test invalid geometry name raises ValueError."""
        with pytest.raises(ValueError, match="Invalid geometry name"):
            GeometryFactory.create_geometry("not-exists")


class TestGeometryStep:
    """Tests for geometry step method (transformation)."""

    def test_circle_step(self):
        """Test circle geometry step transformation."""
        circle = GeometryFactory.create_geometry(
            "circle", center=[0.0, 0.0], radius=1.0
        )
        state = np.array([[1.0], [2.0], [0.0]])
        geometry = circle.step(state)
        assert geometry is not None

    def test_polygon_step(self):
        """Test polygon geometry step transformation."""
        polygon = GeometryFactory.create_geometry(
            "polygon", vertices=[(0, 0), (1, 0), (0, 1)]
        )
        state = np.array([[1.0], [2.0], [0.0]])
        geometry = polygon.step(state)
        assert geometry is not None
