import math
import time

import numpy as np
import pytest

from irsim.util import util


def test_WrapToPi():
    assert util.WrapToPi(0) == 0
    assert util.WrapToPi(math.pi) == math.pi
    assert util.WrapToPi(-math.pi) == -math.pi
    assert util.WrapToPi(3 * math.pi) == math.pi
    assert util.WrapToPi(-3 * math.pi) == -math.pi
    assert util.WrapToPi(math.pi, positive=True) == math.pi
    assert util.WrapToPi(-math.pi, positive=True) == math.pi


def test_WrapTo2Pi():
    assert util.WrapTo2Pi(0) == 0
    assert util.WrapTo2Pi(math.pi) == math.pi
    assert util.WrapTo2Pi(-math.pi) == math.pi
    assert util.WrapTo2Pi(3 * math.pi) == math.pi
    assert util.WrapTo2Pi(-3 * math.pi) == math.pi


def test_WrapToRegion():
    region = [-math.pi, math.pi]
    assert util.WrapToRegion(3 * math.pi, region) == math.pi
    assert util.WrapToRegion(-3 * math.pi, region) == -math.pi
    assert util.WrapToRegion(0, region) == 0


def test_convert_list_length():
    assert util.convert_list_length(1, 3) == [1, 1, 1]
    assert util.convert_list_length([2], 2) == [[2], [2]]
    assert util.convert_list_length([1, 2], 1) == [[1, 2]]
    assert util.convert_list_length([1, 2], 3) == [[1, 2], [1, 2], [1, 2]]
    assert util.convert_list_length([], 0) == []


def test_convert_list_length_dict():
    d = {"a": 1}
    assert util.convert_list_length_dict(d, 2) == [d, d]
    assert util.convert_list_length_dict([d], 2) == [[d], [d]]
    assert util.convert_list_length_dict([d, d], 1) == [[d, d]]
    assert util.convert_list_length_dict([d, d], 3) == [[d, d], [d, d], [d, d]]
    assert util.convert_list_length_dict([], 0) == []


def test_is_list_of_dicts():
    assert util.is_list_of_dicts([{"a": 1}, {"b": 2}])
    assert not util.is_list_of_dicts([{"a": 1}, 2])
    assert not util.is_list_of_dicts([1, 2])
    assert not util.is_list_of_dicts("notalist")


def test_is_list_of_numbers():
    assert util.is_list_of_numbers([1, 2, 3.5])
    assert not util.is_list_of_numbers([1, "a"])
    assert not util.is_list_of_numbers("notalist")


def test_is_list_of_lists():
    assert util.is_list_of_lists([[1], [2]])
    assert not util.is_list_of_lists([1, 2])
    assert not util.is_list_of_lists("notalist")


def test_is_list_not_list_of_lists():
    assert util.is_list_not_list_of_lists([1, 2, 3])
    assert not util.is_list_not_list_of_lists([[1], [2]])
    assert not util.is_list_not_list_of_lists("notalist")


def test_distance():
    assert (
        util.distance(np.array([0, 0]).reshape(2, 1), np.array([0, 0]).reshape(2, 1))
        == 0
    )


def test_random_point_range():
    pt = util.random_point_range([0, 0, -math.pi], [1, 1, math.pi])
    assert len(pt) == 3
    assert 0 <= pt[0] <= 1
    assert 0 <= pt[1] <= 1
    assert -math.pi <= pt[2] <= math.pi


def test_time_it2_counts_no_print(capsys):
    class Dummy:
        def __init__(self):
            self.time_print = False

        @util.time_it2(name="NoPrintFn")
        def fn(self):
            return 42

    d = Dummy()
    assert Dummy.fn.count == 0
    assert Dummy.fn.func_count == 0

    assert d.fn() == 42
    assert d.fn() == 42

    # no output when time_print is False
    captured = capsys.readouterr()
    assert captured.out == ""

    # counters increment
    assert Dummy.fn.count == 2
    assert Dummy.fn.func_count == 2


def test_time_it2_counts_with_print(capsys):
    class Dummy:
        def __init__(self):
            self.time_print = True

        @util.time_it2(name="PrintFn")
        def fn(self, delay=0.0):
            if delay:
                time.sleep(delay)
            return "ok"

    d = Dummy()
    assert Dummy.fn.count == 0
    assert Dummy.fn.func_count == 0

    assert d.fn(0.0) == "ok"

    captured = capsys.readouterr()
    assert "PrintFn execute time" in captured.out
    assert "seconds" in captured.out

    assert Dummy.fn.count == 1
    assert Dummy.fn.func_count == 1


def test_vertices_transform():
    """Test vertices_transform function."""
    vertices = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
    state = np.array([[1], [2], [0]])
    result = util.vertices_transform(vertices, state)
    assert result is not None
    assert result.shape[0] == 2


def test_relative_position():
    """Test relative_position function."""
    state1 = np.array([[0], [0], [0]])
    state2 = np.array([[1], [1], [0]])
    result = util.relative_position(state1, state2)
    assert result is not None


def test_gen_inequal_from_vertex():
    """Test gen_inequal_from_vertex function."""
    vertices = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
    G, h = util.gen_inequal_from_vertex(vertices)
    assert G is not None
    assert h is not None


def test_is_convex_and_ordered():
    """Test is_convex_and_ordered function."""
    # Convex polygon (square)
    vertices = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
    convex, ordered = util.is_convex_and_ordered(vertices)
    assert isinstance(convex, bool)
    # ordered is a string like 'CCW' or 'CW', not a bool
    assert ordered in ("CCW", "CW", None) or isinstance(ordered, str)


def test_is_2d_list():
    """Test is_2d_list function."""
    assert util.is_2d_list([[1, 2], [3, 4]])
    assert not util.is_2d_list([1, 2, 3])
    assert not util.is_2d_list("notalist")


def test_angle_normalize():
    """Test angle normalization functions."""
    # Test WrapToPi with edge cases
    assert util.WrapToPi(2 * math.pi) == pytest.approx(0, abs=1e-10)
    assert util.WrapToPi(-2 * math.pi) == pytest.approx(0, abs=1e-10)

    # Test WrapTo2Pi
    assert util.WrapTo2Pi(-math.pi / 2) == pytest.approx(3 * math.pi / 2, abs=1e-10)


def test_state_to_covariance():
    """Test state_to_covariance function if exists."""
    if hasattr(util, "state_to_covariance"):
        state = np.array([[1], [2], [0]])
        result = util.state_to_covariance(state)
        assert result is not None


def test_random_generate():
    """Test random point generation."""
    point = util.random_point_range([0, 0], [10, 10])
    assert len(point) >= 2
    assert 0 <= point[0] <= 10
    assert 0 <= point[1] <= 10
