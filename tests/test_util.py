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


def test_transform_none_inputs():
    """Test vertices_transform function with None inputs (line 355)."""
    # Vertices as None
    result = util.vertices_transform(None, np.array([[1], [2], [0]]))
    assert result is None

    # State as None
    result = util.vertices_transform(np.array([[0, 1], [0, 1]]), None)
    assert result is None


def test_diff_to_omni_scalar():
    """Test diff_to_omni with scalar/0-dim input (line 425)."""
    # 0-dimensional array edge case
    vel_diff = np.array(0.0)
    result = util.diff_to_omni(0.0, vel_diff)
    assert np.allclose(result, np.zeros((2, 1)))


def test_is_convex_few_points():
    """Test is_convex_and_ordered with less than 3 points (line 462)."""
    # Only 2 points - not a valid polygon
    points = np.array([[0, 1], [0, 1]])
    convex, ordering = util.is_convex_and_ordered(points)
    assert convex is False
    assert ordering is None


def test_convert_list_length_dict_extend():
    """Test convert_list_length_dict extending list (line 168)."""
    # List shorter than needed - should extend
    d = {"a": 1}
    result = util.convert_list_length_dict([d, d], 4)
    assert len(result) == 4
    # Last element repeated
    assert result[2] == [d, d]
    assert result[3] == [d, d]


# ---------------------------------------------------------------------------
# Coverage-targeted tests
# ---------------------------------------------------------------------------


def test_file_check_none():
    """file_check(None) returns None (line 33)."""
    assert util.file_check(None) is None


def test_wrap_to_region_short_range():
    """WrapToRegion with len(range) < 2 raises ValueError (line 122)."""
    with pytest.raises(ValueError, match="length >= 2"):
        util.WrapToRegion(0, [1])


def test_gen_inequal_from_vertex_non_convex():
    """gen_inequal_from_vertex with non-convex polygon returns None (lines 499-500)."""
    # L-shaped (non-convex) polygon
    vertices = np.array([[0, 2, 2, 1, 1, 0], [0, 0, 1, 1, 2, 2]])
    G, h = util.gen_inequal_from_vertex(vertices)
    assert G is None
    assert h is None


def test_gen_inequal_from_vertex_clockwise():
    """gen_inequal_from_vertex with CW polygon reverses vertices (line 503)."""
    # CW square: (0,0) -> (0,1) -> (1,1) -> (1,0)
    vertices = np.array([[0, 0, 1, 1], [0, 1, 1, 0]])
    G, h = util.gen_inequal_from_vertex(vertices)
    assert G is not None
    assert h is not None
    assert G.shape == (4, 2)


def test_random_point_range_none_args():
    """random_point_range with None args uses defaults (lines 565, 572)."""
    pt = util.random_point_range(None, None)
    assert pt.shape == (3, 1)


def test_random_point_range_1d_ndarray():
    """random_point_range with 1D ndarray args (lines 568-569, 575-576)."""
    low = np.array([0, 0, -math.pi])
    high = np.array([5, 5, math.pi])
    pt = util.random_point_range(low, high)
    assert pt.shape == (3, 1)


def test_is_2d_list_numpy():
    """is_2d_list with numpy array returns False (line 588)."""
    assert util.is_2d_list(np.array([[1, 2], [3, 4]])) is False


def test_validate_shape_non_array():
    """validate_shape raises TypeError for non-array (line 634)."""

    @util.validate_shape(state=3)
    def fn(state):
        return state

    with pytest.raises(TypeError, match="must be a numpy array"):
        fn(state=[1, 2, 3])


def test_validate_shape_small_array():
    """validate_shape raises ValueError for small array (line 638)."""

    @util.validate_shape(state=3)
    def fn(state):
        return state

    with pytest.raises(ValueError, match="shape\\[0\\] >= 3"):
        fn(state=np.array([[1], [2]]))


def test_validate_shape_passes():
    """validate_shape passes for valid array."""

    @util.validate_shape(state=3)
    def fn(state):
        return state

    result = fn(state=np.array([[1], [2], [3]]))
    assert result.shape == (3, 1)


def test_validate_length_non_sequence():
    """validate_length raises TypeError for non-sequence (lines 681-687)."""

    @util.validate_length(alpha=4)
    def fn(alpha):
        return alpha

    with pytest.raises(TypeError, match="must be a sequence"):
        fn(alpha=42)


def test_validate_length_short_sequence():
    """validate_length raises ValueError for short sequence (lines 688-692)."""

    @util.validate_length(alpha=4)
    def fn(alpha):
        return alpha

    with pytest.raises(ValueError, match="length >= 4"):
        fn(alpha=[1, 2])


def test_validate_length_passes():
    """validate_length passes for valid sequence."""

    @util.validate_length(alpha=2)
    def fn(alpha):
        return alpha

    result = fn(alpha=[1, 2, 3])
    assert result == [1, 2, 3]


def test_ensure_column_vector_list():
    """ensure_column_vector converts list to column (line 731)."""

    @util.ensure_column_vector("state")
    def fn(state):
        return state

    result = fn(state=[1, 2, 3])
    assert isinstance(result, np.ndarray)
    assert result.shape == (3, 1)


def test_ensure_column_vector_1d_array():
    """ensure_column_vector converts 1D array to column (line 733)."""

    @util.ensure_column_vector("state")
    def fn(state):
        return state

    result = fn(state=np.array([1, 2, 3]))
    assert result.shape == (3, 1)


def test_ensure_column_vector_none():
    """ensure_column_vector leaves None unchanged."""

    @util.ensure_column_vector("state")
    def fn(state=None):
        return state

    assert fn() is None


def test_ensure_numpy_list():
    """ensure_numpy converts list to ndarray (line 772)."""

    @util.ensure_numpy("data")
    def fn(data):
        return data

    result = fn(data=[1, 2, 3])
    assert isinstance(result, np.ndarray)
    np.testing.assert_array_equal(result, [1, 2, 3])


def test_ensure_numpy_none():
    """ensure_numpy leaves None unchanged."""

    @util.ensure_numpy("data")
    def fn(data=None):
        return data

    assert fn() is None


def test_to_numpy_none():
    """to_numpy(None) returns None (line 888)."""
    assert util.to_numpy(None) is None


def test_to_numpy_none_with_default():
    """to_numpy(None, default=...) returns default."""
    default = np.array([[1], [2]])
    result = util.to_numpy(None, default=default)
    np.testing.assert_array_equal(result, default)


def test_to_numpy_list():
    """to_numpy with list converts to column vector."""
    result = util.to_numpy([1, 2, 3])
    assert isinstance(result, np.ndarray)
    assert result.shape == (3, 1)


def test_traj_to_xy_list_ndarray_multi_column():
    """traj_to_xy_list with ndarray multi-column (lines 921-924)."""
    traj = np.array([[1, 2, 3], [4, 5, 6], [0.1, 0.2, 0.3]])
    x_list, y_list = util.traj_to_xy_list(traj)
    assert x_list == [1, 2, 3]
    assert y_list == [4, 5, 6]


def test_traj_to_xy_list_ndarray_single_column():
    """traj_to_xy_list with ndarray single column (lines 927-929)."""
    traj = np.array([[1], [4], [0.1]])
    x_list, y_list = util.traj_to_xy_list(traj)
    assert x_list == [np.array([1])]
    assert y_list == [np.array([4])]


def test_traj_to_xy_list_ndarray_3d():
    """traj_to_xy_list with three_d=True (lines 925-926)."""
    traj = np.array([[1, 2], [4, 5], [7, 8]])
    x_list, y_list, z_list = util.traj_to_xy_list(traj, three_d=True)
    assert x_list == [1, 2]
    assert y_list == [4, 5]
    assert z_list == [7, 8]


def test_traj_to_xy_list_ndarray_single_column_3d():
    """traj_to_xy_list with single column 3D (lines 930-931)."""
    traj = np.array([[1], [4], [7]])
    x_list, _y_list, z_list = util.traj_to_xy_list(traj, three_d=True)
    assert len(x_list) == 1
    assert len(z_list) == 1


def test_traj_to_xy_list_invalid_type():
    """traj_to_xy_list with invalid type raises ValueError (line 933)."""
    with pytest.raises(ValueError, match="Invalid trajectory type"):
        util.traj_to_xy_list("bad")


def test_points_to_xy_list_none():
    """points_to_xy_list(None) returns empty lists (line 956)."""
    x, y = util.points_to_xy_list(None)
    assert x == []
    assert y == []


def test_points_to_xy_list_invalid_type():
    """points_to_xy_list with invalid type raises ValueError (line 975)."""
    with pytest.raises(ValueError, match="Invalid points type"):
        util.points_to_xy_list("bad")


def test_points_to_xy_list_ndarray_single_column():
    """points_to_xy_list with ndarray single column (lines 970-971)."""
    pts = np.array([[1], [2]])
    x, y = util.points_to_xy_list(pts)
    assert len(x) == 1
    assert len(y) == 1


def test_points_to_xy_list_ndarray_3d():
    """points_to_xy_list with three_d=True (lines 967-968)."""
    pts = np.array([[1, 2], [3, 4], [5, 6]])
    x, y, z = util.points_to_xy_list(pts, three_d=True)
    assert x == [1, 2]
    assert y == [3, 4]
    assert z == [5, 6]


def test_points_to_xy_list_ndarray_single_3d():
    """points_to_xy_list single column 3D (lines 972-973)."""
    pts = np.array([[1], [3], [5]])
    x, _y, z = util.points_to_xy_list(pts, three_d=True)
    assert len(x) == 1
    assert len(z) == 1


def test_traj_to_xy_list_list_3d():
    """traj_to_xy_list with list input and three_d=True (line 920)."""
    traj = [np.array([[1], [2], [3]]), np.array([[4], [5], [6]])]
    x, y, z = util.traj_to_xy_list(traj, three_d=True)
    assert x == [1, 4]
    assert y == [2, 5]
    assert z == [3, 6]


def test_points_to_xy_list_list_3d():
    """points_to_xy_list with list input and three_d=True (line 962)."""
    pts = [[1, 2, 3], [4, 5, 6]]
    x, y, z = util.points_to_xy_list(pts, three_d=True)
    assert x == [1, 4]
    assert y == [2, 5]
    assert z == [3, 6]


# ---------------------------------------------------------------------------
# check_unknown_kwargs tests
# ---------------------------------------------------------------------------


def test_check_unknown_kwargs_exact_match():
    """No warnings when all keys are valid."""
    valid = {"alpha", "beta", "gamma"}
    msgs = util.check_unknown_kwargs({"alpha": 1, "beta": 2}, valid)
    assert msgs == []


def test_check_unknown_kwargs_close_typo():
    """Suggests correct name for close typo."""
    valid = {"step_time", "sample_time", "height", "width"}
    msgs = util.check_unknown_kwargs({"step_tim": 0.1}, valid)
    assert len(msgs) == 1
    assert "Did you mean" in msgs[0]
    assert "step_time" in msgs[0]


def test_check_unknown_kwargs_no_close_match():
    """Lists all valid params when no close match exists."""
    valid = {"alpha", "beta"}
    msgs = util.check_unknown_kwargs({"zzzzz": 1}, valid)
    assert len(msgs) == 1
    assert "Valid parameters:" in msgs[0]
    assert "alpha" in msgs[0]
    assert "beta" in msgs[0]


def test_check_unknown_kwargs_empty():
    """No warnings for empty kwargs."""
    valid = {"alpha", "beta"}
    msgs = util.check_unknown_kwargs({}, valid)
    assert msgs == []


def test_check_unknown_kwargs_with_logger():
    """Logger.warning is called for each unknown key."""
    warnings = []

    class FakeLogger:
        def warning(self, msg):
            warnings.append(msg)

    valid = {"step_time", "height"}
    msgs = util.check_unknown_kwargs(
        {"step_tim": 0.1, "xyz": 1}, valid, logger=FakeLogger()
    )
    assert len(msgs) == 2
    assert len(warnings) == 2


def test_check_unknown_kwargs_context():
    """Context string is included in message."""
    valid = {"alpha"}
    msgs = util.check_unknown_kwargs({"alph": 1}, valid, context=" in 'world' config")
    assert len(msgs) == 1
    assert "in 'world' config" in msgs[0]
