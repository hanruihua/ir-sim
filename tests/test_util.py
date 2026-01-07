import math
import time

import numpy as np

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
