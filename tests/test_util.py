import pytest
import numpy as np
from irsim.util import util
import math

def test_WrapToPi():
    assert util.WrapToPi(0) == 0
    assert util.WrapToPi(math.pi) == math.pi
    assert util.WrapToPi(-math.pi) == -math.pi
    assert util.WrapToPi(3*math.pi) == math.pi
    assert util.WrapToPi(-3*math.pi) == -math.pi
    assert util.WrapToPi(math.pi, positive=True) == math.pi
    assert util.WrapToPi(-math.pi, positive=True) == math.pi

def test_WrapToRegion():
    region = [-math.pi, math.pi]
    assert util.WrapToRegion(3*math.pi, region) == math.pi
    assert util.WrapToRegion(-3*math.pi, region) == -math.pi
    assert util.WrapToRegion(0, region) == 0

def test_convert_list_length():
    assert util.convert_list_length(1, 3) == [1, 1, 1]
    assert util.convert_list_length([2], 2) == [[2], [2]]
    assert util.convert_list_length([1, 2], 1) == [[1,2]]
    assert util.convert_list_length([1, 2], 3) == [[1, 2], [1, 2], [1, 2]]
    assert util.convert_list_length([], 0) == []

def test_convert_list_length_dict():
    d = {'a': 1}
    assert util.convert_list_length_dict(d, 2) == [d, d]
    assert util.convert_list_length_dict([d], 2) == [[d], [d]]
    assert util.convert_list_length_dict([d, d], 1) == [[d, d]]
    assert util.convert_list_length_dict([d, d], 3) == [[d, d], [d, d], [d, d]]
    assert util.convert_list_length_dict([], 0) == []

def test_is_list_of_dicts():
    assert util.is_list_of_dicts([{'a': 1}, {'b': 2}])
    assert not util.is_list_of_dicts([{'a': 1}, 2])
    assert not util.is_list_of_dicts([1, 2])
    assert not util.is_list_of_dicts('notalist')

def test_is_list_of_numbers():
    assert util.is_list_of_numbers([1, 2, 3.5])
    assert not util.is_list_of_numbers([1, 'a'])
    assert not util.is_list_of_numbers('notalist')

def test_is_list_of_lists():
    assert util.is_list_of_lists([[1], [2]])
    assert not util.is_list_of_lists([1, 2])
    assert not util.is_list_of_lists('notalist')

def test_is_list_not_list_of_lists():
    assert util.is_list_not_list_of_lists([1, 2, 3])
    assert not util.is_list_not_list_of_lists([[1], [2]])
    assert not util.is_list_not_list_of_lists('notalist')

def test_distance():
    assert util.distance(np.array([0, 0]).reshape(2, 1), np.array([0, 0]).reshape(2, 1)) == 0

def test_random_point_range():
    pt = util.random_point_range([0, 0, -math.pi], [1, 1, math.pi])
    assert len(pt) == 3
    assert 0 <= pt[0] <= 1
    assert 0 <= pt[1] <= 1
    assert -math.pi <= pt[2] <= math.pi 

if __name__ == "__main__":
    test_WrapToPi()
    test_WrapToRegion()
    test_convert_list_length()
    test_convert_list_length_dict()
    test_is_list_of_dicts()
    test_is_list_of_numbers()
    test_is_list_of_lists()
    test_distance()
    test_random_point_range()