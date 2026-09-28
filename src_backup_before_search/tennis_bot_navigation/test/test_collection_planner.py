import math

from tennis_bot_navigation.collection_planner_node import (
    Ball,
    euclidean_distance,
    nearest_ball_index,
)


def test_distance():
    assert math.isclose(euclidean_distance(0, 0, 3, 4), 5.0)


def test_nearest():
    balls = [Ball(2, 0), Ball(0.5, 0), Ball(4, 0)]
    assert nearest_ball_index(0, 0, balls) == 1


def test_empty():
    assert nearest_ball_index(0, 0, []) is None
