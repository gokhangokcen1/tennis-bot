import math

from tennis_bot_perception.ball_detector_node import BallDetectorNode


def test_center_pixel():
    x, y, z = BallDetectorNode.project_pixel_to_3d(
        320.0, 240.0, 2.0, 500.0, 500.0, 320.0, 240.0
    )
    assert math.isclose(x, 0.0)
    assert math.isclose(y, 0.0)
    assert math.isclose(z, 2.0)


def test_horizontal_offset():
    x, y, z = BallDetectorNode.project_pixel_to_3d(
        420.0, 240.0, 2.0, 500.0, 500.0, 320.0, 240.0
    )
    assert math.isclose(x, 0.4)
    assert math.isclose(y, 0.0)
    assert math.isclose(z, 2.0)
