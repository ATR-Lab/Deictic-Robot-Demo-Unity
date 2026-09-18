"""Sensor encoding checks; run with the ROS environment sourced."""

import numpy as np
import pytest

pytest.importorskip("rclpy")
from sensor_msgs.msg import Image

from deictic_registration.core import RegistrationError
from deictic_registration.node import depth_image, rgb_image


def test_padded_bgr_image_preserves_rows_and_colors():
    message = Image(width=2, height=2, encoding="bgr8", step=8)
    message.data = [1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10, 11, 12, 0, 0]
    decoded = rgb_image(message)
    assert decoded.tolist() == [[[3, 2, 1], [6, 5, 4]], [[9, 8, 7], [12, 11, 10]]]


@pytest.mark.parametrize("big_endian", [False, True])
def test_depth_handles_byte_order_without_changing_meters(big_endian):
    z = np.array([[0.25, 2.0], [np.nan, 0.0]], dtype=">f4" if big_endian else "<f4")
    message = Image(width=2, height=2, encoding="32FC1", step=8,
                    is_bigendian=int(big_endian), data=z.tobytes())
    decoded = depth_image(message)
    assert decoded[0].tolist() == [0.25, 2.0]
    assert np.isnan(decoded[1, 0])
    assert decoded[1, 1] == 0


def test_unknown_depth_units_are_rejected():
    message = Image(width=2, height=2, encoding="16UC1", step=4,
                    data=np.zeros((2, 2), np.uint16).tobytes())
    with pytest.raises(RegistrationError, match="optical-z meters"):
        depth_image(message)
