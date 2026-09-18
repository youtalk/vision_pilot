"""Pure colour-channel arithmetic behind jpeg_bridge. Run: python3 -m pytest test_jpeg_bridge.py

jpeg_bridge imports rclpy at module level, so import the one pure function
through the module only when rclpy is present; these tests cover the part
that can silently produce a garbled frame instead of failing.
"""
import numpy as np
import pytest

jb = pytest.importorskip("jpeg_bridge")


def test_bgra8_drops_alpha_and_keeps_channel_order():
    # One pixel, B=1 G=2 R=3 A=4.
    out = jb.to_bgr(bytes([1, 2, 3, 4]), 1, 1, "bgra8")
    assert out.shape == (1, 1, 3)
    assert list(out[0][0]) == [1, 2, 3]


def test_rgb8_is_reordered_to_bgr():
    out = jb.to_bgr(bytes([1, 2, 3]), 1, 1, "rgb8")
    assert list(out[0][0]) == [3, 2, 1]


def test_bgr8_passes_through():
    out = jb.to_bgr(bytes([1, 2, 3]), 1, 1, "bgr8")
    assert list(out[0][0]) == [1, 2, 3]


def test_unknown_encoding_raises_rather_than_reinterpreting():
    with pytest.raises(ValueError):
        jb.to_bgr(bytes([0] * 12), 2, 2, "mono8")


def test_shape_is_height_then_width():
    data = bytes(range(2 * 3 * 4))
    out = jb.to_bgr(data, 2, 3, "bgra8")
    assert out.shape == (2, 3, 3)
    assert np.array_equal(out[0][0], np.array([0, 1, 2], dtype=np.uint8))
