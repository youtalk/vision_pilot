"""Luminance arithmetic, no ROS. Run: python3 -m pytest test_lum_stats.py"""
import numpy as np
import lum_stats as l


def test_luminance_of_flat_grey_is_that_grey():
    rgb = np.full((4, 6, 3), 128, dtype=np.uint8)
    mean, p5, p95 = l.luminance(rgb)
    assert round(mean) == 128 and round(p5) == 128 and round(p95) == 128


def test_luminance_weights_green_most():
    g = np.zeros((2, 2, 3), dtype=np.uint8); g[..., 1] = 255
    r = np.zeros((2, 2, 3), dtype=np.uint8); r[..., 0] = 255
    assert l.luminance(g)[0] > l.luminance(r)[0]


def test_from_image_bgra_reorders_to_rgb():
    data = bytes([10, 20, 30, 255])          # one bgra8 pixel: b=10 g=20 r=30
    rgb = l.from_image(data, 1, 1, "bgra8")
    assert rgb.shape == (1, 1, 3) and list(rgb[0, 0]) == [30, 20, 10]


def test_write_ppm_header(tmp_path):
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    p = tmp_path / "f.ppm"; l.write_ppm(str(p), rgb)
    assert p.read_bytes().startswith(b"P6 3 2 255\n") and p.stat().st_size == len(b"P6 3 2 255\n") + 18
