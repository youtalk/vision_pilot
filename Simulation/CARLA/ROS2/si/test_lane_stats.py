#!/usr/bin/env python3
"""lane_stats must count both line forms and must not count anything else."""
from lane_stats import format_line, summarise

FIT = ("[INFO]  [Lateral] Path: raw_CTE=-0.40m  bias=0.00m  corr=-0.40m  yaw=0.031rad  "
       "κ=0.0012  (37 pts, xmin=5.0m) | AD-κ=0.0011 (raw=0.0010) | Fused CTE=-0.40m")
NO_FIT = ("[INFO]  [Lateral] Path: (no path fit, 3 pts) | AD-κ=(none) | "
          "Fused CTE=0.00m (0.00m/s) yaw=0.000rad (0.000rad/s) κ=0.0000")
NOISE = "[INFO]  [Longitudinal] something entirely different"

cycles, fits, ctes, pts = summarise([FIT, NO_FIT, NOISE, FIT])
assert (cycles, fits) == (3, 2), (cycles, fits)
assert pts == [37, 37] and ctes == [0.40, 0.40], (pts, ctes)

line = format_line(cycles, fits, ctes, pts)
assert "fit_rate=0.667" in line, line
assert "median_abs_cte_m=0.40" in line, line

assert format_line(*summarise([NOISE])) == "LANE_BASELINE_FAIL reason=no_lateral_lines"
print("TEST_PASS test_lane_stats")
