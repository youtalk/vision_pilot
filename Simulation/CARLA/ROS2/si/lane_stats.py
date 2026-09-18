#!/usr/bin/env python3
"""Summarise VisionPilot's [Lateral] debug lines into one lane-detection baseline.

VisionPilot prints one such line per cycle when `fusion.debug = true`. A cycle
either fits a lane path or reports "(no path fit, N pts)". The fit rate is the
number the CARLA rig has to be re-baselined on after a render change, because a
lane departure caused by a thin render looks exactly like a planner fault.

  lane_stats.py <vp.log>          prints LANE_BASELINE ... or LANE_BASELINE_FAIL
"""
import re
import statistics
import sys

FIT = re.compile(r"\[Lateral\] Path: raw_CTE=(-?[\d.]+)m.*?\((\d+) pts, xmin=(-?[\d.]+)m\)")
NO_FIT = re.compile(r"\[Lateral\] Path: \(no path fit, (\d+) pts\)")


def summarise(lines):
    """Return (cycles, fits, ctes, pts). A line is a cycle only if it matches one form."""
    ctes, pts, no_fits = [], [], 0
    for line in lines:
        hit = FIT.search(line)
        if hit:
            ctes.append(abs(float(hit.group(1))))
            pts.append(int(hit.group(2)))
        elif NO_FIT.search(line):
            no_fits += 1
    return len(ctes) + no_fits, len(ctes), ctes, pts


def format_line(cycles, fits, ctes, pts):
    if cycles == 0:
        return "LANE_BASELINE_FAIL reason=no_lateral_lines"
    rate = fits / cycles
    med_pts = statistics.median(pts) if pts else 0
    med_cte = statistics.median(ctes) if ctes else float("nan")
    return (f"LANE_BASELINE cycles={cycles} fits={fits} fit_rate={rate:.3f} "
            f"median_pts={med_pts:.0f} median_abs_cte_m={med_cte:.2f}")


def main():
    if len(sys.argv) != 2:
        print("LANE_BASELINE_FAIL reason=bad_args")
        return 1
    with open(sys.argv[1], errors="replace") as f:
        out = format_line(*summarise(f))
    print(out)
    return 0 if out.startswith("LANE_BASELINE ") else 1


if __name__ == "__main__":
    sys.exit(main())
