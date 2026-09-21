#!/usr/bin/env python3
"""The composer-facing contract of one demo recording: index files and manifest.

  demo_streams.py --run-id <id> --mode kill --fault-at <epoch-seconds>

Prints manifest.json on stdout; record-demo.sh redirects it into the run
directory.

The demo video is composed by a separate script in openadkit out of five
streams: two cameras and the CR52 console recorded on this bench host, and
VisionPilot's HUD frames and journal pulled off the X5H board. The board has no
RTC and no NTP, so the five streams share no clock and the composer aligns them
all on the fault instant instead. Everything it needs to do that is the
per-stream index.csv written here and the manifest below.

Both are a hard interface: a renamed column or key does not degrade the video,
it makes the composer refuse to compose. That is why they are formatted in one
module with tests rather than inline by each recorder and by the shell driver.

Every ValueError raised here carries a marker slug as its message, so a caller
can put it straight into its own <NAME>_FAIL reason=<slug> line.
"""
import argparse
import json
import sys

INDEX_HEADER = "file,bench_time"

# The routes run-d6.sh drives. A typo here would mislabel the reel rather than
# fail it, and the label is what tells two otherwise identical videos apart.
MODES = ("kill", "channel", "standin")

# Epoch seconds on this bench are about 1.79e9. Anything below this is a
# relative time that leaked in where an absolute one belongs (a DRIVE_S, a
# t_rel_s out of trace.csv), which would slide every pane by decades.
MIN_EPOCH_S = 1.0e9


def open_index(path):
    """Create an index.csv and write its header.

    Line buffered on purpose: record-demo.sh stops the recorders with a signal,
    and a block-buffered index would lose its last frames while the JPEGs they
    name are already on disk, so the pane would end before the fault it exists
    to show.
    """
    f = open(path, "w", buffering=1)
    f.write(INDEX_HEADER + "\n")
    return f


def write_row(f, name, bench_time):
    """One frame: its file name and its bench-clock time in seconds.

    Millisecond resolution is finer than either camera's frame interval, so it
    is enough to place a pane, and it is what the composer parses.
    """
    if "," in name or "\n" in name:
        raise ValueError("bad_frame_name")
    f.write(f"{name},{bench_time:.3f}\n")


def stamp_seconds(sec, nanosec):
    """A ROS header stamp as bench epoch seconds.

    Zero is not a stamp. A republish that forgets the header, or a node running
    on sim time, leaves it at 0 or at a few thousand seconds; the pane then
    aligns to 1970 and the composer drops it without a word. Refuse it here,
    where the caller still knows which stream it was.
    """
    t = sec + nanosec * 1e-9
    if t < MIN_EPOCH_S:
        raise ValueError("bad_stamp")
    return t


def build_manifest(run_id, mode, fault_at):
    """The run's self-description. All five streams are always named.

    hud/ is filled later by the board-side pull script, and console/trace are
    single files, but the composer looks up all five keys unconditionally: an
    absent key is a KeyError there, while an empty directory is a missing pane
    it can report.
    """
    if not run_id:
        raise ValueError("bad_run_id")
    if mode not in MODES:
        raise ValueError("bad_mode")
    try:
        fault_at = float(fault_at)
    except (TypeError, ValueError):
        raise ValueError("bad_fault_at") from None
    if fault_at < MIN_EPOCH_S:
        raise ValueError("bad_fault_at")
    # Every path is relative to the run directory, so the whole directory can be
    # copied to whichever machine composes the video.
    return {
        "run_id": run_id,
        "mode": mode,
        "fault_at": fault_at,
        "streams": {
            "chase": {"dir": "chase", "index": "chase/index.csv"},
            "camera": {"dir": "camera", "index": "camera/index.csv"},
            "hud": {"dir": "hud", "journal": "hud/vp-journal.txt"},
            "console": {"file": "cr52-console.txt"},
            "trace": {"file": "trace.csv"},
        },
    }


def stamp_line(line, now):
    """One CR52 console line with the bench clock in front of it.

    The console is 115200 8N1 and ends its lines with CR LF. Left in place the
    carriage return lands one column after the timestamp it is supposed to
    follow and pushes the rest of the line over it in every viewer. Only the
    line ending is stripped: firmware indentation is evidence.
    """
    # Kept out of the f-string: a backslash inside one is a syntax error before
    # Python 3.12, and this module is imported both in the host carla venv and
    # in visionpilot:si.
    text = line.rstrip("\r\n")
    return f"{now:.3f} {text}"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-id", required=True)
    p.add_argument("--mode", required=True)
    p.add_argument("--fault-at", type=float, required=True)
    a = p.parse_args()
    try:
        m = build_manifest(a.run_id, a.mode, a.fault_at)
    except ValueError as exc:
        # On stderr: stdout is the manifest, and a marker inside it would leave
        # a file that no longer parses as JSON rather than no file at all.
        print(f"MANIFEST_FAIL reason={exc}", file=sys.stderr, flush=True)
        return 1
    print(json.dumps(m, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
