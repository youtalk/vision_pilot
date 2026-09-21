"""The composer interface of the demo recording. Run: python3 -m pytest test_demo_streams.py

Everything here is the part that is wrong silently: the composer aligns five
streams on times it reads out of index.csv and manifest.json, and a renamed
column, a lost millisecond or a relative fault_at produces a video that is
merely out of sync rather than a run that fails.

demo_cam.py and record_camera.py are not tested: they import carla and rclpy,
which exist on the bench and in visionpilot:si only. Mocking a simulator would
test the mock, which is why lap_gate.py, snap.py and the other clients in this
directory have no tests either. Their pure parts live in demo_streams.py and
are tested below.
"""
import io
import json

import demo_streams as d
import stamp_console

NOW = 1789432101.4823


# --- index.csv -----------------------------------------------------------
def test_index_header_is_exactly_what_the_composer_expects():
    assert d.INDEX_HEADER == "file,bench_time"


def test_open_index_writes_the_header_and_nothing_else(tmp_path):
    p = tmp_path / "index.csv"
    d.open_index(str(p)).close()
    assert p.read_text() == "file,bench_time\n"


def test_write_row_keeps_millisecond_resolution(tmp_path):
    p = tmp_path / "index.csv"
    f = d.open_index(str(p))
    d.write_row(f, "000000.jpg", NOW)
    d.write_row(f, "000001.jpg", NOW + 0.05)
    f.close()
    assert p.read_text().splitlines() == [
        "file,bench_time",
        "000000.jpg,1789432101.482",
        "000001.jpg,1789432101.532",
    ]


def test_write_row_refuses_a_name_that_would_break_the_csv(tmp_path):
    f = d.open_index(str(tmp_path / "index.csv"))
    for bad in ("a,b.jpg", "a\nb.jpg"):
        try:
            d.write_row(f, bad, NOW)
            raise AssertionError(f"accepted {bad!r}")
        except ValueError as exc:
            assert str(exc) == "bad_frame_name"
    f.close()


def test_index_is_readable_while_the_recorder_is_still_running(tmp_path):
    # Line buffered: the recorders are stopped with a signal, so an index that
    # was never flushed would name fewer frames than are on disk.
    p = tmp_path / "index.csv"
    f = d.open_index(str(p))
    d.write_row(f, "000000.jpg", NOW)
    assert p.read_text().endswith("000000.jpg,1789432101.482\n")
    f.close()


# --- header stamps -------------------------------------------------------
def test_stamp_seconds_joins_the_two_header_fields():
    assert abs(d.stamp_seconds(1789432101, 482300000) - 1789432101.4823) < 1e-6


def test_stamp_seconds_refuses_a_stamp_that_is_not_a_bench_time():
    for sec, nanosec in ((0, 0), (0, 1), (1234, 0)):
        try:
            d.stamp_seconds(sec, nanosec)
            raise AssertionError(f"accepted {sec}.{nanosec}")
        except ValueError as exc:
            assert str(exc) == "bad_stamp"


# --- console stamping ----------------------------------------------------
def test_stamp_line_prefixes_the_bench_clock_with_millisecond_resolution():
    assert d.stamp_line("[SI] heartbeat stale, override engaged\r\n", NOW) \
        == "1789432101.482 [SI] heartbeat stale, override engaged"


def test_stamp_line_keeps_firmware_indentation():
    assert d.stamp_line("    tick 0x00\r\n", NOW) == "1789432101.482     tick 0x00"


def test_stamp_line_timestamp_field_is_a_float_with_three_decimals():
    field = d.stamp_line("x\r\n", NOW).split(" ")[0]
    assert len(field.split(".")[1]) == 3
    assert abs(float(field) - NOW) < 0.001


def test_pump_stamps_every_line_and_counts_them():
    out = io.StringIO()
    n = stamp_console.pump(io.StringIO("one\r\ntwo\r\n"), out, clock=lambda: NOW)
    assert n == 2
    assert out.getvalue() == f"{NOW:.3f} one\n{NOW:.3f} two\n"


def test_pump_stamps_each_line_at_its_own_arrival_time():
    out = io.StringIO()
    ticks = iter([1000000000.25, 1000000001.5])
    stamp_console.pump(io.StringIO("a\n b\n"), out, clock=lambda: next(ticks))
    assert out.getvalue() == "1000000000.250 a\n1000000001.500  b\n"


def test_pump_stamps_a_final_line_that_has_no_newline():
    out = io.StringIO()
    n = stamp_console.pump(io.StringIO("done\r\ncut off"), out, clock=lambda: NOW)
    assert n == 2
    assert out.getvalue().splitlines()[1] == f"{NOW:.3f} cut off"


def test_pump_never_splits_one_console_line_in_two():
    # What readline() returns while tio is mid-line, including mid-token.
    # Stamping the first half would invent a firmware message never printed.
    class PartialReads:
        def __init__(self, parts):
            self.parts = list(parts)

        def readline(self):
            return self.parts.pop(0) if self.parts else ""

    out = io.StringIO()
    n = stamp_console.pump(PartialReads(["[SI] fault latched 0x0", "1", " cycle 7\r\n"]),
                           out, clock=lambda: NOW)
    assert n == 1
    assert out.getvalue() == f"{NOW:.3f} [SI] fault latched 0x01 cycle 7\n"


# --- manifest.json -------------------------------------------------------
FAULT_AT = 1789432100.123


def test_manifest_has_exactly_the_keys_the_composer_looks_up():
    m = d.build_manifest("20260918-134501", "kill", FAULT_AT)
    assert set(m) == {"run_id", "mode", "fault_at", "streams"}
    assert set(m["streams"]) == {"chase", "camera", "hud", "console", "trace"}
    assert m["streams"]["chase"] == {"dir": "chase", "index": "chase/index.csv"}
    assert m["streams"]["camera"] == {"dir": "camera", "index": "camera/index.csv"}
    assert m["streams"]["hud"] == {"dir": "hud", "journal": "hud/vp-journal.txt"}
    assert m["streams"]["console"] == {"file": "cr52-console.txt"}
    assert m["streams"]["trace"] == {"file": "trace.csv"}


def test_manifest_keeps_the_fault_instant_and_the_run_identity():
    m = d.build_manifest("20260918-134501", "kill", FAULT_AT)
    assert m["run_id"] == "20260918-134501"
    assert m["mode"] == "kill"
    assert m["fault_at"] == FAULT_AT
    # It travels as JSON, and json.dumps must not round the fault instant.
    assert json.loads(json.dumps(m))["fault_at"] == FAULT_AT


def test_manifest_paths_are_all_relative_to_the_run_directory():
    m = d.build_manifest("20260918-134501", "kill", FAULT_AT)
    for stream in m["streams"].values():
        for path in stream.values():
            assert not path.startswith("/")


def test_manifest_refuses_a_mislabelled_run():
    for run_id, mode, fault_at, slug in (
        ("", "kill", FAULT_AT, "bad_run_id"),
        ("20260918-134501", "Kill", FAULT_AT, "bad_mode"),
        ("20260918-134501", "killed", FAULT_AT, "bad_mode"),
        # A relative time where an epoch belongs: a DRIVE_S, or a t_rel_s out of
        # trace.csv. Composed with it, every pane is decades out.
        ("20260918-134501", "kill", 40.0, "bad_fault_at"),
        ("20260918-134501", "kill", 0.0, "bad_fault_at"),
        ("20260918-134501", "kill", "", "bad_fault_at"),
    ):
        try:
            d.build_manifest(run_id, mode, fault_at)
            raise AssertionError(f"accepted {run_id!r} {mode!r} {fault_at!r}")
        except ValueError as exc:
            assert str(exc) == slug
