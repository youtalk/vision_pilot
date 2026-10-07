"""record_dlt.py, stream 6 of the demo recording. Run: python3 -m pytest -q test_record_dlt.py

The datagrams are built here in the layout the S-CORE datarouter sends
(score_logging 0.2.4: several messages back to back, the standard header length
big endian). The end-to-end tests run the script on a free local port.
"""
import pathlib
import socket
import struct
import subprocess
import sys
import time

import record_dlt as r

HERE = pathlib.Path(__file__).resolve().parent


def message(text, ecu=b"X5H\0"):
    raw = text.encode() + b"\0"
    payload = struct.pack("<IH", 0x200, len(raw)) + raw
    ext = struct.pack("<BB4s4s", 0x41, 1, b"VP\0\0", b"VP\0\0")
    n = 4 + 4 + 4 + len(ext) + len(payload)
    return struct.pack(">BBH", 0x35, 0, n) + ecu + struct.pack(">I", 12345678) + ext + payload


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_split_takes_every_message_of_a_datagram():
    a, b = message("one"), message("two")
    assert r.split_messages(a + b) == ([a, b], 0)


def test_split_keeps_the_complete_messages_of_a_cut_datagram():
    a, b = message("one"), message("two")
    assert r.split_messages(a + b[:-2]) == ([a], len(b) - 2)


def test_split_stops_at_a_length_that_cannot_be_a_message():
    a = message("one")
    assert r.split_messages(a + b"\x35\x00\x00\x02junk") == ([a], 8)


def test_storage_header_holds_the_bench_time_and_the_ecu():
    h = r.storage_header(1789432101.25, message("x"))
    assert h == struct.pack("<4sIi4s", b"DLT\x01", 1789432101, 250000, b"X5H\0")


def test_storage_header_without_an_ecu_id_leaves_it_empty():
    msg = bytearray(message("x"))
    msg[0] &= ~0x04
    assert r.storage_header(1.0, bytes(msg))[12:] == b"\0\0\0\0"


def test_records_what_arrives_until_stopped(tmp_path):
    port = free_port()
    out = tmp_path / "dlt.dlt"
    proc = subprocess.Popen([sys.executable, str(HERE / "record_dlt.py"), str(out),
                             "--port", str(port), "--seconds", "20"],
                            stderr=subprocess.PIPE, text=True)
    a, b = message("one"), message("two")
    want = 32 + len(a) + len(b)
    try:
        assert proc.stderr.readline().startswith(f"REC_DLT ready port={port}")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(a + b, ("127.0.0.1", port))
        for _ in range(50):
            if out.exists() and out.stat().st_size >= want:
                break
            time.sleep(0.1)
        proc.terminate()
        _, err = proc.communicate(timeout=5)
    finally:
        proc.kill()
    assert "REC_DLT n=2 bad=0" in err
    data = out.read_bytes()
    assert len(data) == want
    assert data[:4] == b"DLT\x01"
    assert data[16:16 + len(a)] == a
    assert data[32 + len(a):] == b


def test_refuses_a_port_another_listener_holds(tmp_path):
    # dlt-viewer left open on the bench holds 3490, and every datagram would go
    # to it instead. The recording must stop at once and say so.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("0.0.0.0", 0))
        port = s.getsockname()[1]
        res = subprocess.run([sys.executable, str(HERE / "record_dlt.py"), str(tmp_path / "x.dlt"),
                              "--port", str(port), "--seconds", "1"],
                             capture_output=True, text=True, timeout=10)
    assert res.returncode == 1
    assert f"REC_DLT_FAIL reason=port_busy port={port}" in res.stderr
