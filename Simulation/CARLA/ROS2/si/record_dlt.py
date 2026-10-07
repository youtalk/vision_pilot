#!/usr/bin/env python3
"""Record the DLT the X5H datarouter sends to the bench, stream 6 of the recording.

  record_dlt.py <out.dlt> [--port 3490] [--seconds N]

The board's S-CORE datarouter sends DLT as unicast UDP to this host, port 3490
(openadkit platforms/autosd/x5h/config/score/log-channels.json). One datagram
carries several messages back to back, flushed every 100 ms. This writes each
message behind a DLT storage header that holds the bench clock at arrival, the
file format dlt-viewer opens. The composer in openadkit places each message by
its own board stamp and uses the arrival time only to check that placement.

dlt-viewer on the same port takes the datagrams instead of this script. Close
it before a recording, or the bind fails with port_busy.

Markers on stderr: REC_DLT ready port=<p>, then REC_DLT n=<messages>
bad=<datagrams with a cut tail>, or REC_DLT_FAIL reason=<slug> port=<p>.
"""
import argparse
import errno
import signal
import socket
import struct
import sys
import time

STORAGE = struct.Struct("<4sIi4s")


def _on_sigterm(signum, frame):
    raise SystemExit(0)


def split_messages(datagram):
    """([each DLT message], bytes left over) for one datagram.

    The length in each standard header (bytes 2-3, big endian) covers that
    message, so the messages follow one another. A length that cannot be a
    message, or one that runs past the datagram, ends the split there.
    """
    out, i = [], 0
    while len(datagram) - i >= 4:
        (n,) = struct.unpack_from(">H", datagram, i + 2)
        if n < 4 or i + n > len(datagram):
            break
        out.append(datagram[i:i + n])
        i += n
    return out, len(datagram) - i


def storage_header(now, msg):
    """The storage header for one message: the bench time, and the ECU id it carries."""
    sec = int(now)
    usec = int(round((now - sec) * 1e6))
    if usec == 1000000:
        sec, usec = sec + 1, 0
    ecu = msg[4:8] if msg[0] & 0x04 and len(msg) >= 8 else b"\0\0\0\0"
    return STORAGE.pack(b"DLT\x01", sec, usec, ecu)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("out")
    p.add_argument("--port", type=int, default=3490)
    p.add_argument("--seconds", type=float, default=600.0)
    a = p.parse_args(argv)
    signal.signal(signal.SIGTERM, _on_sigterm)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", a.port))
    except OSError as exc:
        reason = "port_busy" if exc.errno == errno.EADDRINUSE else "bind_failed"
        print(f"REC_DLT_FAIL reason={reason} port={a.port}", file=sys.stderr, flush=True)
        return 1
    sock.settimeout(0.5)
    print(f"REC_DLT ready port={a.port}", file=sys.stderr, flush=True)
    n = bad = 0
    deadline = time.monotonic() + a.seconds
    with open(a.out, "wb") as out:
        try:
            while time.monotonic() < deadline:
                try:
                    data = sock.recv(65535)
                except socket.timeout:
                    continue
                now = time.time()
                msgs, left = split_messages(data)
                for msg in msgs:
                    out.write(storage_header(now, msg) + msg)
                out.flush()
                n += len(msgs)
                bad += 1 if left else 0
        except (KeyboardInterrupt, SystemExit):
            # record-demo.sh stops the recorder with a signal. What is already
            # written is the recording, so report it instead of dying silently.
            pass
    print(f"REC_DLT n={n} bad={bad}", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
