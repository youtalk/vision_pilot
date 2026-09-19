#!/usr/bin/env python3
"""Prefix every CR52 console line with the bench clock, stream 4 of the recording.

  stamp_console.py [<capture-file>]      no argument: read standard input

/dev/x5h2-cr52 is a serial tty and a long-running tio capture already owns it. A
serial tty admits one reader, so a second open() would steal bytes out of that
capture rather than duplicate them, and the capture is the record the board work
runs on. This therefore follows the capture FILE and never opens the device.

The board has no RTC and no NTP, so nothing the firmware prints carries a time
the composer can use. The prefix is the bench clock at the moment the line
reached this host, which is the clock the other four streams are on too.

Stamped lines go to standard output and the markers to standard error: a marker
written into the console stream would read as a firmware line to the composer.

Prints STAMP ready file=<path> then STAMP n=<lines>, or STAMP_FAIL reason=<slug>.
"""
import argparse
import os
import signal
import sys
import time

from demo_streams import stamp_line


def _on_sigterm(signum, frame):
    raise SystemExit(0)


def pump(src, out, follow=False, clock=time.time, idle=0.1):
    """Stamp every complete line of src into out; return the number written.

    A line is held back until its newline arrives. The capture is being appended
    to while this reads it, so readline() regularly returns a line the firmware
    is still in the middle of writing; stamping that would split one console
    line into two, each with its own timestamp, and the second one would read as
    a firmware message that was never printed. The cost is that the stamp is the
    time the line ENDED, about 7 ms later than its first byte at 115200 baud.

    A partial line still unterminated when the stamper is stopped is dropped:
    half a line with a timestamp on it is worse evidence than no line.
    """
    n = 0
    held = ""
    try:
        while True:
            chunk = src.readline()
            if chunk:
                held += chunk
                if held.endswith("\n"):
                    out.write(stamp_line(held, clock()) + "\n")
                    out.flush()
                    held = ""
                    n += 1
                continue
            if follow:
                time.sleep(idle)
                continue
            # End of a finite input: the last line may have no newline, and at
            # EOF it is complete by definition.
            if held:
                out.write(stamp_line(held, clock()) + "\n")
                out.flush()
                n += 1
            break
    except (KeyboardInterrupt, SystemExit):
        # record-demo.sh stops the stamper with a signal. The lines already
        # written are the recording, so report them instead of dying silently.
        pass
    return n


def main():
    p = argparse.ArgumentParser()
    p.add_argument("capture", nargs="?",
                   help="tio capture file to follow; omit to read standard input")
    a = p.parse_args()
    signal.signal(signal.SIGTERM, _on_sigterm)
    if a.capture is None:
        print("STAMP ready file=<stdin>", file=sys.stderr, flush=True)
        n = pump(sys.stdin, sys.stdout)
    else:
        try:
            # errors="replace" because a serial line drops and doubles bytes:
            # one burst of noise must not kill the stamper with a decode error
            # in the middle of the run it is recording.
            src = open(a.capture, "r", errors="replace")
        except OSError:
            print("STAMP_FAIL reason=no_capture", file=sys.stderr, flush=True)
            return 1
        # Past the existing content: the capture has been running for hours and
        # its history belongs to earlier sessions, not to this recording.
        src.seek(0, os.SEEK_END)
        print(f"STAMP ready file={a.capture}", file=sys.stderr, flush=True)
        try:
            n = pump(src, sys.stdout, follow=True)
        finally:
            src.close()
    print(f"STAMP n={n}", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
