#!/usr/bin/env python3
"""
serial_ping_test.py — communication-only test for the Pi <-> Pico USB serial
link, no motors involved.

Flash pico_pi/pico_serial_echo_test.py to the Pico first (Thonny "Run", or
`mpremote run` -- no need to save it as main.py). Then, from raspberry_pi/:

    uv run python serial_ping_test.py
    uv run python serial_ping_test.py --port /dev/ttyACM0 --count 10
"""

import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=5)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(2.0)  # let the Pico's USB CDC settle after port open
    ser.reset_input_buffer()

    ok = 0
    for _ in range(args.count):
        ser.write(b"PING\n")
        reply = ser.readline().decode(errors="replace").strip()
        print(f"sent PING -> got {reply!r}")
        if reply == "PONG":
            ok += 1
        time.sleep(0.3)

    print(f"\n{ok}/{args.count} PINGs answered correctly")
    if ok < args.count:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
