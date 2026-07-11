#!/usr/bin/env python3
"""
movement_test.py — runs on the Raspberry Pi 4.

Drives the robot through a fixed sequence of moves to exercise every axis of
motion (forward/back, strafe, diagonals, spin). No camera/vision involved —
useful for confirming wiring, DIRECTION flags, and mecanum mixing on the Pico
before running cone_follower.py.

Usage:
    python3 movement_test.py                 # live run, loops forever
    python3 movement_test.py --dry-run       # print commands, no motors
    python3 movement_test.py --port /dev/ttyACM0 --speed 40

Requires: sudo apt install python3-serial
"""

import argparse
import time

try:
    import serial
except ImportError:
    serial = None

CMD_HZ = 20  # command rate to Pico (feeds its 0.5s watchdog)

# 360-degree spin duration is NOT calibrated — there's no encoder feedback,
# so this is a guess. Time an actual spin at SPIN_SPEED with a stopwatch and
# adjust SPIN_SECS until it comes back to its starting heading.
SPIN_SECS = 3.0


def clamp(v, lo=-100, hi=100):
    return max(lo, min(hi, v))


class PicoLink:
    def __init__(self, port, dry_run):
        self.dry = dry_run or serial is None
        self.ser = None if self.dry else serial.Serial(port, 115200, timeout=0.05)
        time.sleep(1.5)  # let the Pico settle after port open

    def send(self, vx, vy, w):
        line = f"V {int(clamp(vx))} {int(clamp(vy))} {int(clamp(w))}\n"
        if self.dry:
            return
        self.ser.write(line.encode())
        self.ser.reset_input_buffer()  # discard OK acks; protocol is fire-and-forget

    def stop(self):
        self.send(0, 0, 0)


def run_phase(link, label, vx, vy, w, secs, dry_run):
    print(f"[movement_test] {label}  (vx={vx:+d} vy={vy:+d} w={w:+d}, {secs}s)")
    period = 1.0 / CMD_HZ
    end = time.time() + secs
    while time.time() < end:
        t0 = time.time()
        link.send(vx, vy, w)
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)
    link.stop()


def build_sequence(speed, spin_speed):
    # vx: forward(+)/back(-)   vy: strafe right(+)/left(-)   w: CCW(+)/CW(-)
    return [
        ("forward",            speed,  0,     0,          2.0),
        ("backward",          -speed,  0,     0,          2.0),
        ("strafe right",       0,      speed, 0,          2.0),
        ("strafe left",        0,     -speed, 0,          2.0),
        ("diagonal right-forward",   speed,  speed, 0,    2.0),
        ("diagonal right-backward", -speed,  speed, 0,    2.0),
        ("diagonal left-forward",    speed, -speed, 0,    2.0),
        ("diagonal left-backward",  -speed, -speed, 0,    2.0),
        ("spin right (CW) 360",   0, 0, -spin_speed,  SPIN_SECS),
        ("spin left (CCW) 360",   0, 0,  spin_speed,  SPIN_SECS),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--speed", type=int, default=50, help="drive speed, -100..100")
    ap.add_argument("--spin-speed", type=int, default=50, help="yaw speed for the 360 spins")
    args = ap.parse_args()

    link = PicoLink(args.port, args.dry_run)
    sequence = build_sequence(args.speed, args.spin_speed)
    print(f"[movement_test] starting  (dry_run={args.dry_run})")

    try:
        while True:
            print("[movement_test] waiting 10s...")
            time.sleep(10)
            for label, vx, vy, w, secs in sequence:
                run_phase(link, label, vx, vy, w, secs, args.dry_run)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        print("\n[movement_test] stopped.")


if __name__ == "__main__":
    main()
