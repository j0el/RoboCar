#!/usr/bin/env python3
"""
cone_follower.py — runs on the Raspberry Pi 4.

Detects orange cones with the USB camera and drives counterclockwise around
the OUTSIDE of the cone cluster (cones stay on the robot's left).

Usage (from raspberry_pi/, deps installed via `uv sync`):
    uv run python cone_follower.py                 # live run
    uv run python cone_follower.py --dry-run       # print decisions, no motors
    uv run python cone_follower.py --port /dev/ttyACM0
"""

import argparse
import time
import cv2

from vision import FRAME_W, FRAME_H, open_camera, detect_cones

try:
    import serial
except ImportError:
    serial = None

# ---------------- TUNABLES ----------------
# Camera device and HSV bounds live in vision.py (shared with the other
# Pi programs); tune with hsv_tuner.py.
TARGET_BEARING = -0.45     # keep tracked cone left-of-center (-1..+1)
TARGET_HEIGHT = 90         # px, apparent cone height = standoff distance
BASE_FORWARD = 35          # cruise speed, -100..100
YAW_GAIN = 55.0            # yaw per unit bearing error
STRAFE_GAIN = 0.55         # strafe per pixel of height error
SEARCH_YAW = 25            # CCW rotate speed while searching
LOST_TIMEOUT = 0.6         # s without a cone before LOST
CMD_HZ = 20                # command rate to Pico (feeds its watchdog)
# -------------------------------------------


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


def pick_target(cones):
    """Track the largest cone in the left 2/3 of the frame; else largest anywhere.
    Passing a cone slides it out the left and shrinks it, so tracking naturally
    hands off to the next cone appearing ahead."""
    left_side = [c for c in cones if c["bearing"] < 0.33]
    if left_side:
        return left_side[0]
    return cones[0] if cones else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    link = PicoLink(args.port, args.dry_run)
    cap = open_camera()

    state = "SEARCH"
    last_seen = 0.0
    period = 1.0 / CMD_HZ
    print(f"[cone_follower] starting in {state}  (dry_run={args.dry_run})")

    try:
        while True:
            t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                link.stop()
                continue
            if frame.shape[1] != FRAME_W:
                frame = cv2.resize(frame, (FRAME_W, FRAME_H))

            target = pick_target(detect_cones(frame))
            now = time.time()
            if target:
                last_seen = now

            # ---- state transitions ----
            if state == "SEARCH" and target:
                state = "FOLLOW"
                print("[state] SEARCH -> FOLLOW")
            elif state == "FOLLOW" and now - last_seen > LOST_TIMEOUT:
                state = "LOST"
                print("[state] FOLLOW -> LOST")
            elif state == "LOST":
                state = "SEARCH"   # boundary was on our left; CCW search re-finds it
                print("[state] LOST -> SEARCH")

            # ---- state actions ----
            if state == "FOLLOW" and target:
                bearing_err = target["bearing"] - TARGET_BEARING   # + means cone too far right
                height_err = target["height"] - TARGET_HEIGHT      # + means too close
                w = clamp(-YAW_GAIN * bearing_err)                 # steer toward target bearing
                vy = clamp(STRAFE_GAIN * height_err)               # too close -> strafe right (+)
                # slow down when corrections are big
                vx = clamp(BASE_FORWARD * (1 - min(1, abs(bearing_err))), 10, 100)
                link.send(vx, vy, w)
                if args.dry_run:
                    print(f"FOLLOW b={target['bearing']:+.2f} h={target['height']:3d} "
                          f"-> vx={vx:3.0f} vy={vy:+4.0f} w={w:+4.0f}")
            elif state == "SEARCH":
                link.send(0, 0, SEARCH_YAW)   # rotate CCW in place
            else:
                link.stop()

            # hold the command rate steady (also feeds Pico watchdog)
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        cap.release()
        print("\n[cone_follower] stopped.")


if __name__ == "__main__":
    main()
