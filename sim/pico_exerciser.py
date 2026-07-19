#!/usr/bin/env python3
"""
pico_exerciser.py — play the Pi's role over USB serial to test a real Pico.

Plug the Pico (flashed with pico_motor_controller.py as main.py) into this
computer's USB port and drive it from the keyboard, exactly the way the Pi
does: a continuous 20 Hz stream of `V vx vy w` commands, while showing the
firmware's `D <cm>` ultrasonic telemetry live. Bench-test with the wheels
OFF the ground.

Keys (press repeats/extends the move; it auto-stops HOLD_S after the last press):
    w / s     forward / backward
    a / d     strafe left / right
    q / e     yaw CCW / CW
    space     stop now
    + / -     speed up / down
    t         watchdog test: sends forward once, then goes silent —
              the wheels must stop by themselves within 0.5 s
    x or Esc  quit (sends V 0 0 0)

What to verify on the bench:
  * each key moves the right wheels the right way (fix DIRECTION flags if not)
  * `D` distances track a hand moved in front of the ultrasonic
  * with a hand closer than 15 cm, `w` produces no motion (safety block)
    but `s` still reverses
  * the `t` watchdog test stops the wheels without any stop command

Needs pyserial:  python3 -m pip install pyserial
Usage:           python3 sim/pico_exerciser.py [--port /dev/tty.usbmodemXXXX]
                 (auto-detects the port if not given)
"""

import argparse
import glob
import select
import sys
import termios
import time
import tty

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  python3 -m pip install pyserial")

CMD_HZ = 20
HOLD_S = 0.45        # a keypress drives for this long; key autorepeat extends it
SAFE_CM = 15         # mirrors the firmware threshold, for the status display


def find_port():
    for pattern in ("/dev/tty.usbmodem*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    sys.exit("No Pico serial port found (looked for /dev/tty.usbmodem*, "
             "/dev/ttyACM*). Is the Pico plugged in?")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    args = ap.parse_args()
    port = args.port or find_port()

    ser = serial.Serial(port, 115200, timeout=0)
    time.sleep(1.5)  # let the Pico settle
    print(f"connected to {port} — streaming at {CMD_HZ} Hz. keys: "
          f"w/s a/d q/e space +/- t x\n")

    speed = 40
    vx = vy = w = 0
    move_until = 0.0
    distance = None
    ok_count = 0
    rxbuf = b""
    last_send = 0.0
    watchdog_test_until = 0.0

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            now = time.time()

            # ---- keyboard ----
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch in ("x", "\x1b"):
                    return
                elif ch == " ":
                    vx = vy = w = 0
                    move_until = 0
                elif ch == "+":
                    speed = min(100, speed + 10)
                elif ch == "-":
                    speed = max(10, speed - 10)
                elif ch == "t":
                    # watchdog test: one forward command, then silence
                    ser.write(b"V %d 0 0\n" % speed)
                    watchdog_test_until = now + 1.5
                    vx = vy = w = 0
                    move_until = 0
                    print("\n[watchdog test] sent one forward command, now "
                          "silent — wheels must stop on their own in 0.5 s\n")
                elif ch in "wsadqe":
                    vx, vy, w = {
                        "w": (speed, 0, 0), "s": (-speed, 0, 0),
                        "a": (0, -speed, 0), "d": (0, speed, 0),
                        "q": (0, 0, speed), "e": (0, 0, -speed),
                    }[ch]
                    move_until = now + HOLD_S

            if now > move_until:
                vx = vy = w = 0

            # ---- 20 Hz command stream (paused during the watchdog test) ----
            if now >= watchdog_test_until and now - last_send >= 1.0 / CMD_HZ:
                ser.write(b"V %d %d %d\n" % (vx, vy, w))
                last_send = now

            # ---- telemetry ----
            n = ser.in_waiting
            if n:
                rxbuf += ser.read(n)
                *lines, rxbuf = rxbuf.split(b"\n")
                for ln in lines:
                    ln = ln.strip()
                    if ln.startswith(b"D "):
                        try:
                            v = int(ln[2:])
                        except ValueError:
                            continue
                        distance = None if v < 0 else v
                    elif ln == b"OK":
                        ok_count += 1
                    elif ln:
                        print(f"\n[pico] {ln.decode(errors='replace')}")

            blocked = distance is not None and distance < SAFE_CM
            dist = " --" if distance is None else f"{distance:3d}"
            safety = "[SAFETY: fwd blocked]" if blocked else " " * 21
            wd = "  WATCHDOG TEST (silent)" if now < watchdog_test_until else ""
            sys.stdout.write(
                f"\rcmd=({vx:+4d},{vy:+4d},{w:+4d}) speed={speed:3d}  "
                f"dist={dist} cm  {safety} acks={ok_count}{wd}   ")
            sys.stdout.flush()

            time.sleep(0.01)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        try:
            ser.write(b"V 0 0 0\n")
            ser.flush()
        except Exception:
            pass
        ser.close()
        print("\nstopped (sent V 0 0 0).")


if __name__ == "__main__":
    main()
