#!/usr/bin/env python3
"""
pi_server_sim.py — simulate the whole robot on a Mac/PC to test the Android app.

Runs the REAL cone_visitor state machine and the REAL cone detection against
a synthetic 2D world: orange cones on a floor, a robot with mecanum kinematics,
a simulated forward ultrasonic, and the firmware's forward-block safety rule.
Serves the exact same HTTP API as cone_visitor.py (/, /stream, /start, /stop,
/drive, /status) on port 8080.

To test the Android app: run this on the Mac, put the phone on the same WiFi
as the Mac, and enter the IP printed below into the app (instead of 10.42.0.1).
START runs the actual cone-visiting behavior in the simulated world; manual
drive works while stopped. Any browser can watch too.

Needs opencv + numpy (no pyserial, no camera, no hardware):
    python3 sim/pi_server_sim.py
"""

import argparse
import math
import os
import socket
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "raspberry_pi"))

from http.server import ThreadingHTTPServer

from cone_visitor import Visitor, Shared, make_handler, CMD_HZ, HTTP_PORT
from vision import FRAME_W, FRAME_H, HSV_LOW, HSV_HIGH, detect_cones, annotate

# ---------------- SIM TUNABLES ----------------
N_CONES = 5
CLUSTER_RADIUS = 1.2      # m, cones on a circle
ROBOT_START = (0.0, -2.8) # m, outside the cluster
CONE_HEIGHT_M = 0.30
CAMERA_HEIGHT_M = 0.10
HFOV = math.radians(100)  # horizontal FOV of the simulated camera
SPEED_MS = 0.5 / 100      # m/s per unit of vx/vy command
YAW_RADS = math.radians(90) / 100   # rad/s per unit of w command
ULTRA_BEAM = math.radians(8)        # half-angle of the simulated sonar beam
ULTRA_MAX_CM = 300
SAFE_CM = 15              # firmware forward-block, mirrored here
MIN_CONE_GAP_M = 0.15     # robot can't push into a cone
# ----------------------------------------------

FOCAL = (FRAME_W / 2) / math.tan(HFOV / 2)

# A fill color guaranteed to pass the tuned HSV range in vision.py
_mid = [(lo + hi) // 2 for lo, hi in zip(HSV_LOW, HSV_HIGH)]
CONE_BGR = tuple(int(c) for c in
                 cv2.cvtColor(np.uint8([[_mid]]), cv2.COLOR_HSV2BGR)[0, 0])
FLOOR_BGR = (60, 60, 60)
SKY_BGR = (90, 80, 70)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class World:
    """2D world: robot pose + cones. Mirrors the firmware's safety block."""

    def __init__(self):
        self.cones = [(CLUSTER_RADIUS * math.cos(2 * math.pi * i / N_CONES),
                       CLUSTER_RADIUS * math.sin(2 * math.pi * i / N_CONES))
                      for i in range(N_CONES)]
        self.x, self.y = ROBOT_START
        self.theta = math.pi / 2          # facing the cluster (+y)
        self.bumps = 0                    # times the robot contacted a cone

    # --- sensors ---

    def ultra_cm(self):
        """Nearest cone inside the sonar beam, in whole cm; None if nothing."""
        best = None
        for cx, cy in self.cones:
            d = math.hypot(cx - self.x, cy - self.y)
            rel = wrap(math.atan2(cy - self.y, cx - self.x) - self.theta)
            if abs(rel) < ULTRA_BEAM and d * 100 <= ULTRA_MAX_CM:
                best = d if best is None or d < best else best
        return None if best is None else int(best * 100)

    def render(self):
        """Draw the camera view: floor, horizon, visible cones far-to-near."""
        frame = np.full((FRAME_H, FRAME_W, 3), FLOOR_BGR, np.uint8)
        frame[:FRAME_H // 2] = SKY_BGR
        visible = []
        for cx, cy in self.cones:
            d = math.hypot(cx - self.x, cy - self.y)
            rel = wrap(math.atan2(cy - self.y, cx - self.x) - self.theta)
            if abs(rel) < HFOV / 2 and d > 0.05:
                visible.append((d, rel))
        for d, rel in sorted(visible, reverse=True):   # far first (occlusion)
            h = FOCAL * CONE_HEIGHT_M / d
            # horizontal pixel from bearing: linear in angle like vision.py's
            # bearing convention (-1..+1 across the FOV). rel + = left of
            # heading (CCW), which is the left half of the frame.
            u = FRAME_W / 2 - (rel / (HFOV / 2)) * (FRAME_W / 2)
            base_y = FRAME_H / 2 + FOCAL * CAMERA_HEIGHT_M / d
            wpx = 0.7 * h
            pts = np.array([[u - wpx / 2, base_y],
                            [u + wpx / 2, base_y],
                            [u, base_y - h]], np.int32)
            cv2.fillPoly(frame, [pts], CONE_BGR)
        return frame

    # --- physics ---

    def tick(self, vx, vy, w, dt):
        # firmware ultrasonic safety: forward blocked when something is close
        cm = self.ultra_cm()
        if cm is not None and cm < SAFE_CM and vx > 0:
            vx = 0
        f = vx * SPEED_MS
        r = vy * SPEED_MS
        nx = self.x + (f * math.cos(self.theta) + r * math.sin(self.theta)) * dt
        ny = self.y + (f * math.sin(self.theta) - r * math.cos(self.theta)) * dt
        # bump-and-slide off cones (and count it: contact = behavior bug)
        for cx, cy in self.cones:
            d = math.hypot(nx - cx, ny - cy)
            if 1e-6 < d < MIN_CONE_GAP_M:
                self.bumps += 1
                nx = cx + (nx - cx) * MIN_CONE_GAP_M / d
                ny = cy + (ny - cy) * MIN_CONE_GAP_M / d
        self.x, self.y = nx, ny
        self.theta = wrap(self.theta + w * YAW_RADS * dt)


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    args = ap.parse_args()

    world = World()
    shared = Shared()
    visitor = Visitor()

    server = ThreadingHTTPServer(("0.0.0.0", args.http_port),
                                 make_handler(shared))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[sim] serving the cone_visitor API on http://{lan_ip()}:{args.http_port}")
    print("[sim] enter that IP/port in the Android app (phone on the same WiFi),")
    print("[sim] or open it in a browser. START runs the real state machine.")

    period = 1.0 / CMD_HZ
    was_running = False
    try:
        while True:
            t0 = time.time()
            frame = world.render()
            cones = detect_cones(frame)
            dist = world.ultra_cm()
            with shared.lock:
                running = shared.running

            if running:
                if not was_running:
                    visitor.reset()
                vx, vy, w = visitor.step(cones, dist)
            else:
                vx, vy, w = shared.manual_cmd()
            was_running = running
            world.tick(vx, vy, w, period)

            state = visitor.state if running else "STOPPED"
            annotate(frame, cones, target=visitor.target if running else None,
                     lines=(
                f"SIM  {state}{'' if running else ' (manual ok)'}",
                f"cones={len(cones)} dist={'--' if dist is None else dist}cm "
                f"cmd=({vx:.0f},{vy:.0f},{w:.0f})",
                f"pose=({world.x:+.2f},{world.y:+.2f}) "
                f"th={math.degrees(world.theta):+.0f}deg",
            ))
            _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with shared.lock:
                shared.jpeg = jpg.tobytes()
                shared.state = state
                shared.cones = len(cones)
                shared.distance_cm = dist

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        print("\n[sim] stopped.")


if __name__ == "__main__":
    main()
