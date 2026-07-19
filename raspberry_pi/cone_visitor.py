#!/usr/bin/env python3
"""
cone_visitor.py — runs on the Raspberry Pi 4. The master program.

Visits the orange cones one at a time, counterclockwise around the cluster:
drive to the closest cone, stop short of it, slide sideways to the next cone
(mecanum showcase — the robot crabs and slews rather than turn-and-drive),
and repeat forever.

Also serves the phone/browser UI on port 8080 (connect to the Pi's WiFi AP,
see setup_ap.sh):
    /            control page (works in any browser)
    /stream      MJPEG video, annotated with detected cones + state
    /start /stop enable/disable autonomy
    /drive?vx=&vy=&w=   manual drive (only honored while autonomy is stopped;
                        expires after 0.5 s, so callers must repeat while held)
    /status      JSON: state, cone count, ultrasonic distance

The Pico firmware independently blocks forward motion under 15 cm and streams
`D <cm>` ultrasonic telemetry, which this program reads for cone arrival.

Usage (from raspberry_pi/, deps installed via `uv sync`):
    uv run python cone_visitor.py                 # live
    uv run python cone_visitor.py --dry-run       # no serial, camera + web only
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2

from vision import FRAME_W, FRAME_H, open_camera, detect_cones, annotate

try:
    import serial
except ImportError:
    serial = None

# ---------------- TUNABLES ----------------
CMD_HZ = 20                 # command rate to Pico (feeds its watchdog)
HTTP_PORT = 8080

# SEARCH: spin counterclockwise (same sweep direction as the SLIDE orbit,
# so after a slide times out the spin continues toward the next cone)
SEARCH_YAW = 25

# APPROACH: crab toward the target — strafe does the centering, yaw stays
# gentle so the motion reads as a mecanum slide, not a car turn.
APPROACH_BASE = 40          # forward speed, tapers off as the cone grows
APPROACH_MIN = 12
APPROACH_STRAFE_GAIN = 60   # strafe per unit bearing (centers the cone)
APPROACH_YAW_GAIN = 18      # gentle yaw per unit bearing
TRACK_BEARING_WINDOW = 0.3  # frame-to-frame target association window
LOST_TIMEOUT = 0.8          # s without the target before giving up -> SEARCH

# Arrival at a cone: it looks big enough, or the ultrasonic sees it close
ARRIVE_HEIGHT = 170         # px apparent height
ARRIVE_CM = 25              # ultrasonic distance (firmware hard-blocks at 15)
ARRIVED_PAUSE = 1.0         # s stopped at the cone

# SLIDE: orbit the visited cone counterclockwise — strafe right while
# yawing CCW keeps the camera locked on the cone as the robot circles it
# (cones on the left = CCW travel, same insight as program 1). The next
# cone CCW around the cluster sweeps into frame as the orbit progresses.
# The vy:w ratio sets the orbit radius — tune on hardware so the robot
# neither spirals into the cone nor drifts away from it.
SLIDE_STRAFE = 45
SLIDE_YAW = 28              # CCW (+), paired with the strafe to orbit
NEW_CONE_FRAC = 0.7         # next target must look this much smaller than
                            # the cone we just visited (don't re-target it)
NEW_CONE_BEARING = 0.35     # ...and this far right of it (the orbit keeps
                            # the visited cone centered; accepting the next
                            # cone too early would cut the corner straight
                            # through the visited cone)
NEW_CONE_MIN_FRAMES = 3     # consecutive sightings before committing
SLIDE_TIMEOUT = 8.0         # s of orbiting before spinning CCW to search,
                            # also relaxes the size filter

MANUAL_TIMEOUT = 0.5        # s before a manual /drive command expires
# -------------------------------------------


def clamp(v, lo=-100, hi=100):
    return max(lo, min(hi, v))


class PicoLink:
    """Serial link to the Pico: sends V commands, reads D telemetry."""

    def __init__(self, port, dry_run):
        self.dry = dry_run or serial is None
        self.ser = None if self.dry else serial.Serial(port, 115200, timeout=0)
        self.distance_cm = None   # latest ultrasonic reading; None = no echo
        self._rxbuf = b""
        time.sleep(1.5)  # let the Pico settle after port open

    def send(self, vx, vy, w):
        if self.dry:
            return
        line = f"V {int(clamp(vx))} {int(clamp(vy))} {int(clamp(w))}\n"
        self.ser.write(line.encode())
        self._drain()

    def _drain(self):
        n = self.ser.in_waiting
        if not n:
            return
        self._rxbuf += self.ser.read(n)
        *lines, self._rxbuf = self._rxbuf.split(b"\n")
        for ln in lines:
            if ln.startswith(b"D "):
                try:
                    v = int(ln[2:])
                except ValueError:
                    continue
                self.distance_cm = None if v < 0 else v
            # "OK" acks are ignored

    def stop(self):
        self.send(0, 0, 0)


class Shared:
    """State shared between the control loop and the HTTP server."""

    def __init__(self):
        self.lock = threading.Lock()
        self.running = False          # autonomy on/off (start/stop button)
        self.state = "STOPPED"
        self.jpeg = b""               # latest annotated frame
        self.cones = 0
        self.distance_cm = None
        self.manual = (0, 0, 0)
        self.manual_t = 0.0

    def manual_cmd(self):
        with self.lock:
            if time.time() - self.manual_t > MANUAL_TIMEOUT:
                return (0, 0, 0)
            return self.manual


PAGE = """<!doctype html><html><head><title>RoboCar</title>
<meta name=viewport content="width=device-width,initial-scale=1"><style>
body{font-family:sans-serif;background:#222;color:#eee;margin:12px;text-align:center}
img{width:100%;max-width:640px;border:1px solid #555}
button{font-size:17px;padding:12px 8px;margin:3px;border-radius:8px;border:0;
 background:#444;color:#eee;min-width:90px} button:active{background:#666}
#start{background:#2a6} #stop{background:#a33}
.grid{display:grid;grid-template-columns:repeat(3,1fr);max-width:400px;margin:auto}
#status{margin:8px;color:#8cf}
</style></head><body>
<img src="/stream"><div id="status">...</div>
<button id="start" onclick="fetch('/start')">START</button>
<button id="stop" onclick="fetch('/stop')">STOP</button>
<h4>Manual (autonomy stopped)</h4><div class="grid">
<button data-v="30,-30,0">&#8598;</button><button data-v="40,0,0">Fwd</button><button data-v="30,30,0">&#8599;</button>
<button data-v="0,-40,0">&#8592; Strafe</button><button data-v="0,0,0">&#9632;</button><button data-v="0,40,0">Strafe &#8594;</button>
<button data-v="0,0,35">Turn CCW</button><button data-v="-40,0,0">Back</button><button data-v="0,0,-35">Turn CW</button>
</div><script>
let iv=null;
function drive(v){fetch('/drive?vx='+v[0]+'&vy='+v[1]+'&w='+v[2])}
for(const b of document.querySelectorAll('[data-v]')){
  const v=b.dataset.v.split(',').map(Number);
  b.onpointerdown=e=>{e.preventDefault();drive(v);clearInterval(iv);iv=setInterval(()=>drive(v),150)};
  const up=()=>{clearInterval(iv);iv=null;drive([0,0,0])};
  b.onpointerup=up;b.onpointerleave=up;b.onpointercancel=up;}
setInterval(async()=>{try{const s=await(await fetch('/status')).json();
 document.getElementById('status').textContent=
  `${s.running?'RUNNING':'stopped'} | state ${s.state} | cones ${s.cones} | `+
  `dist ${s.distance_cm==null?'--':s.distance_cm+' cm'}`;}catch(e){}},500);
</script></body></html>"""


def make_handler(shared):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _ok(self, body=b"", ctype="text/plain"):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                self._ok(PAGE.encode(), "text/html")
            elif u.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with shared.lock:
                            buf = shared.jpeg
                        if buf:
                            # Content-Length lets the Android app read each
                            # frame exactly instead of scanning for markers
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\n"
                                b"Content-Length: %d\r\n\r\n" % len(buf))
                            self.wfile.write(buf + b"\r\n")
                        time.sleep(0.08)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif u.path == "/start":
                with shared.lock:
                    shared.running = True
                self._ok(b"started")
            elif u.path == "/stop":
                with shared.lock:
                    shared.running = False
                    shared.manual = (0, 0, 0)
                self._ok(b"stopped")
            elif u.path == "/drive":
                q = parse_qs(u.query)
                try:
                    cmd = tuple(int(q[k][0]) for k in ("vx", "vy", "w"))
                except (KeyError, ValueError):
                    self.send_response(400)
                    self.end_headers()
                    return
                with shared.lock:
                    shared.manual = cmd
                    shared.manual_t = time.time()
                self._ok(b"ok")
            elif u.path == "/status":
                with shared.lock:
                    body = json.dumps({
                        "running": shared.running,
                        "state": shared.state,
                        "cones": shared.cones,
                        "distance_cm": shared.distance_cm,
                    }).encode()
                self._ok(body, "application/json")
            else:
                self.send_response(404)
                self.end_headers()

    return Handler


class Visitor:
    """The cone-to-cone state machine. step() returns (vx, vy, w)."""

    def __init__(self):
        self.state = "SEARCH"
        self.target_bearing = 0.0
        self.target = None            # cone dict currently being approached
        self.last_seen = time.time()
        self.arrived_t = 0.0
        self.visited_height = ARRIVE_HEIGHT
        self.slide_t = 0.0
        self.candidate_frames = 0

    def _to(self, state):
        print(f"[state] {self.state} -> {state}")
        self.state = state
        if state != "APPROACH":
            self.target = None

    def reset(self):
        self._to("SEARCH")

    def step(self, cones, distance_cm):
        now = time.time()

        if self.state == "SEARCH":
            if cones:
                self.target_bearing = cones[0]["bearing"]
                self.last_seen = now
                self._to("APPROACH")
                return self._approach(cones[0], distance_cm, now)
            return (0, 0, SEARCH_YAW)    # spin CCW

        if self.state == "APPROACH":
            # re-associate the target: near the last known bearing AND of
            # similar apparent size. Size continuity matters right after
            # SLIDE: the huge just-visited cone can still sit inside the
            # bearing window and must not steal the lock from the small,
            # distant cone we just acquired.
            last_h = self.target["height"] if self.target else None
            near = [c for c in cones
                    if abs(c["bearing"] - self.target_bearing) < TRACK_BEARING_WINDOW
                    and (last_h is None or 0.5 * last_h <= c["height"] <= 2.0 * last_h)]
            if near:
                return self._approach(near[0], distance_cm, now)
            if now - self.last_seen > LOST_TIMEOUT:
                self._to("SEARCH")
            return (0, 0, 0)

        if self.state == "ARRIVED":
            if now - self.arrived_t > ARRIVED_PAUSE:
                self.slide_t = now
                self.candidate_frames = 0
                self._to("SLIDE")
            return (0, 0, 0)

        if self.state == "SLIDE":
            sliding = now - self.slide_t < SLIDE_TIMEOUT
            # look for the NEXT cone: it sweeps in from the left, crosses
            # center, and keeps moving right as the orbit progresses; accept
            # once it's clearly right of the (centered) visited cone, so the
            # approach path swings wide of it instead of cutting the corner.
            # It must also look clearly smaller than the cone just visited so
            # we never re-target that one (size filter relaxed after timeout)
            limit = NEW_CONE_FRAC * self.visited_height if sliding else ARRIVE_HEIGHT
            candidates = [c for c in cones
                          if c["bearing"] > NEW_CONE_BEARING and c["height"] < limit]
            if candidates:
                self.candidate_frames += 1
                if self.candidate_frames >= NEW_CONE_MIN_FRAMES:
                    self.target_bearing = candidates[0]["bearing"]
                    self.last_seen = now
                    self._to("APPROACH")
                    return self._approach(candidates[0], distance_cm, now)
            else:
                self.candidate_frames = 0
            if sliding:
                return (0, SLIDE_STRAFE, SLIDE_YAW)   # the mecanum money shot
            return (0, 0, SEARCH_YAW)                 # spin CCW, keep looking

        return (0, 0, 0)

    def _approach(self, cone, distance_cm, now):
        self.target_bearing = cone["bearing"]
        self.target = cone
        self.last_seen = now
        near_by_sonar = distance_cm is not None and distance_cm <= ARRIVE_CM
        if cone["height"] >= ARRIVE_HEIGHT or near_by_sonar:
            self.visited_height = cone["height"]
            self.arrived_t = now
            self._to("ARRIVED")
            return (0, 0, 0)
        b = cone["bearing"]
        vy = clamp(APPROACH_STRAFE_GAIN * b)          # strafe centers the cone
        w = clamp(-APPROACH_YAW_GAIN * b)             # yaw only trims heading
        vx = clamp(APPROACH_BASE * (1 - cone["height"] / ARRIVE_HEIGHT),
                   APPROACH_MIN, APPROACH_BASE)       # ease in as it grows
        return (vx, vy, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    link = PicoLink(args.port, args.dry_run)
    cap = open_camera()
    shared = Shared()
    visitor = Visitor()

    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), make_handler(shared))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[cone_visitor] web UI on http://10.42.0.1:{HTTP_PORT} "
          f"(dry_run={args.dry_run}) — press START there to run")

    period = 1.0 / CMD_HZ
    was_running = False
    try:
        while True:
            t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                link.stop()
                continue
            if frame.shape[1] != FRAME_W:
                frame = cv2.resize(frame, (FRAME_W, FRAME_H))

            cones = detect_cones(frame)
            with shared.lock:
                running = shared.running

            if running:
                if not was_running:
                    visitor.reset()       # every START begins with a search
                vx, vy, w = visitor.step(cones, link.distance_cm)
            else:
                vx, vy, w = shared.manual_cmd()
            was_running = running
            link.send(vx, vy, w)

            state = visitor.state if running else "STOPPED"
            dist = link.distance_cm
            annotate(frame, cones, target=visitor.target if running else None,
                     lines=(
                f"{state}{'' if running else ' (manual ok)'}",
                f"cones={len(cones)} dist={'--' if dist is None else dist}cm "
                f"cmd=({vx:.0f},{vy:.0f},{w:.0f})",
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
        link.stop()
        cap.release()
        server.shutdown()
        print("\n[cone_visitor] stopped.")


if __name__ == "__main__":
    main()
