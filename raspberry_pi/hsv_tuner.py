#!/usr/bin/env python3
"""
hsv_tuner.py — tune the orange HSV range from any browser (Pi stays headless).

Run on the Pi:   python3 hsv_tuner.py
Then open:       http://<pi-ip>:8000
Left image = camera, right = mask. Adjust sliders until the cones are solid
white and everything else is black. Current values print in the page and the
terminal — copy them into HSV_LOW / HSV_HIGH in cone_follower.py.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

W, H = 640, 480
vals = {"hl": 5, "sl": 120, "vl": 90, "hh": 20, "sh": 255, "vh": 255}
lock = threading.Lock()
latest = {"cam": b"", "mask": b""}

PAGE = """<!doctype html><html><head><title>HSV Tuner</title><style>
body{font-family:sans-serif;background:#222;color:#eee;margin:16px}
img{width:480px;border:1px solid #555} .row{display:flex;gap:12px;flex-wrap:wrap}
label{display:inline-block;width:110px} input{width:300px;vertical-align:middle}
code{background:#333;padding:4px 8px;border-radius:4px;font-size:15px}
</style></head><body><h2>Orange cone HSV tuner</h2>
<div class="row"><div><h4>Camera</h4><img src="/cam"></div>
<div><h4>Mask (cones should be solid white)</h4><img src="/mask"></div></div>
<div id="sliders"></div><p>Copy into cone_follower.py:</p><p><code id="out"></code></p>
<script>
const defs=[["hl","H low",0,179],["hh","H high",0,179],["sl","S low",0,255],
["sh","S high",0,255],["vl","V low",0,255],["vh","V high",0,255]];
const v={{VALS}};
const div=document.getElementById("sliders");
for(const [k,name,lo,hi] of defs){
  div.insertAdjacentHTML("beforeend",
   `<div><label>${name}: <b id="${k}v">${v[k]}</b></label>
    <input type=range id="${k}" min=${lo} max=${hi} value=${v[k]}></div>`);
  document.getElementById(k).oninput=e=>{v[k]=+e.target.value;
    document.getElementById(k+"v").textContent=v[k];push();};}
function show(){document.getElementById("out").textContent=
 `HSV_LOW = (${v.hl}, ${v.sl}, ${v.vl})   HSV_HIGH = (${v.hh}, ${v.sh}, ${v.vh})`;}
let t=null;function push(){show();clearTimeout(t);
 t=setTimeout(()=>fetch(`/set?${new URLSearchParams(v)}`),80);}
show();
</script></body></html>"""


def capture_loop():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        with lock:
            lo = np.array([vals["hl"], vals["sl"], vals["vl"]])
            hi = np.array([vals["hh"], vals["sh"], vals["vh"]])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lo, hi)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        _, jc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        _, jm = cv2.imencode(".jpg", mask, [cv2.IMWRITE_JPEG_QUALITY, 70])
        latest["cam"], latest["mask"] = jc.tobytes(), jm.tobytes()
        time.sleep(0.05)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def stream(self, key):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                buf = latest[key]
                if buf:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(buf + b"\r\n")
                time.sleep(0.08)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            body = PAGE.replace("{{VALS}}", str(vals)).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/cam":
            self.stream("cam")
        elif u.path == "/mask":
            self.stream("mask")
        elif u.path == "/set":
            q = parse_qs(u.query)
            with lock:
                for k in vals:
                    if k in q:
                        vals[k] = int(q[k][0])
            print(f"HSV_LOW = ({vals['hl']}, {vals['sl']}, {vals['vl']})   "
                  f"HSV_HIGH = ({vals['hh']}, {vals['sh']}, {vals['vh']})",
                  end="\r", flush=True)
            self.send_response(204)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


if __name__ == "__main__":
    threading.Thread(target=capture_loop, daemon=True).start()
    print("HSV tuner running — open http://<pi-ip>:8000 in a browser. Ctrl+C to quit.")
    ThreadingHTTPServer(("0.0.0.0", 8000), Handler).serve_forever()
