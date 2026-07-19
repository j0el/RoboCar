# Cone Runner — Autonomous Boundary Following

Robot: Adeept 4WD Omni-Directional Mecanum Wheels Robotic Car (Raspberry Pi Pico version)
Vision computer: Raspberry Pi 4 (2GB), Raspberry Pi OS Lite 64-bit, headless
Camera: 720p USB 2.0 UVC camera, 120° DFOV, mounted facing forward
Goal (Program 1): Detect orange cones and drive around the **outside** of the cone
cluster in a **counterclockwise** direction, regardless of the cluster's shape.

---

## 1. System architecture

Two-brain design. The Pi does perception and planning; the Pico does real-time
motor actuation. They talk over a USB serial cable. Both boards draw power
from the same battery pack — see §2 for how the Pi is wired in.

```
+---------------------------+          USB serial           +--------------------------+
|  Raspberry Pi 4 (2GB)     |  "V vx vy w\n" @ 20 Hz        |  Raspberry Pi Pico       |
|                           | ----------------------------> |  (Adeept expansion board)|
|  - USB camera capture     |                               |                          |
|  - HSV cone detection     |          "OK\n" ack           |  - Parses commands       |
|  - Behavior state machine | <---------------------------- |  - Mecanum wheel mixing  |
|  - Velocity commands      |                               |  - PWM to 4 DC motors    |
+---------------------------+                               |  - 0.5 s safety watchdog |
                                                            +--------------------------+
```

Why not do everything on one board?
- The Pico cannot run OpenCV or read a USB camera (no OS, 264 KB RAM).
- The Pi could drive motors directly, but that means rewiring the kit and
  losing the expansion board. Serial commands keep the kit intact.

## 2. Power distribution

The kit's single 2S Li-ion battery feeds the Adeept expansion board through
an on/off switch and a reverse-polarity diode onto a `VIN` net (see `Circuit
Schematic.pdf`). `VIN` powers the two `DRV8833` motor drivers directly and a
small onboard buck regulator that produces the board's logic-level `5V`
rail — LEDs, buzzer, IR receiver, ESP8266, and the Pico's `VSYS`, the last
via a MOSFET power-select that automatically hands the Pico off to USB power
whenever something drives its `VBUS` (e.g. the Pi, over the serial cable).

Adding the Pi 4B, three options were ruled out:
- **Pico's USB port → Pi's USB-C.** Doesn't work: the Pico's USB port is
  wired only as a power *input* (used to detect a USB host and disconnect
  the battery rail from VSYS). It never sources 5V out from the battery, and
  the Pi's USB-C port is input-only too — neither side can supply the other.
- **Battery straight into the Pi's USB-C.** Would destroy the Pi. It has no
  onboard buck for the pack's 7.4V nominal (6.0–8.4V range) — its power tree
  assumes 5V already arrives regulated.
- **Tap the onboard logic `5V` rail** (the IIC connectors `X2`–`X4`, or the
  3-pin GPIO breakouts `X5`–`X10`). All of these are fed from the same small
  buck regulator already shared by the Pico, ESP8266, and LEDs. A Pi 4B's
  draw (up to 3A peak) would overload it and brown out the Pico too.

**What we're doing instead:** a dedicated 5V/3A+ fixed-output buck converter,
wired input-side to `VIN`. `VIN` is downstream of the power switch and its
reverse-protection diode, so tapping it anywhere means the existing on/off
switch controls the Pi's power along with everything else. The bulk caps
`C21`/`C26` (next to the `DRV8833` chips) sit on this net but are small SMD
cans with little solder-friendly area; the switch itself is the easier tap —
same net, standard through-hole legs.

The switch has two terminals: one wired back to the battery ("Power") jack
(always hot whenever the battery is plugged in, regardless of switch
position), the other wired onward to `D2`/`VIN` (only hot when the switch is
on). Identify the `VIN`-side terminal before soldering anything: with the
battery connected and the switch **off**, multimeter both terminals against
GND — the one still reading battery voltage is the input side (skip it); the
one reading 0V is the `VIN` side (tap here). Solder the buck converter's
input wire to that terminal, GND to any convenient ground point.

Output side of the buck converter goes to a USB-C power-only pigtail plugged
into the Pi (VBUS/GND only — leave D+/D-/CC unconnected).

Before connecting the Pi: reconfirm the buck converter's input reads 0V with
the switch off and battery voltage with it on, then verify its output reads
~5.0–5.1V under no load before plugging in the USB-C pigtail.

## 3. Serial protocol (Pi → Pico)

One line per command, ASCII, newline-terminated:

```
V <vx> <vy> <w>\n
```

- `vx`: forward speed, -100..100 (+ = forward)
- `vy`: strafe speed,  -100..100 (+ = strafe right)
- `w` : yaw rate,      -100..100 (+ = rotate counterclockwise)
- `V 0 0 0` = stop.

Safety watchdog: if the Pico receives no valid command for 0.5 s (Pi crashed,
cable pulled), it stops all motors. The Pi therefore streams commands
continuously (~20 Hz) even when the values haven't changed.

**Telemetry (Pico → Pi):** besides `OK` acks, the firmware samples the front
HC-SR04 ultrasonic (~14 Hz) and streams each reading:

```
D <cm>\n      distance in whole cm; D -1 = no echo (nothing in range)
```

**Ultrasonic safety stop (on the Pico):** when an obstacle is closer than
15 cm, the forward component of any command is forced to zero — reverse,
strafe, and turn still work so the robot can free itself. The block releases
with hysteresis once the obstacle is beyond 20 cm. This runs in firmware so
it holds even if the Pi-side program misbehaves. The Pi additionally uses the
`D` telemetry for cone-arrival detection (see §6b).

## 4. Mecanum kinematics (on the Pico)

With rollers at 45°, wheel speeds are a linear mix of body velocities:

```
front_left  = vx + vy - w
front_right = vx - vy + w
back_left   = vx - vy - w
back_right  = vx + vy + w
```

Results are clipped to ±100 and scaled to PWM duty. Signs for "forward" on each
motor depend on wiring; the firmware has per-motor DIRECTION flags to flip
during bring-up (drive each wheel individually and correct any that spin backward).

## 5. Vision pipeline (on the Pi, per frame)

1. Capture 640×480 MJPG from the UVC camera (downscaled from 720p — plenty of
   detail for cones, and it keeps USB 2.0 at full frame rate).
2. Convert BGR → HSV.
3. Threshold for orange (defaults H 5–20, S 120–255, V 90–255 — **must be tuned
   with hsv_tuner.py under your actual lighting**).
4. Morphological open + close to remove speckle and fill holes.
5. Find contours; keep those with area above a minimum and height ≥ width
   (cones are taller than wide; this rejects most orange floor clutter).
6. For each cone, report:
   - `bearing`: horizontal center, normalized to [-1, +1] (left edge → right edge)
   - `size`: bounding-box height in pixels (proxy for distance — bigger = closer)

No neural network needed. HSV segmentation runs 25–30 fps on a Pi 4 at this
resolution, far faster than the robot moves.

## 6. Behavior: counterclockwise outside traversal

Key insight: going CCW around the *outside* of the cluster means the cones are
always on the robot's **left**. The task reduces to "wall following," where the
wall is the nearest cone on the left.

Control law while following (proportional control, tuned by two gains):

- **Yaw** to hold the tracked cone at a target bearing of about -0.45
  (left-of-center in the frame). Cone drifts right → turn left toward it;
  drifts left → turn right away.
- **Strafe** to hold the cone's apparent size at a target height (standoff
  distance). Too big (too close) → strafe right, away. Too small → strafe left,
  toward. Mecanum wheels let us do this without changing heading — a normal
  car can't.
- **Forward** at a constant base speed, reduced when corrections are large.

Cone handoff: we always track the *most relevant* cone — the largest one in the
left two-thirds of the frame. As the robot passes a cone it slides left out of
frame and shrinks; the next cone ahead naturally becomes the largest and
tracking snaps to it. The 120° lens makes this overlap generous. Because we
never assume the cluster's shape, circles, squares, and irregular layouts all
work: the robot simply keeps orange at a fixed distance on its left forever.

State machine:

```
SEARCH  --cone seen-->  FOLLOW  --no cone 0.6 s-->  LOST
  ^                                                   |
  |            (rotate CCW in place; cones             |
  +--- 3 s ----  were on the left, so turning  <-------+
                 left re-acquires them)
```

- SEARCH: rotate slowly counterclockwise in place until a cone is detected.
- FOLLOW: the control law above.
- LOST: brief pause, then back to SEARCH. Turning CCW is the right recovery
  direction since the boundary was on our left.

## 6b. Behavior: cone-to-cone visitor (Program 2, `cone_visitor.py`)

The master program. Instead of orbiting the cluster at a standoff, it visits
the cones one at a time, counterclockwise, with deliberately strafe-heavy
motion to show off the mecanum wheels. It also hosts the phone/browser UI
(§6c).

States (20 Hz loop):

- **STOPPED** — autonomy off. Manual `/drive` commands from the phone are
  honored (each expires after 0.5 s — a dead-man switch on top of the Pico's
  own watchdog).
- **SEARCH** — spin counterclockwise in place until a cone is seen (same
  sweep direction as the SLIDE orbit, so a timed-out slide continues
  naturally into a search).
- **APPROACH** — crab toward the target cone: strafe does the centering
  (`vy ∝ bearing`), yaw only gently trims heading, forward speed tapers as
  the cone's apparent height grows. The result is a diagonal mecanum slide
  rather than a car-like turn-and-drive. Frame-to-frame target association
  is by bearing window *and* similar apparent size — size continuity stops
  the huge just-visited cone from stealing the lock right after SLIDE.
  Losing the target for 0.8 s falls back to SEARCH.
- **ARRIVED** — the cone looks big enough (height ≥ 170 px) *or* the
  ultrasonic reports ≤ 25 cm. Stop and pause 1 s.
- **SLIDE** — orbit the visited cone counterclockwise: strafe right while
  yawing CCW keeps the camera locked on the cone as the robot circles it
  (cones on the left = CCW travel — program 1's insight applied to a single
  cone; the vy:w ratio sets the orbit radius). The next cone CCW around the
  cluster sweeps into frame from the left and crosses to the right; it is
  accepted once clearly right of center (bearing > +0.35, so the approach
  path swings wide of the visited cone instead of cutting through it) and
  clearly smaller than the visited cone (no re-targeting), for 3 consecutive
  frames. After 8 s without a candidate, spin CCW and relax the size filter.

Then APPROACH the new target, forever. Lap counting stays a v2 idea.
This behavior is exercised end-to-end by `sim/pi_server_sim.py`, which runs
this exact state machine in a synthetic world — the acceptance-bearing and
size-continuity rules above came out of sim runs that caught the robot
touring clockwise, re-visiting cones, and clipping the visited cone.

## 6c. WiFi access point + phone control

The Pi hosts its own WiFi AP (no router needed): `raspberry_pi/setup_ap.sh`
creates a NetworkManager hotspot — SSID **RoboCar**, WPA2 password
**robocar1**, Pi at **10.42.0.1**. While the hotspot is up, wlan0 has no
internet; use ethernet (or `nmcli connection down RoboCarAP`) to update code.

`cone_visitor.py` embeds an HTTP server on port **8080**:

| Endpoint  | Purpose                                                        |
|-----------|----------------------------------------------------------------|
| `/`       | Control page usable from any browser (video, start/stop, drive)|
| `/stream` | MJPEG video annotated with cone boxes, target, state, distance |
| `/start`  | Enable autonomy (always begins in SEARCH)                      |
| `/stop`   | Disable autonomy (robot stops; manual drive re-enabled)        |
| `/drive`  | `?vx=&vy=&w=` manual command; ignored while autonomy runs      |
| `/status` | JSON: `{running, state, cones, distance_cm}`                   |

The Android app (`android_controller/`) is a thin client for this API: it
shows the MJPEG stream, START/STOP AUTO buttons, and hold-to-drive manual
buttons that re-send `/drive` at 10 Hz while pressed. Plain platform APIs,
no third-party dependencies. Build with `./gradlew assembleDebug`. The same
HTTP API is the intended seam for the later internet-remote phase — only the
transport in front of it changes.

## 7. Files

| File                      | Runs on | Purpose                                    |
|---------------------------|---------|--------------------------------------------|
| `pico_pi/pico_motor_controller.py`      | Pico | MicroPython firmware: serial → motor PWM, ultrasonic safety stop + `D` telemetry |
| `raspberry_pi/vision.py`                | Pi 4 | Shared camera setup + HSV cone detection |
| `raspberry_pi/cone_visitor.py`          | Pi 4 | **Master program**: cone-to-cone CCW visitor + phone/browser UI (§6b, §6c) |
| `raspberry_pi/cone_follower.py`         | Pi 4 | Program 1: outside-orbit wall following  |
| `raspberry_pi/hsv_tuner.py`             | Pi 4 | Browser-based live tuning of orange range|
| `raspberry_pi/setup_ap.sh`              | Pi 4 | One-time WiFi hotspot setup (§6c)        |
| `android_controller/`                   | Phone| Android client for the §6c HTTP API      |
| `sim/pi_server_sim.py`                  | Mac  | Robot simulator: real FSM + detection + HTTP API against a synthetic cone world (tests the Android app, no hardware) |
| `sim/pico_exerciser.py`                 | Mac  | Plays the Pi over USB serial to a real Pico: keyboard drive, `D` telemetry, watchdog test |

## 8. Bring-up order (do these in sequence)

0a. **Wire Pi power (see §2)** — buck converter tapped off `VIN`, USB-C
   pigtail into the Pi — before powering on the Pi for the first time.
0b. **Pi OS setup.** On a fresh Raspberry Pi OS Lite 64-bit install, run
   `raspberry_pi/setup.sh` (installs git, python3, gh, uv; runs
   `gh auth login`). Then `gh repo clone <you>/RoboCar`, `cd RoboCar/raspberry_pi`,
   `uv sync` to install opencv-python-headless/pyserial/numpy.
1. **Pico pins.** Open Adeept's lesson code for your kit and copy the motor pin
   numbers into the `PINS` table at the top of `pico_pi/pico_motor_controller.py`.
   Flash it to the Pico (Thonny or mpremote, saved as `main.py`).
1.5. **Connect Pi ↔ Pico.** Plain USB cable, Pico's micro-USB port to any Pi
   USB-A port — no GPIO wiring, no WiFi. The firmware talks over the Pico's
   native USB CDC serial (`sys.stdin`/`print`, see `pico_motor_controller.py`),
   the same port Thonny uses. Shows up on the Pi as `/dev/ttyACM0` (check
   with `ls /dev/ttyACM*`). The Pico will then be powered from both the Pi's
   USB port and the kit's battery pack simultaneously — the Pico's onboard
   diode is designed for this, but it's worth a sanity check against the
   Adeept expansion board's power routing the first time.

   Verify the link itself before involving motors: run
   `pico_pi/pico_serial_echo_test.py` on the Pico (Thonny "Run", don't save
   as `main.py`), then from `raspberry_pi/`, `uv run python
   serial_ping_test.py`. It sends `PING` a few times and confirms `PONG`
   comes back on `/dev/ttyACM0`.
2. **Bench test motors.** Wheels off the ground. From the Pi:
   `python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200); s.write(b'V 30 0 0\n'); time.sleep(2); s.write(b'V 0 0 0\n')"`
   All four wheels should spin forward. Flip DIRECTION flags for any that don't.
   Then test strafe (`V 0 30 0`) and yaw (`V 0 0 30`).
2.5. **Ultrasonic check.** With the firmware running, hold a hand ~10 cm in
   front of the sensor: `V 30 0 0` should produce no motion (forward blocked),
   while `V -30 0 0` still reverses. Watch the `D` lines on the serial port.
3. **Tune orange.** Place cones where the robot will run. From `raspberry_pi/`,
   run `uv run python hsv_tuner.py`, open the shown URL in a browser, adjust
   sliders until cones are solid white and everything else black. Copy the
   printed values into `vision.py`.
4. **Dry run.** From `raspberry_pi/`, run `uv run python cone_visitor.py
   --dry-run` (camera + web UI, no serial) and carry the robot around by hand
   to sanity-check detections and states in the video overlay.
5. **WiFi + phone.** Run `sudo bash setup_ap.sh` once, join the phone to the
   `RoboCar` network, and open http://10.42.0.1:8080 (browser) or the Android
   app. Verify video, manual drive (autonomy stopped), then START.
6. **Low-speed live run.** Default speeds are gentle. Raise them only after
   it reliably works the full cone circuit.

## 9. Known limitations / v2 ideas

- Lighting sensitivity: HSV bounds tuned indoors will drift outdoors. Retune,
  or move to a lightweight learned detector later.
- Sharp concave layouts: if the boundary bends away more than the standoff
  can see, the robot may cut the corner. Slower speed and closer standoff help.
- No odometry: everything is reactive. Adding lap detection (e.g., recognizing
  a start marker) would let it count laps or stop after N.
- The ultrasonic only looks dead ahead: strafing (SLIDE) is unprotected by it.
  Keep the arena clear to the robot's right, or add side sensors later.
- Internet-remote control/monitoring (beyond the local AP) is a planned later
  phase; the §6c HTTP API is the seam it will build on.
