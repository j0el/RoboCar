# Cone Runner — Autonomous Boundary Following

Robot: Adeept 4WD Omni-Directional Mecanum Wheels Robotic Car (Raspberry Pi Pico version)
Vision computer: Raspberry Pi 4 (2GB), Raspberry Pi OS Lite 64-bit, headless
Camera: 720p USB 2.0 UVC camera, 120° DFOV, mounted facing forward
Goal (Program 1): Detect orange cones and drive around the **outside** of the cone
cluster in a **counterclockwise** direction, regardless of the cluster's shape.

---

## 1. System architecture

Two-brain design. The Pi does perception and planning; the Pico does real-time
motor actuation. They talk over a USB serial cable (the Pico is powered by the
kit's battery pack; the Pi has its own supply).

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

## 2. Serial protocol (Pi → Pico)

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

## 3. Mecanum kinematics (on the Pico)

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

## 4. Vision pipeline (on the Pi, per frame)

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

## 5. Behavior: counterclockwise outside traversal

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

## 6. Files

| File                      | Runs on | Purpose                                    |
|---------------------------|---------|--------------------------------------------|
| `pico_pi/pico_motor_controller.py`| Pico | MicroPython firmware: serial → motor PWM |
| `cone_follower.py`        | Pi 4    | Vision + state machine + serial commands   |
| `hsv_tuner.py`            | Pi 4    | Browser-based live tuning of orange range  |

## 7. Bring-up order (do these in sequence)

0. **Pi OS setup.** On a fresh Raspberry Pi OS Lite 64-bit install, run
   `raspberry_pi/setup.sh` (installs git, python3, gh, uv; runs
   `gh auth login`). Then `gh repo clone <you>/RoboCar`, `cd RoboCar`,
   `uv sync` to install opencv-python/pyserial/numpy.
1. **Pico pins.** Open Adeept's lesson code for your kit and copy the motor pin
   numbers into the `PINS` table at the top of `pico_pi/pico_motor_controller.py`.
   Flash it to the Pico (Thonny or mpremote, saved as `main.py`).
2. **Bench test motors.** Wheels off the ground. From the Pi:
   `python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200); s.write(b'V 30 0 0\n'); time.sleep(2); s.write(b'V 0 0 0\n')"`
   All four wheels should spin forward. Flip DIRECTION flags for any that don't.
   Then test strafe (`V 0 30 0`) and yaw (`V 0 0 30`).
3. **Tune orange.** Place cones where the robot will run. Run `hsv_tuner.py`,
   open the shown URL in a browser, adjust sliders until cones are solid white
   and everything else black. Copy the printed values into `cone_follower.py`.
4. **Dry run.** Run `cone_follower.py --dry-run` (prints decisions, no motion)
   and carry the robot around by hand to sanity-check detections and states.
5. **Low-speed live run.** Default speeds are gentle. Widen the standoff and
   raise speed only after it reliably makes full laps.

## 8. Known limitations / v2 ideas

- Lighting sensitivity: HSV bounds tuned indoors will drift outdoors. Retune,
  or move to a lightweight learned detector later.
- Sharp concave layouts: if the boundary bends away more than the standoff
  can see, the robot may cut the corner. Slower speed and closer standoff help.
- No odometry: everything is reactive. Adding lap detection (e.g., recognizing
  a start marker) would let it count laps or stop after N.
- The kit's ultrasonic sensor is unused so far — a natural v2 is collision
  backstop (halt if anything closer than 15 cm dead ahead).
