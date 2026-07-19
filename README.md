# RoboCar

Autonomous behaviors for the **Adeept 4WD Omni-Directional Mecanum Wheels Robotic Car**
(Raspberry Pi Pico version), with vision and planning on a **Raspberry Pi 4** and a
forward-facing 720p USB camera (120° DFOV).

## Program 1: Cone Runner

Detect orange traffic cones with the camera and drive **counterclockwise around the
outside** of the cone cluster — circle, square, or any irregular layout. The approach
is reactive wall-following: keep the nearest cone on the robot's left at a fixed
apparent distance, and the boundary shape takes care of itself.

## Program 2: Cone Visitor (master program)

Drive to the closest cone, stop short of it (vision + ultrasonic), then slide
sideways to the next cone counterclockwise — deliberately strafe-heavy motion
to show off the mecanum wheels. Start/stop and live annotated video from a
phone: the Pi hosts its own WiFi AP (SSID `RoboCar`) and `cone_visitor.py`
serves the control page and MJPEG stream at `http://10.42.0.1:8080`, which the
Android app (or any browser) connects to. The Pico firmware independently
blocks forward motion when the ultrasonic sees anything closer than 15 cm.

Full design details, serial protocol, control laws, and bring-up checklist are in
[ARCHITECTURE.md](ARCHITECTURE.md).

## Hardware

| Part | Role |
|------|------|
| Adeept 4WD Mecanum kit (Raspberry Pi Pico + expansion board) | Chassis, motors, real-time motor control |
| Raspberry Pi 4 (2GB), Raspberry Pi OS Lite 64-bit | Vision + behavior ("the brain") |
| 720p USB 2.0 UVC camera, 120° DFOV | Forward-facing cone detection |
| USB cable, Pi ↔ Pico | Serial command link (`V vx vy w` at 20 Hz) |

## Repository contents

- `ARCHITECTURE.md` — system design, protocol, vision pipeline, behavior state machines
- `pico_pi/` — everything that runs on the Pico. `pico_motor_controller.py` is
  the MicroPython firmware (flash as `main.py`); parses velocity commands,
  mecanum mixing, PWM output, 0.5 s safety watchdog, ultrasonic safety stop
  with `D <cm>` telemetry. Also holds bench-test and legacy ESP01 scripts.
- `raspberry_pi/` — everything that runs on the Pi. `cone_visitor.py` is the
  master program (cone-to-cone visitor + phone web UI); `cone_follower.py`
  is Program 1 (outside-orbit wall following); `vision.py` holds the shared
  camera/HSV detection code. `hsv_tuner.py` is browser-based live tuning
  of the orange HSV range (the Pi stays headless; open `http://<pi-ip>:8000`).
  `setup.sh` provisions a fresh Pi OS install; `setup_ap.sh` creates the WiFi
  hotspot; `pyproject.toml` declares the Python deps, installed with `uv sync`.
- `android_controller/` — Android app: annotated live video, START/STOP
  autonomy, and hold-to-drive manual control over the Pi's AP
  (`./gradlew assembleDebug` to build).
- `sim/` — no-hardware test harnesses that stand in for the Pi:
  - `pi_server_sim.py` — full robot simulator on the Mac: synthetic cone
    world + the real cone_visitor state machine, detection, and HTTP API.
    Run it, point the Android app (or a browser) at the printed Mac IP, and
    test start/stop/video/manual drive with no robot at all.
  - `pico_exerciser.py` — keyboard drive of a real Pico over USB from the
    Mac: streams `V` commands at 20 Hz like the Pi does, shows `D` ultrasonic
    telemetry, and has a one-key watchdog test. Needs pyserial
    (`uv run --with pyserial python3 sim/pico_exerciser.py`).

## Quick start

On the Pi (after `raspberry_pi/setup.sh`, see `raspberry_pi/README.md`):

```bash
cd raspberry_pi
uv sync
```

1. Flash `pico_pi/pico_motor_controller.py` to the Pico as `main.py`
   (pins are already confirmed against the Adeept lesson code).
2. Bench-test with wheels off the ground; flip any `DIRECTION` flag for a
   wheel spinning the wrong way. Hold a hand in front of the ultrasonic:
   forward commands should be blocked under 15 cm.
3. From `raspberry_pi/`, run `uv run python hsv_tuner.py`, tune until cones
   are solid white in the mask, and copy the values into `HSV_LOW` /
   `HSV_HIGH` in `vision.py`.
4. `uv run python cone_visitor.py --dry-run` to verify detection in the web
   UI video, then `sudo bash setup_ap.sh` (once), join the phone to the
   `RoboCar` WiFi (password `robocar1`), open `http://10.42.0.1:8080` or the
   Android app, and press START.

## Status

- [x] Architecture and v1 code
- [x] Pin mapping confirmed against Adeept lesson code
- [x] Pi ↔ Pico USB serial link verified
- [x] Ultrasonic safety stop in firmware
- [x] Cone visitor master program + phone web UI (code complete)
- [x] Android app: video + start/stop + manual drive (builds; untested on phone)
- [ ] Bench test (motor directions, ultrasonic)
- [ ] HSV tuned for run environment
- [ ] First full autonomous cone circuit
- [ ] v2 ideas: lap counting, outdoor lighting robustness, internet-remote control
