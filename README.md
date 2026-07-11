# RoboCar

Autonomous behaviors for the **Adeept 4WD Omni-Directional Mecanum Wheels Robotic Car**
(Raspberry Pi Pico version), with vision and planning on a **Raspberry Pi 4** and a
forward-facing 720p USB camera (120° DFOV).

## Program 1: Cone Runner

Detect orange traffic cones with the camera and drive **counterclockwise around the
outside** of the cone cluster — circle, square, or any irregular layout. The approach
is reactive wall-following: keep the nearest cone on the robot's left at a fixed
apparent distance, and the boundary shape takes care of itself.

Full design details, serial protocol, control law, and bring-up checklist are in
[ARCHITECTURE.md](ARCHITECTURE.md).

## Hardware

| Part | Role |
|------|------|
| Adeept 4WD Mecanum kit (Raspberry Pi Pico + expansion board) | Chassis, motors, real-time motor control |
| Raspberry Pi 4 (2GB), Raspberry Pi OS Lite 64-bit | Vision + behavior ("the brain") |
| 720p USB 2.0 UVC camera, 120° DFOV | Forward-facing cone detection |
| USB cable, Pi ↔ Pico | Serial command link (`V vx vy w` at 20 Hz) |

## Repository contents

- `ARCHITECTURE.md` — system design, protocol, vision pipeline, behavior state machine
- `pico_pi/pico_motor_controller.py` — MicroPython firmware for the Pico (flash
  as `main.py`); parses velocity commands, mecanum mixing, PWM output, 0.5 s
  safety watchdog
- `cone_follower.py` — main Pi program: camera capture, HSV cone detection,
  SEARCH → FOLLOW → LOST state machine, serial output
- `hsv_tuner.py` — browser-based live tuning of the orange HSV range
  (the Pi stays headless; open `http://<pi-ip>:8000`)

## Quick start

On the Pi:

```bash
sudo apt install python3-opencv python3-serial python3-numpy
```

1. Edit the `PINS` table in `pico_pi/pico_motor_controller.py` to match the Adeept
   expansion board (copy pin numbers from Adeept's motor lesson code), then
   flash it to the Pico as `main.py`.
2. Bench-test with wheels off the ground; flip any `DIRECTION` flag for a
   wheel spinning the wrong way.
3. Run `python3 hsv_tuner.py`, tune until cones are solid white in the mask,
   and copy the values into `HSV_LOW` / `HSV_HIGH` in `cone_follower.py`.
4. `python3 cone_follower.py --dry-run` to verify decisions, then run live.

## Status

- [x] Architecture and v1 code
- [x] Pin mapping confirmed against Adeept lesson code
- [ ] Bench test (motor directions)
- [ ] HSV tuned for run environment
- [ ] First full autonomous lap
- [ ] v2 ideas: ultrasonic collision backstop, lap counting, outdoor lighting robustness
