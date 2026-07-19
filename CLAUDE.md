# CLAUDE.md — project context for Claude Code

## What this project is

Autonomous behaviors for an Adeept 4WD Mecanum Wheels Robotic Car (the
**Raspberry Pi Pico** version of the kit — a microcontroller, it cannot run
vision). A Raspberry Pi 4 (2GB, Raspberry Pi OS Lite 64-bit, headless) is the
vision/planning brain with a 720p USB 2.0 UVC camera (120° DFOV, forward-facing).
The Pi sends velocity commands to the Pico over USB serial; the Pico drives the
four motors.

Program 1 (`cone_follower.py`): detect orange cones and drive counterclockwise
around the **outside** of the cone cluster (any layout — circle, square,
irregular) by reactive wall-following.
Program 2 (`cone_visitor.py`, the **master program**): visit the cones one at
a time counterclockwise with strafe-heavy mecanum motion, controlled and
monitored from a phone over the Pi's own WiFi AP. Read **ARCHITECTURE.md**
for the full design — serial protocol, mecanum mixing, vision pipeline, state
machines, WiFi/phone API, bring-up checklist.

## Files

- `ARCHITECTURE.md` — authoritative design doc. Keep it updated when design changes.
- `ENHANCEMENTS.md` — prioritized specs for future work (none implemented);
  when picking up an item from it, follow its spec and mark it done there.
- `pico_pi/` — everything that runs on the Pico. `pico_motor_controller.py` is
  the MicroPython firmware, flashed to the Pico as `main.py`. Serial protocol
  `V vx vy w\n` (each -100..100), mecanum mixing, 0.5 s watchdog stop, plus
  the HC-SR04 ultrasonic safety stop (forward blocked < 15 cm) and `D <cm>`
  distance telemetry back to the Pi. Also holds bench-test/bring-up scripts
  (`motor_test_individual.py`, `pico_movement_test.py`,
  `pico_serial_echo_test.py` — USB serial link test, no motors) and WiFi/ESP01
  experiments (`pico_esp01_test.py`, `pico_esp01_diag.py`,
  `pico_wifi_drive.py`) — the ESP01 path is superseded by the Pi-hosted AP.
- `raspberry_pi/` — everything that runs on the Pi.
  - `vision.py` — shared camera setup + HSV cone detection (tuned HSV values
    live here) used by the programs below.
  - `cone_visitor.py` — **master program**: cone-to-cone CCW visitor state
    machine + HTTP server on :8080 (annotated MJPEG `/stream`, `/start`,
    `/stop`, `/drive`, `/status`) for the phone/browser. Has `--dry-run`.
  - `cone_follower.py` — Program 1: outside-orbit wall following,
    SEARCH/FOLLOW/LOST state machine, 20 Hz command stream. Has `--dry-run`.
  - `hsv_tuner.py` — browser-based HSV tuning at http://<pi-ip>:8000 (Pi is headless).
  - `setup_ap.sh` — one-time WiFi hotspot setup (SSID RoboCar / robocar1,
    Pi at 10.42.0.1) via NetworkManager.
- `sim/` — no-hardware test harnesses (run on the Mac).
  - `pi_server_sim.py` — synthetic 2D cone world driving the REAL Visitor
    FSM, detection, and HTTP handler from cone_visitor.py; lets the Android
    app/browser connect to the Mac's IP:8080. Mirrors the firmware's
    forward-block rule and counts cone contacts (any bump = behavior bug).
  - `pico_exerciser.py` — plays the Pi's role over USB serial to a real
    Pico: 20 Hz keyboard drive, `D` telemetry display, watchdog test.
    Needs pyserial (`uv run --with pyserial python3 sim/pico_exerciser.py`).
  - `movement_test.py` — bench-test script that drives fixed moves over serial.
  - `serial_ping_test.py` — pairs with `pico_pi/pico_serial_echo_test.py` to
    verify the Pi<->Pico USB serial link before testing motors.
  - `pyproject.toml` — Pi-side Python deps (opencv-python-headless, pyserial, numpy),
    installed with `uv sync` (run from inside `raspberry_pi/`).
  - `setup.sh` — one-time Pi provisioning (git, python3, gh, uv; `gh auth
    login` for GitHub access). Run once on a fresh Raspberry Pi OS install,
    before cloning the repo.

## Current state / immediate next step

The code is written but **not yet run on hardware** (waiting on parts to
finalize the build). Pin assignments ARE confirmed (copied from Adeept lesson
code into the firmware — motors, ultrasonic trig=GP3/echo=GP2, servo GP7,
buzzer GP26, WS2812 GP11). The Adeept vendor folder is **no longer on this
machine**; re-download the ADR032 kit archive if the original lesson code is
needed again. USB serial Pi↔Pico is verified working.

Next steps, in order (full sequence in ARCHITECTURE.md §8): bench test motors
(wheels off ground, fix `DIRECTION` flags), ultrasonic check, HSV tuning,
`cone_visitor.py --dry-run`, WiFi AP + phone app check, live run.

## Conventions

- Pi-side code: Python 3, deps managed with `uv` against
  `raspberry_pi/pyproject.toml` (opencv-python-headless, pyserial, numpy — install
  with `uv sync`, run with `uv run python <script>.py` from inside
  `raspberry_pi/`), not apt. Keep it lightweight — the Pi has 2GB RAM.
- Pico-side: MicroPython, single-file firmware, no external libs.
- Coordinate/sign conventions (do not change without updating both sides and
  ARCHITECTURE.md): vx + = forward, vy + = strafe right, w + = CCW yaw.
- Safety invariants: Pico watchdog stop stays; Pi streams commands at 20 Hz;
  every exit path sends `V 0 0 0`.
- Deployment: this repo is developed on the Mac and pulled/copied onto the Pi.

## v2 backlog

Lap detection/counting, outdoor lighting robustness, camera pan servo usage,
internet-remote control/monitoring (reuse the cone_visitor HTTP API behind a
different transport). Ultrasonic collision backstop: done (in firmware).
