# CLAUDE.md — project context for Claude Code

## What this project is

Autonomous behaviors for an Adeept 4WD Mecanum Wheels Robotic Car (the
**Raspberry Pi Pico** version of the kit — a microcontroller, it cannot run
vision). A Raspberry Pi 4 (2GB, Raspberry Pi OS Lite 64-bit, headless) is the
vision/planning brain with a 720p USB 2.0 UVC camera (120° DFOV, forward-facing).
The Pi sends velocity commands to the Pico over USB serial; the Pico drives the
four motors.

Program 1 goal: detect orange cones and drive counterclockwise around the
**outside** of the cone cluster (any layout — circle, square, irregular).
Approach: reactive wall-following, keeping the nearest cone on the robot's left
at a fixed apparent size. Read **ARCHITECTURE.md** for the full design — serial
protocol, mecanum mixing, vision pipeline, state machine, bring-up checklist.

## Files

- `ARCHITECTURE.md` — authoritative design doc. Keep it updated when design changes.
- `pico_pi/` — everything that runs on the Pico. `pico_motor_controller.py` is
  the MicroPython firmware, flashed to the Pico as `main.py`. Serial protocol
  `V vx vy w\n` (each -100..100), mecanum mixing, 0.5 s watchdog stop. Also
  holds bench-test/bring-up scripts (`motor_test_individual.py`,
  `pico_movement_test.py`, `pico_serial_echo_test.py` — USB serial link test,
  no motors) and WiFi/ESP01 experiments (`pico_esp01_test.py`,
  `pico_esp01_diag.py`, `pico_wifi_drive.py`).
- `raspberry_pi/` — everything that runs on the Pi.
  - `cone_follower.py` — main Pi program. OpenCV HSV cone detection,
    SEARCH/FOLLOW/LOST state machine, 20 Hz command stream. Has `--dry-run`.
  - `hsv_tuner.py` — browser-based HSV tuning at http://<pi-ip>:8000 (Pi is headless).
  - `movement_test.py` — bench-test script that drives fixed moves over serial.
  - `serial_ping_test.py` — pairs with `pico_pi/pico_serial_echo_test.py` to
    verify the Pi<->Pico USB serial link before testing motors.
  - `pyproject.toml` — Pi-side Python deps (opencv-python-headless, pyserial, numpy),
    installed with `uv sync` (run from inside `raspberry_pi/`).
  - `setup.sh` — one-time Pi provisioning (git, python3, gh, uv; `gh auth
    login` for GitHub access). Run once on a fresh Raspberry Pi OS install,
    before cloning the repo.

## Current state / immediate next step

The code is written but **not yet run on hardware**. The blocker:

1. **The `PINS` table in `pico_pi/pico_motor_controller.py` contains PLACEHOLDER GPIO
   numbers.** The real motor pin assignments must be copied from Adeept's
   lesson/sample code. The Adeept docs and code are on this machine at:
   `~/Desktop/ADR032-Omni-directional_Mecanum_Wheels_Robotic_Car_Kit_for_Pico-20260413`
   → Find the motor driver pin definitions in their MicroPython lesson code
   (likely a motor.py / move.py or similar in the sample code). Note whether the
   board uses PWM+IN1+IN2 per motor or two-PWM (IN1/IN2 both PWM) drive — the
   Motor class in the firmware currently assumes PWM+dir pins and has a comment
   describing the two-PWM variant. Also check for a servo pin for the camera
   pan "neck" servo (unused so far, keep at 90°/center) and note WS2812 LED,
   buzzer, ultrasonic pins for later use.

2. After pins: bench test (wheels off ground), fix `DIRECTION` flags, then HSV
   tuning, dry run, live run — full sequence in ARCHITECTURE.md §7.

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

Ultrasonic collision backstop, lap detection/counting, outdoor lighting
robustness, camera pan servo usage.
