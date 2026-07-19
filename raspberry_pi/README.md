# Raspberry Pi

Everything that runs on the Pi 4 (vision/planning brain): provisioning,
the vision/behavior programs, and their Python deps. See `../CLAUDE.md`
and `../ARCHITECTURE.md` for what runs here and why.

- `setup.sh` — one-time OS-level provisioning (see below).
- `flash_pico.sh` — push the current firmware to the Pico over USB
  (mpremote cp as `main.py` + reboot + telemetry check). Run after every
  `git pull` that touches `pico_pi/pico_motor_controller.py`, with
  cone_visitor stopped. `mpremote run <script>` runs the `pico_pi` bench
  scripts without saving them as `main.py`.
- `setup_ap.sh` — one-time WiFi hotspot setup for the phone controller
  (SSID `RoboCar`, password `robocar1`, Pi at `10.42.0.1`). Run with sudo.
- `pyproject.toml` — Python deps (opencv-python-headless, pyserial, numpy), installed
  with `uv sync` run from this directory.
- `vision.py` — shared camera setup + HSV cone detection; the tuned HSV
  values live here.
- `cone_visitor.py` — **master program**: drives cone-to-cone CCW
  (strafe-heavy) and serves the phone/browser UI on port 8080 (annotated
  MJPEG stream, start/stop, manual drive, status). Reads the Pico's `D <cm>`
  ultrasonic telemetry for cone arrival. `--dry-run` = camera + web UI only.
- `cone_follower.py` — Program 1: camera capture, HSV cone detection,
  SEARCH/FOLLOW/LOST state machine, serial output to the Pico.
- `hsv_tuner.py` — browser-based live tuning of the orange HSV range.
- `movement_test.py` — bench-test script, drives a fixed move sequence over
  serial to exercise every axis.

## `setup.sh`

Run once, right after first boot / `raspi-config` basics (hostname, SSH,
Wi-Fi, locale):

```
bash raspberry_pi/setup.sh
```

Installs via apt: `git`, `python3`, `curl`, `ca-certificates`, and the
GitHub CLI (`gh`, from GitHub's own apt repo). Installs `uv` via the
official installer script (not apt — Raspberry Pi OS doesn't package it).
Then runs `gh auth login` so you can clone/push over HTTPS with a browser
device code, no manual SSH key to generate or upload.

Python *libraries* (opencv-python-headless, pyserial, numpy) are intentionally not
installed by this script — they're managed by `uv` via `pyproject.toml`
in this same directory (`cd raspberry_pi && uv sync`, see next steps
printed at the end of the script).
