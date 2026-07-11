# Raspberry Pi

Everything that runs on the Pi 4 (vision/planning brain): provisioning,
the vision/behavior programs, and their Python deps. See `../CLAUDE.md`
and `../ARCHITECTURE.md` for what runs here and why.

- `setup.sh` — one-time OS-level provisioning (see below).
- `pyproject.toml` — Python deps (opencv-python, pyserial, numpy), installed
  with `uv sync` run from this directory.
- `cone_follower.py` — main program: camera capture, HSV cone detection,
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

Python *libraries* (opencv-python, pyserial, numpy) are intentionally not
installed by this script — they're managed by `uv` via `pyproject.toml`
in this same directory (`cd raspberry_pi && uv sync`, see next steps
printed at the end of the script).
