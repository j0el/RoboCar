# Raspberry Pi provisioning

Scripts for setting up the Pi 4 (vision/planning brain) from a fresh
Raspberry Pi OS Lite 64-bit install. See `../CLAUDE.md` and
`../ARCHITECTURE.md` for what runs on the Pi and why.

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
installed by this script — they're managed by `uv` via the repo's
`pyproject.toml` once you've cloned it (see next steps printed at the end
of the script).
