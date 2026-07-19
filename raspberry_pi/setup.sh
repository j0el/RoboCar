#!/usr/bin/env bash
# One-time provisioning script for a fresh Raspberry Pi OS Lite 64-bit install.
# Installs git, python3, the GitHub CLI (gh), and uv (Python package/venv
# manager). Python *libraries* (opencv-python-headless, pyserial, numpy) are
# installed later via `uv sync` against the repo's pyproject.toml, not via
# apt. We use the headless OpenCV build (no imshow/GUI in this codebase, and
# the Pi is headless anyway) specifically so it doesn't pull in libGL/X11/GTK
# at all; libgomp1 below is the one native shared library those wheels still
# need (OpenMP runtime for numpy/opencv's linear algebra and threading).
set -euo pipefail

echo "==> Updating apt package index"
sudo apt-get update

echo "==> Installing base packages (git, python3, curl, ca-certificates, libgomp1)"
sudo apt-get install -y --no-install-recommends \
    git \
    python3 \
    ca-certificates \
    curl \
    libgomp1

echo "==> Installing GitHub CLI (gh)"
if ! command -v gh >/dev/null 2>&1; then
    sudo mkdir -p -m 755 /etc/apt/keyrings
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
        | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
    sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
        | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y gh
else
    echo "    gh already installed, skipping"
fi

echo "==> Installing uv (Python package/venv manager)"
if ! command -v uv >/dev/null 2>&1; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # uv's installer adds ~/.local/bin to PATH for future shells (via
    # .bashrc/.profile); export it here too so the rest of this script sees it.
    export PATH="$HOME/.local/bin:$PATH"
else
    echo "    uv already installed, skipping"
fi

echo "==> Installing mpremote (flashes/runs MicroPython on the Pico over USB)"
if ! command -v mpremote >/dev/null 2>&1; then
    uv tool install mpremote
else
    echo "    mpremote already installed, skipping"
fi

echo "==> Authenticating gh with GitHub"
if ! gh auth status >/dev/null 2>&1; then
    # Pin protocol to https + web flow so gh never prompts about SSH keys
    # (it only asks those questions when the ssh protocol is selected).
    gh auth login --hostname github.com --git-protocol https --web
else
    echo "    gh already authenticated, skipping"
fi

cat <<'EOF'

==> Done. Next steps:
  1. Clone the repo:       gh repo clone <your-github-username>/RoboCar
  2. cd RoboCar/raspberry_pi
  3. Install Python deps:  uv sync
  4. Flash the Pico:       bash flash_pico.sh
  5. Sanity-check vision:  uv run python hsv_tuner.py
EOF
