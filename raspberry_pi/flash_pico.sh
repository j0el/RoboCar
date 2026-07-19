#!/usr/bin/env bash
# flash_pico.sh — push the current firmware to the Pico over USB serial.
#
# Copies pico_pi/pico_motor_controller.py onto the Pico as main.py, reboots
# it, and confirms the new firmware is alive by watching for its `D <cm>`
# ultrasonic telemetry. Run on the Pi after a `git pull` (works from the Mac
# too if the Pico is plugged in there).
#
#     bash flash_pico.sh [port]        # default port: /dev/ttyACM0
#
# Safe with the running firmware: mpremote interrupts it to get a REPL, and
# the firmware stops all motors on any exception. But the serial port must
# be free — stop cone_visitor.py (or any program holding the port) first;
# this script refuses to run if the port is busy.
set -euo pipefail

PORT="${1:-/dev/ttyACM0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE="$SCRIPT_DIR/../pico_pi/pico_motor_controller.py"

export PATH="$HOME/.local/bin:$PATH"   # uv tool install target

if ! command -v mpremote >/dev/null 2>&1; then
    echo "mpremote not found — install it with:  uv tool install mpremote" >&2
    exit 1
fi
if [ ! -f "$FIRMWARE" ]; then
    echo "Firmware not found at $FIRMWARE" >&2
    exit 1
fi
if [ ! -e "$PORT" ]; then
    echo "No serial device at $PORT — is the Pico plugged in? (try: ls /dev/ttyACM* /dev/tty.usbmodem*)" >&2
    exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser -s "$PORT" 2>/dev/null; then
    echo "$PORT is in use (cone_visitor.py or another program?). Stop it first:" >&2
    fuser -v "$PORT" >&2 || true
    exit 1
fi

echo "==> Copying $(basename "$FIRMWARE") -> ${PORT} as main.py"
mpremote connect "$PORT" cp "$FIRMWARE" :main.py

echo "==> Rebooting the Pico"
mpremote connect "$PORT" reset

echo "==> Waiting for the new firmware's telemetry (D lines) ..."
sleep 2   # let the Pico re-enumerate and boot main.py
if ! command -v timeout >/dev/null 2>&1; then
    echo "==> 'timeout' not available (macOS?) — skipping the telemetry check."
    echo "    Verify manually:  mpremote connect $PORT repl   (expect D lines)"
    exit 0
fi
if command -v stty >/dev/null 2>&1; then
    stty -F "$PORT" 115200 raw -echo 2>/dev/null \
        || stty -f "$PORT" 115200 raw -echo   # macOS uses -f
fi
if timeout 5 grep -qm1 "^D " "$PORT" 2>/dev/null; then
    echo "==> OK: firmware is running (ultrasonic telemetry seen)."
else
    echo "==> WARNING: no D telemetry within 5 s. The copy succeeded, but"
    echo "    verify the firmware boots:  mpremote connect $PORT repl"
    echo "    (Ctrl-D soft-reboots and shows any traceback; Ctrl-X exits.)"
    exit 1
fi
