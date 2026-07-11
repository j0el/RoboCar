# pico_motor_controller.py  (save to the Pico as main.py)
#
# MicroPython firmware for the Adeept 4WD Mecanum kit's Raspberry Pi Pico.
# Listens on USB serial for velocity commands from the Raspberry Pi 4 and
# drives the four DC motors with mecanum mixing.
#
# Protocol (one ASCII line per command):
#   V <vx> <vy> <w>\n     each value -100..100
#     vx: forward(+)/backward(-)
#     vy: strafe right(+)/left(-)
#     w : yaw counterclockwise(+)/clockwise(-)
#
# Safety: if no valid command arrives for WATCHDOG_MS, all motors stop.
#
# BRING-UP STEP 1 complete: pins confirmed from Adeept lesson code.
# Board uses two-PWM (IN1/IN2) H-bridge drive — see Motor class below.

import sys
import time
import select
from machine import Pin, PWM

# ---------------- CONFIG: EDIT THESE FOR YOUR BOARD ----------------
# Board uses two-PWM (IN1/IN2) drive: direction is determined by which pin
# carries the PWM signal; the other pin is held at 0.
# Pins from Adeept lesson code (06_motor.py / pico_car.py):
#   M1=FL  M2=FR
#   M4=BL  M3=BR
# left front wheel M1

PINS = {
    #            in1  in2
    "FL": dict(in1=12, in2=13),  # front-left  (M1)
    "FR": dict(in1=15, in2=14),  # front-right (M2)
    "BL": dict(in1=18, in2=19),  # back-left   (M4)
    "BR": dict(in1=17, in2=16),  # back-right  (M3)
}
# Flip to -1 for any wheel that spins backward during bench test:
DIRECTION = {"FL": 1, "FR": 1, "BL": 1, "BR": 1}

# Other board pins (for future use):
#   Servo (camera pan "neck"): GPIO 7  -- keep at 0° (center) until needed
#   Ultrasonic: trig=GPIO 3, echo=GPIO 2
#   Buzzer: GPIO 26
#   WS2812 LED (4 LEDs): GPIO 11
#   Line tracking: left=GPIO 6, mid=GPIO 5, right=GPIO 4
#   LCD I2C: SDA=GPIO 20, SCL=GPIO 21 (addr 0x27)

PWM_FREQ = 1000        # Hz
WATCHDOG_MS = 500      # stop if no command for this long
MAX_CMD = 100
# --------------------------------------------------------------------


class Motor:
    def __init__(self, in1, in2, direction):
        self.in1 = PWM(Pin(in1))
        self.in2 = PWM(Pin(in2))
        self.in1.freq(PWM_FREQ)
        self.in2.freq(PWM_FREQ)
        self.dir = direction

    def set(self, speed):
        """speed: -100..100. Sign = direction, magnitude = duty."""
        speed = max(-MAX_CMD, min(MAX_CMD, speed)) * self.dir
        duty = int(abs(speed) / MAX_CMD * 65535)
        if speed > 0:
            self.in1.duty_u16(duty); self.in2.duty_u16(0)
        elif speed < 0:
            self.in1.duty_u16(0); self.in2.duty_u16(duty)
        else:
            self.in1.duty_u16(0); self.in2.duty_u16(0)

    def stop(self):
        self.set(0)


motors = {name: Motor(direction=DIRECTION[name], **p) for name, p in PINS.items()}


def stop_all():
    for m in motors.values():
        m.stop()


def apply_velocity(vx, vy, w):
    """Standard mecanum mix for 45-degree rollers."""
    wheel = {
        "FL": vx + vy - w,
        "FR": vx - vy + w,
        "BL": vx - vy - w,
        "BR": vx + vy + w,
    }
    # Normalize so no wheel exceeds 100 (preserves the motion's direction)
    peak = max(abs(v) for v in wheel.values())
    scale = MAX_CMD / peak if peak > MAX_CMD else 1.0
    for name, v in wheel.items():
        motors[name].set(v * scale)


def parse_line(line):
    """Return (vx, vy, w) or None if malformed."""
    parts = line.split()
    if len(parts) != 4 or parts[0] != "V":
        return None
    try:
        vx, vy, w = (max(-MAX_CMD, min(MAX_CMD, int(p))) for p in parts[1:])
        return vx, vy, w
    except ValueError:
        return None


def main():
    stop_all()
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    buf = ""
    last_cmd = time.ticks_ms()
    moving = False

    while True:
        # Non-blocking read of whatever serial bytes are available
        while poller.poll(0):
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                cmd = parse_line(buf)
                buf = ""
                if cmd is not None:
                    apply_velocity(*cmd)
                    moving = cmd != (0, 0, 0)
                    last_cmd = time.ticks_ms()
                    print("OK")
            else:
                buf += ch
                if len(buf) > 64:   # garbage guard
                    buf = ""

        # Watchdog: lost the Pi -> stop
        if moving and time.ticks_diff(time.ticks_ms(), last_cmd) > WATCHDOG_MS:
            stop_all()
            moving = False

        time.sleep_ms(5)


try:
    main()
except BaseException:
    stop_all()
    raise
