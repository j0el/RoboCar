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
# Ultrasonic safety stop: the front HC-SR04 is sampled continuously. When an
# obstacle is closer than SAFE_CM the forward component of any command is
# forced to zero (reverse, strafe and turn still work so the robot can get
# itself out). Each reading is reported to the Pi as a telemetry line:
#   D <cm>\n      distance in whole cm; D -1 = no echo (nothing in range)
#
# BRING-UP STEP 1 complete: pins confirmed from Adeept lesson code.
# Board uses two-PWM (IN1/IN2) H-bridge drive — see Motor class below.

import sys
import time
import select
from machine import Pin, PWM, time_pulse_us

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

ULTRA_TRIG = 3         # HC-SR04 trigger
ULTRA_ECHO = 2         # HC-SR04 echo
SAFE_CM = 15           # block forward motion when obstacle closer than this
SAFE_CLEAR_CM = 20     # ...until it is at least this far away (hysteresis)
ULTRA_PERIOD_MS = 70   # sample interval (~14 Hz)

# Other board pins (for future use):
#   Servo (camera pan "neck"): GPIO 7  -- keep at 0° (center) until needed
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


class Ultrasonic:
    """Front HC-SR04. read_cm() blocks at most ~18 ms (ECHO_TIMEOUT_US)."""

    ECHO_TIMEOUT_US = 18000   # ~3 m round trip; beyond that report "no echo"

    def __init__(self, trig, echo):
        self.trig = Pin(trig, Pin.OUT, value=0)
        self.echo = Pin(echo, Pin.IN)

    def read_cm(self):
        """Distance in cm, or None if no echo within range."""
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        t = time_pulse_us(self.echo, 1, self.ECHO_TIMEOUT_US)
        if t < 0:              # -1/-2: no echo started or it never ended
            return None
        return t / 58.0        # us -> cm (speed of sound, round trip)


motors = {name: Motor(direction=DIRECTION[name], **p) for name, p in PINS.items()}
ultrasonic = Ultrasonic(ULTRA_TRIG, ULTRA_ECHO)


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
    last_ultra = time.ticks_ms()
    cmd = (0, 0, 0)      # last velocity commanded by the Pi
    blocked = False      # ultrasonic safety: forward motion suppressed

    def apply_cmd():
        vx, vy, w = cmd
        if blocked and vx > 0:
            vx = 0
        apply_velocity(vx, vy, w)

    while True:
        # Non-blocking read of whatever serial bytes are available
        while poller.poll(0):
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                parsed = parse_line(buf)
                buf = ""
                if parsed is not None:
                    cmd = parsed
                    apply_cmd()
                    last_cmd = time.ticks_ms()
                    print("OK")
            else:
                buf += ch
                if len(buf) > 64:   # garbage guard
                    buf = ""

        # Ultrasonic safety stop: sample, report, re-apply on state change
        if time.ticks_diff(time.ticks_ms(), last_ultra) >= ULTRA_PERIOD_MS:
            last_ultra = time.ticks_ms()
            cm = ultrasonic.read_cm()
            print("D -1" if cm is None else "D %d" % int(cm + 0.5))
            was_blocked = blocked
            if cm is not None and cm < SAFE_CM:
                blocked = True
            elif cm is None or cm >= SAFE_CLEAR_CM:
                blocked = False
            # between SAFE_CM and SAFE_CLEAR_CM: keep previous state (hysteresis)
            if blocked != was_blocked:
                apply_cmd()

        # Watchdog: lost the Pi -> forget its last command and stop.
        # Checks cmd (not just wheel motion) so a stale forward command being
        # suppressed by `blocked` can't resume when the obstacle clears.
        if cmd != (0, 0, 0) and time.ticks_diff(time.ticks_ms(), last_cmd) > WATCHDOG_MS:
            cmd = (0, 0, 0)
            stop_all()

        time.sleep_ms(5)


try:
    main()
except BaseException:
    stop_all()
    raise
