# pico_movement_test.py — runs directly on the Pico, no Pi/serial involved.
#
# Standalone chassis bring-up test: drives the same move sequence as
# movement_test.py, but locally on the Pico (via Thonny "Run", or copied to
# the Pico as a one-off script). Use this before the Pi/camera are in the
# loop, to confirm wiring, DIRECTION flags, and mecanum mixing.
#
# Pins/mixing copied from pico_motor_controller.py. Do NOT save this as
# main.py — it's a one-shot bench test, not the serial firmware.

import time
from machine import Pin, PWM, I2C

# ---------------- CONFIG: same as pico_motor_controller.py ----------------
PINS = {
    #            in1  in2
    "FL": dict(in1=12, in2=13),  # front-left  (M1)
    "FR": dict(in1=15, in2=14),  # front-right (M2)
    "BL": dict(in1=18, in2=19),  # back-left   (M4)
    "BR": dict(in1=17, in2=16),  # back-right  (M3)
}
# Flip to -1 for any wheel that spins backward during bench test:
DIRECTION = {"FL": 1, "FR": 1, "BL": 1, "BR": 1}

PWM_FREQ = 1000
MAX_CMD = 100

SPEED = 50        # drive speed, -100..100
SPIN_SPEED = 50   # yaw speed for the 360 spins

# 360-degree spin duration is NOT calibrated -- there's no encoder feedback,
# so this is a guess. Time an actual spin with a stopwatch and adjust
# SPIN_SECS until it comes back to its starting heading.
SPIN_SECS = 3.0

# LCD I2C: SDA=GPIO 20, SCL=GPIO 21 (addr 0x27) — from pico_motor_controller.py
LCD_SDA, LCD_SCL, LCD_ADDR = 20, 21, 0x27
# ----------------------------------------------------------------------------


class LCD1602:
    """Minimal driver for a 16x2 HD44780 LCD behind a PCF8574 I2C backpack
    (the common 0x27 module). Written inline rather than pulling in a
    third-party lcd lib, per this project's single-file-firmware convention."""

    def __init__(self, i2c, addr=LCD_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.bl = 0x08  # backlight on
        time.sleep_ms(50)
        for _ in range(3):
            self._write4(0x30)
            time.sleep_ms(5)
        self._write4(0x20)   # switch to 4-bit mode
        self._cmd(0x28)      # 4-bit, 2 line, 5x8 font
        self._cmd(0x0C)      # display on, cursor off, blink off
        self._cmd(0x06)      # entry mode: increment, no shift
        self.clear()

    def _strobe(self, data):
        self.i2c.writeto(self.addr, bytes([data | 0x04 | self.bl]))
        time.sleep_us(1)
        self.i2c.writeto(self.addr, bytes([(data & ~0x04) | self.bl]))
        time.sleep_us(100)

    def _write4(self, data):
        self.i2c.writeto(self.addr, bytes([data | self.bl]))
        self._strobe(data)

    def _cmd(self, cmd):
        self._write4(cmd & 0xF0)
        self._write4((cmd << 4) & 0xF0)

    def _data(self, data):
        self._write4((data & 0xF0) | 0x01)
        self._write4(((data << 4) & 0xF0) | 0x01)

    def clear(self):
        self._cmd(0x01)
        time.sleep_ms(2)

    def move_to(self, col, row):
        self._cmd(0x80 | (col + (0x40 if row else 0x00)))

    def putstr(self, text):
        for ch in text:
            self._data(ord(ch))

    def show(self, line1, line2=""):
        self.clear()
        self.move_to(0, 0)
        self.putstr(line1[:16])
        self.move_to(0, 1)
        self.putstr(line2[:16])


lcd_i2c = I2C(0, sda=Pin(LCD_SDA), scl=Pin(LCD_SCL), freq=100000)
lcd = LCD1602(lcd_i2c)


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
    peak = max(abs(v) for v in wheel.values())
    scale = MAX_CMD / peak if peak > MAX_CMD else 1.0
    for name, v in wheel.items():
        motors[name].set(v * scale)


def run_phase(label, vx, vy, w, secs):
    print("[movement_test]", label, "vx=%+d vy=%+d w=%+d" % (vx, vy, w), secs, "s")
    lcd.show("Move:", label)
    apply_velocity(vx, vy, w)
    time.sleep(secs)
    stop_all()


# vx: forward(+)/back(-)   vy: strafe right(+)/left(-)   w: CCW(+)/CW(-)
SEQUENCE = [
    ("forward",                  SPEED,  0,     0,           2.0),
    ("backward",                -SPEED,  0,     0,           2.0),
    ("strafe right",             0,      SPEED, 0,           2.0),
    ("strafe left",              0,     -SPEED, 0,           2.0),
    ("diagonal right-forward",   SPEED,  SPEED, 0,           2.0),
    ("diagonal left-backward",  -SPEED, -SPEED, 0,           2.0),
    ("diagonal right-backward", -SPEED,  SPEED, 0,           2.0),
    ("diagonal left-forward",    SPEED, -SPEED, 0,           2.0),
    ("spin right (CW) 360",      0, 0, -SPIN_SPEED,  SPIN_SECS),
    ("spin left (CCW) 360",      0, 0,  SPIN_SPEED,  SPIN_SECS),
]


def main():
    stop_all()
    print("[movement_test] starting")
    lcd.show("Movement test", "waiting 10s...")
    time.sleep(10)
    try:
        while True:
            for label, vx, vy, w, secs in SEQUENCE:
                run_phase(label, vx, vy, w, secs)
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        lcd.show("Stopped")
        print("[movement_test] stopped.")


main()
