# pico_wifi_drive.py — runs directly on the Pico.
#
# WiFi remote-control firmware: brings up the ESP01 hotspot (same bring-up as
# pico_esp01_test.py) and drives the mecanum wheels from "V <vx> <vy> <w>\n"
# commands received over its TCP socket -- the same protocol the Pi<->Pico
# USB serial link uses (see ARCHITECTURE.md), so the RoboCar Android app
# (android_controller/) and cone_follower.py speak an identical wire format.
#
# This is a manual-control path, independent of the USB-serial firmware in
# pico_motor_controller.py -- useful for driving the chassis by hand over
# WiFi during bring-up, before the Pi/camera are wired in.
#
# Do NOT save this as main.py -- it's a bring-up tool, not the production
# USB-serial firmware.

import time
from machine import UART, Pin, PWM, I2C

SSID = "RoboCar"
PASSWORD = ""
PORT = "4000"

uart = UART(0, 115200)

# ---------------- Motor config: same as pico_motor_controller.py ----------------
PINS = {
    #            in1  in2
    "FL": dict(in1=12, in2=13),  # front-left  (M1)
    "FR": dict(in1=15, in2=14),  # front-right (M2)
    "BL": dict(in1=18, in2=19),  # back-left   (M4)
    "BR": dict(in1=17, in2=16),  # back-right  (M3)
}
DIRECTION = {"FL": 1, "FR": 1, "BL": 1, "BR": 1}

PWM_FREQ = 1000
MAX_CMD = 100
WATCHDOG_MS = 500  # stop if no valid V command for this long, same as the USB firmware

LCD_SDA, LCD_SCL, LCD_ADDR = 20, 21, 0x27
# ----------------------------------------------------------------------------


class Motor:
    def __init__(self, in1, in2, direction):
        self.in1 = PWM(Pin(in1))
        self.in2 = PWM(Pin(in2))
        self.in1.freq(PWM_FREQ)
        self.in2.freq(PWM_FREQ)
        self.dir = direction

    def set(self, speed):
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


class LCD1602:
    """Minimal driver for a 16x2 HD44780 LCD behind a PCF8574 I2C backpack.
    Same implementation as pico_movement_test.py / pico_esp01_test.py."""

    def __init__(self, i2c, addr=LCD_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.bl = 0x08
        time.sleep_ms(50)
        for _ in range(3):
            self._write4(0x30)
            time.sleep_ms(5)
        self._write4(0x20)
        self._cmd(0x28)
        self._cmd(0x0C)
        self._cmd(0x06)
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


lcd = LCD1602(I2C(0, sda=Pin(LCD_SDA), scl=Pin(LCD_SCL), freq=100000))


def send_cmd(cmd, wait=1.0):
    uart.write(cmd + "\r\n")
    time.sleep(wait)
    resp = b""
    while uart.any():
        resp += uart.read()
    print("[wifi_drive] %-28s -> %r" % (cmd, resp))
    return resp


def setup():
    uart.write("+++")
    time.sleep(1)
    if uart.any():
        uart.read()

    ok = send_cmd("AT")
    if b"OK" not in ok:
        lcd.show("ESP01 no reply", "check wiring")
        print("[wifi_drive] no response to plain AT -- module isn't answering.")
        return False

    send_cmd("AT+CWMODE=2")
    send_cmd("AT+RST", wait=2.0)
    send_cmd('AT+CWSAP="%s","%s",11,0' % (SSID, PASSWORD))
    send_cmd("AT+CIPMUX=1")
    send_cmd("AT+CIPSERVER=1," + PORT)
    ip_resp = send_cmd("AT+CIFSR")

    if b"192.168.4.1" not in ip_resp:
        lcd.show("AP setup failed", "see REPL log")
        print("[wifi_drive] AT+CIFSR didn't report the expected IP.")
        return False

    print("[wifi_drive] hotspot '%s' up -- connect to 192.168.4.1:%s" % (SSID, PORT))
    lcd.show("Ready", "192.168.4.1:" + PORT)
    return True


rx_buffer = ""


def poll_command():
    """Read one +IPD frame if present, buffer it, and return the most
    recent complete 'V vx vy w' line seen (or None)."""
    global rx_buffer
    s = uart.read()
    if s is None:
        return None
    s = s.decode()
    if "+IPD" not in s:
        return None
    n1 = s.find("+IPD,")
    n2 = s.find(":", n1)
    if n2 == -1:
        return None
    rx_buffer += s[n2 + 1:]

    latest = None
    while "\n" in rx_buffer:
        line, rx_buffer = rx_buffer.split("\n", 1)
        line = line.strip()
        if line.startswith("V "):
            latest = line
    return latest


def parse_v_line(line):
    parts = line.split()
    if len(parts) != 4 or parts[0] != "V":
        return None
    try:
        vx, vy, w = (max(-MAX_CMD, min(MAX_CMD, int(p))) for p in parts[1:])
        return vx, vy, w
    except ValueError:
        return None


def main():
    if not setup():
        return
    print("[wifi_drive] waiting for commands...")
    last_cmd_ms = time.ticks_ms()
    moving = False
    try:
        while True:
            line = poll_command()
            if line:
                parsed = parse_v_line(line)
                if parsed:
                    vx, vy, w = parsed
                    apply_velocity(vx, vy, w)
                    moving = (vx, vy, w) != (0, 0, 0)
                    last_cmd_ms = time.ticks_ms()
                    lcd.show("V %d %d %d" % (vx, vy, w), "connected")

            if moving and time.ticks_diff(time.ticks_ms(), last_cmd_ms) > WATCHDOG_MS:
                stop_all()
                moving = False
                lcd.show("Ready", "192.168.4.1:" + PORT)

            time.sleep_ms(20)
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        print("[wifi_drive] stopped.")


main()
