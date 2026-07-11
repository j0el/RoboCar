# pico_esp01_test.py — runs directly on the Pico, no Pi/motors/vision involved.
#
# Bench test for the ESP01 (ESP8266) WiFi module bundled with this kit.
# Confirms the module answers AT commands, brings up its WiFi hotspot, and
# that data sent to its TCP socket arrives over UART. No car movement here
# -- see pico_movement_test.py for chassis-only testing.
#
# Wiring/values taken from Adeept's own sample (Code/ESP8266/wifi_pico.py in
# the kit docs): UART0 defaults to TX=GPIO0/RX=GPIO1 on this board, LCD is
# I2C0 on GPIO20/21 at addr 0x27 (same as pico_movement_test.py).
#
# Do NOT save this as main.py -- it's a one-off bench test.

import time
from machine import UART, Pin, I2C

SSID = "RoboCar"
PASSWORD = ""
PORT = "4000"

uart = UART(0, 115200)

LCD_SDA, LCD_SCL, LCD_ADDR = 20, 21, 0x27


class LCD1602:
    """Minimal driver for a 16x2 HD44780 LCD behind a PCF8574 I2C backpack.
    Same implementation as pico_movement_test.py -- kept inline per this
    project's single-file-firmware convention."""

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
    """Send an AT command and print whatever comes back, so bring-up
    failures are visible instead of silently discarded (Adeept's own sample
    reads and throws the response away)."""
    uart.write(cmd + "\r\n")
    time.sleep(wait)
    resp = b""
    while uart.any():
        resp += uart.read()
    print("[esp01_test] %-28s -> %r" % (cmd, resp))
    return resp


def receive_data():
    """Parse one +IPD,<id>,<len>:<data> frame if present. Returns (id, data)
    or (None, None)."""
    s = uart.read()
    if s is None:
        return None, None
    s = s.decode()
    if "+IPD" not in s:
        return None, None
    n1 = s.find("+IPD,")
    n2 = s.find(",", n1 + 5)
    conn_id = int(s[n1 + 5:n2])
    n3 = s.find(":")
    payload = s[n3 + 1:].strip()
    return conn_id, payload


def setup():
    uart.write("+++")
    time.sleep(1)
    if uart.any():
        uart.read()

    ok = send_cmd("AT")                  # expect OK -- module alive
    if b"OK" not in ok:
        lcd.show("ESP01 no reply", "check wiring")
        print("[esp01_test] no response to plain AT -- module isn't answering "
              "at all. Run pico_esp01_diag.py before trusting anything below.")
        return False

    send_cmd("AT+CWMODE=2")              # 2 = access point mode
    send_cmd("AT+RST", wait=2.0)         # reboot into the new mode
    send_cmd('AT+CWSAP="%s","%s",11,0' % (SSID, PASSWORD))
    send_cmd("AT+CIPMUX=1")              # allow multiple connections
    send_cmd("AT+CIPSERVER=1," + PORT)   # start TCP server
    ip_resp = send_cmd("AT+CIFSR")       # print assigned IP (should be 192.168.4.1)

    if b"192.168.4.1" not in ip_resp:
        lcd.show("AP setup failed", "see REPL log")
        print("[esp01_test] AT+CIFSR didn't report the expected IP -- the "
              "hotspot may not actually be up even though earlier commands ran.")
        return False

    print("[esp01_test] hotspot '%s' up -- connect to 192.168.4.1:%s" % (SSID, PORT))
    lcd.show("IP:192.168.4.1", "Port:" + PORT)
    return True


def main():
    if not setup():
        return
    print("[esp01_test] waiting for a client to connect and send data...")
    try:
        while True:
            conn_id, data = receive_data()
            if data is not None:
                print("[esp01_test] recv from #%s: %r" % (conn_id, data))
                lcd.show("Recv:", data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print("[esp01_test] stopped.")


main()
