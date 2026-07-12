# pico_serial_echo_test.py — runs directly on the Pico (Thonny "Run", or
# `mpremote run`). No need to save this as main.py.
#
# Communication-only test for the Pi <-> Pico USB serial link -- no motors,
# no PINS table, nothing that depends on the Adeept board wiring. Confirms
# the USB cable, port, and baud rate all work before testing
# pico_motor_controller.py.
#
# Protocol: reply "PONG" to a "PING" line; echo anything else back
# prefixed with "ECHO: ". Pair with raspberry_pi/serial_ping_test.py.

import sys
import time
import select

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)
buf = ""

print("Pico serial echo test ready")

while True:
    while poller.poll(0):
        ch = sys.stdin.read(1)
        if ch in ("\n", "\r"):
            line = buf.strip()
            buf = ""
            if line == "PING":
                print("PONG")
            elif line:
                print("ECHO: " + line)
        else:
            buf += ch
            if len(buf) > 200:   # garbage guard
                buf = ""
    time.sleep_ms(5)
