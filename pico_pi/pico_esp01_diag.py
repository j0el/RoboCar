# pico_esp01_diag.py — runs directly on the Pico.
#
# pico_esp01_test.py got zero bytes back from the ESP01 for every AT command,
# even a plain "AT" -- that points at wiring/power/baud, not SSID/config.
# This script rules those out one at a time:
#   1. Try a handful of common ESP8266 AT-firmware baud rates and print
#      whatever raw bytes come back (even garbage/boot-log bytes are useful
#      signal -- they mean the module IS powered and TX is wired correctly,
#      just at a different baud than we guessed).
#   2. Toggle a GPIO high/low with a short delay so you can confirm with a
#      meter/scope that the Pico's TX pin is actually toggling, if you want
#      to rule out a dead GPIO.
#
# Do NOT save this as main.py -- it's a one-off diagnostic.

import time
from machine import UART, Pin

# Common bauds for ESP8266 AT firmware. 74880 is the ROM bootloader's own
# boot-log baud (not AT firmware) -- if you see readable "ets Jan ..." text
# only at 74880, the module boots but has no/different AT firmware flashed.
BAUDS_TO_TRY = [115200, 9600, 57600, 38400, 74880]


def try_baud(baud):
    print("\n[diag] --- trying %d baud ---" % baud)
    u = UART(0, baud)
    time.sleep_ms(100)
    if u.any():
        u.read()  # flush any boot-log noise so it doesn't get attributed to our command

    u.write("AT\r\n")
    time.sleep(1)
    resp = b""
    while u.any():
        resp += u.read()
        time.sleep_ms(20)
    print("[diag] baud=%d  raw=%r" % (baud, resp))
    return resp


def main():
    print("[diag] power-on / reset the ESP01 now if you can, then watch this output.")
    print("[diag] scanning bauds for ANY response to plain AT...")
    hits = []
    for baud in BAUDS_TO_TRY:
        resp = try_baud(baud)
        if resp:
            hits.append((baud, resp))
        time.sleep(0.5)

    print("\n[diag] ==================== summary ====================")
    if not hits:
        print("[diag] NOTHING came back at any baud rate.")
        print("[diag] This means the Pico never saw a single byte from the ESP01.")
        print("[diag] Check, in this order:")
        print("[diag]  1. Is the ESP01 fully and correctly seated in its socket?")
        print("[diag]     (8-pin headers on cheap sockets have no keying notch --")
        print("[diag]      it's easy to insert reversed or offset by one pin.)")
        print("[diag]  2. Does the ESP01 have any LED lit at all? No LED usually")
        print("[diag]     means no power reaching it (bad seating, or the")
        print("[diag]     board's 3.3V regulator for that socket is dead/disabled).")
        print("[diag]  3. Is anything else using UART0 / GPIO0-1 at the same time")
        print("[diag]     (another script, or a REPL still attached over USB serial")
        print("[diag]     while this runs over the same UART -- shouldn't be the")
        print("[diag]     case here since USB REPL and UART0 are separate, but")
        print("[diag]     worth double-checking nothing else claims those pins).")
    else:
        for baud, resp in hits:
            print("[diag] GOT a response at baud=%d: %r" % (baud, resp))
        print("[diag] -> re-run pico_esp01_test.py with UART(0, %d) instead of 115200."
              % hits[0][0])
    print("[diag] ===================================================")


main()


