from machine import Pin, PWM
import time

def spin(pwm_pin, dir_pin, fwd=True, speed=50, secs=2):
    p = PWM(Pin(pwm_pin)); p.freq(500)
    d = Pin(dir_pin, Pin.OUT)
    d.value(0 if fwd else 1)
    p.duty_u16(int(speed / 100 * 65535))
    time.sleep(secs)
    p.duty_u16(0)

# Run one at a time in the Thonny REPL to identify each motor.
# Expected layout:
#   M1 (LF) front-left    M2 (RF) front-right
#   M4 (LB) rear-left     M3 (RB) rear-right

spin(12, 13)   # M1 — Left Front
# spin(15, 14)   # M2 — Right Front
# spin(17, 16)   # M3 — Right Back
# spin(18, 19)   # M4 — Left Back
