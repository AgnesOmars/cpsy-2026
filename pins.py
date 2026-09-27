# pins.py — BCM numbering. Change wiring here and nowhere else.

# DRV8833 motor driver
MOTOR_LEFT  = (13, 19)   # A1N1, A1N2
MOTOR_RIGHT = (12, 18)   # B1N1, B1N2
MOTOR_SLEEP = 26         # SLP/enable (verify: 24 or 26?)

# Rotary encoders (C1, C2)
ENC_LEFT  = (5, 6)
ENC_RIGHT = (17, 27)
