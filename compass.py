#! /usr/bin/python3

import math
import time
from gpiozero import LED
import board
import adafruit_icm20x

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c, address=0x68)

red_led = LED(23)      # on when facing magnetic north
blue_led = LED(24)    # on when board is level

def heading(ax, ay, az, mx, my, mz):
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
    mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
    my2 = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)
    return math.degrees(math.atan2(-my2, mx2)) % 360

def tilt(ax, ay, az):
    magnitude = math.sqrt(ax**2 + ay**2 + az**2)
    return math.degrees(math.acos(az / magnitude))

while True:
    ax, ay, az = icm.acceleration
    mx, my, mz = icm.magnetic

    h = heading(ax, ay, az, mx, my, mz)
    t = tilt(ax, ay, az)

    red_led.value = h < 15 or h > 345
    blue_led.value = t < 5
    print(f"heading={h:.1f}  tilt={t:.1f}  north={red_led.value}  level={blue_led.value}")
    time.sleep(0.2)
