import time
import math
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

SPEED = 0.8            # motor speed, 0 to 1
DRIVE_TIME = 5         # seconds
PPR = 692              # steps per wheel rotation, from PPR test
WHEEL_DIAMETER = 0.0445 # metres
WHEEL_BASE = 0.125      # metres between the wheels, measure your robot

slp = DigitalOutputDevice('GPIO26')
slp.on()
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))
enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

try:
    left0, right0 = enc.read()
    tstart = time.perf_counter()
    robot.forward(SPEED)
    time.sleep(DRIVE_TIME)
    robot.stop()
    tdrive = time.perf_counter() - tstart

    time.sleep(0.5) 
    left1, right1 = enc.read()
finally:
    robot.stop()
    slp.off()
    enc.close()

left_steps = left1 - left0
right_steps = right1 - right0

circumference = math.pi * WHEEL_DIAMETER
left_dist = left_steps / PPR * circumference
right_dist = right_steps / PPR * circumference
dist = (left_dist + right_dist) / 2
heading = math.degrees((right_dist - left_dist) / WHEEL_BASE)

print(f"Speed: {SPEED}, drove for {tdrive:.2f} s")
print(f"Steps:    left {left_steps}, right {right_steps}")
print(f"Distance: left {left_dist:.3f} m, right {right_dist:.3f} m, average {dist:.3f} m")
print(f"Heading change from encoders: {heading:.1f} deg (positive = turned left)")
