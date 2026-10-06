import time
import math
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

# --- Test settings ---
SETPOINT = 2000    # target speed, steps per second
KP = 0.0008
KI = 0.002
KD = 0.0
DRIVE_TIME = 5     # seconds, same as the open-loop distance test
SAMPLE = 0.05

# --- Robot measurements ---
PPR = 692               # steps per wheel rotation
WHEEL_DIAMETER = 0.0445 # metres
WHEEL_BASE = 0.125      # metres between the wheel centres

slp = DigitalOutputDevice('GPIO26')
slp.on()
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))
enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

pid_left = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))
pid_right = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))

try:
    # Starting counts: the distance is measured from here.
    left0, right0 = enc.read()
    left_prev, right_prev = left0, right0
    t_start = time.perf_counter()
    t_prev = t_start

    # PID control loop, same as in pid_controller.py
    while time.perf_counter() - t_start < DRIVE_TIME:
        time.sleep(SAMPLE)
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt
        left_prev, right_prev, t_prev = left, right, t_now

        robot.value = (pid_left(left_speed), pid_right(right_speed))

    robot.stop()
    tdrive = time.perf_counter() - t_start

    time.sleep(0.5)  # let the wheels stop rolling before the final reading
    left1, right1 = enc.read()
finally:
    robot.stop()
    slp.off()
    enc.close()

# --- Distance and heading from the encoder steps ---
left_steps = left1 - left0
right_steps = right1 - right0

circumference = math.pi * WHEEL_DIAMETER
left_dist = left_steps / PPR * circumference
right_dist = right_steps / PPR * circumference
dist = (left_dist + right_dist) / 2
heading = math.degrees((right_dist - left_dist) / WHEEL_BASE)

print(f"Setpoint: {SETPOINT} steps/s, drove for {tdrive:.2f} s")
print(f"Steps:    left {left_steps}, right {right_steps}")
print(f"Distance: left {left_dist:.3f} m, right {right_dist:.3f} m, average {dist:.3f} m")
print(f"Heading change from encoders: {heading:.1f} deg (positive = turned left)")
