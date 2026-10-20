import time
import board
import adafruit_tcs34725
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

# ---------------------------------------------------------------
# Settings: change these between tests
# ---------------------------------------------------------------
SETPOINT = 600     # Base speed for both wheels, in encoder steps per second.
                   # 692 steps = one wheel rotation, so 600 is just under 1 rotation/s.
K = 1.0            # Steering strength. Higher = sharper turns, lower = smoother.
KP = 0.0008        # Wheel PID: proportional gain
KI = 0.002         # Wheel PID: integral gain
KD = 0.0           # Wheel PID: derivative gain

BLACK = 300        # Measured c on the black line
WHITE = 4700       # Measured c on the white floor
TH = (BLACK + WHITE) / 2   # The edge we follow (2500)

# ---------------------------------------------------------------
# Hardware setup
# ---------------------------------------------------------------
# Color sensor: short integration time so we get new readings quickly.
sensor = adafruit_tcs34725.TCS34725(board.I2C())
sensor.integration_time = 24
sensor.gain = 4

# The motor driver has a sleep pin. It must be on for the motors to run.
slp = DigitalOutputDevice('GPIO26')
slp.on()

robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))

# Starts the C++ decoder with our encoder pins and talks to it over UDP.
enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

# ---------------------------------------------------------------
# PID controllers: one per wheel
# ---------------------------------------------------------------
pid_left = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))
pid_right = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))

try:
    left_prev, right_prev = enc.read()
    t_prev = time.perf_counter()

    # No sleep needed: reading the color sensor already takes ~25 ms.
    while True:
        # 1. SENSE: how far is the sensor from the edge?
        #    error = -0.5 on black, 0 on the edge, +0.5 on white
        r, g, b, c = sensor.color_raw
        error = (c - TH) / (WHITE - BLACK)

        # 2. MEASURE: wheel speeds in steps per second
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt
        left_prev, right_prev, t_prev = left, right, t_now

        # 3. DECIDE (follow the left edge of the line):
        #    on black (error < 0) -> left wheel slower, right faster -> turn left
        #    on white (error > 0) -> left wheel faster, right slower -> turn right
        #    the further from the edge, the harder it turns
        pid_left.setpoint = max(0, SETPOINT * (1 + K * error))
        pid_right.setpoint = max(0, SETPOINT * (1 - K * error))

        # 4. CALCULATE: each wheel PID reaches its target speed
        left_cmd = pid_left(left_speed)
        right_cmd = pid_right(right_speed)

        # 5. ACT
        robot.value = (left_cmd, right_cmd)

        print(f"c={c:5d} err={error:+.2f}  "
              f"left {left_speed:5.0f} ({left_cmd:.2f})  right {right_speed:5.0f} ({right_cmd:.2f})")

finally:
    # Always runs, even after Ctrl+C: stop motors, sleep driver, stop decoder.
    robot.stop()
    slp.off()
    enc.close()