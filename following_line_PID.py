import time
import board
import adafruit_tcs34725
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

# ---------------------------------------------------------------
# Settings: change these between tests
# ---------------------------------------------------------------
SETPOINT = 600     # Target speed for the driving wheel, in encoder steps per second.
                   # 692 steps = one wheel rotation, so 600 is just under 1 rotation/s.
KP = 0.0008        # Proportional gain
KI = 0.002         # Integral gain
KD = 0.0           # Derivative gain
TH = 2500          # Clear-channel threshold: below = black line, above = white floor

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

# Last motor power each PID gave while its wheel was driving.
# Used to restart a wheel with roughly the right power after it was stopped.
left_out = 0.0
right_out = 0.0

try:
    left_prev, right_prev = enc.read()
    t_prev = time.perf_counter()

    # No sleep needed: reading the color sensor already takes ~25 ms.
    while True:
        # 1. SENSE: is the sensor over the black line?
        r, g, b, c = sensor.color_raw
        on_line = c < TH

        # 2. MEASURE: wheel speeds in steps per second
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt
        left_prev, right_prev, t_prev = left, right, t_now

        # 3. DECIDE (follow the left edge of the line):
        #    black -> turn left: stop left wheel, right wheel drives
        #    white -> turn right: stop right wheel, left wheel drives
        drive_left = not on_line
        drive_right = on_line

        # 4. CALCULATE: PID only for the wheel that drives.
        #    The stopped wheel's PID is paused, so its integral is frozen.
        #    When it starts again it continues from its last output.
        if drive_left:
            if not pid_left.auto_mode:
                pid_left.set_auto_mode(True, last_output=left_out)
            left_out = pid_left(left_speed)
            left_cmd = left_out
        else:
            pid_left.auto_mode = False
            left_cmd = 0

        if drive_right:
            if not pid_right.auto_mode:
                pid_right.set_auto_mode(True, last_output=right_out)
            right_out = pid_right(right_speed)
            right_cmd = right_out
        else:
            pid_right.auto_mode = False
            right_cmd = 0

        # 5. ACT
        robot.value = (left_cmd, right_cmd)

        print(f"c={c:5d} {'BLACK' if on_line else 'white'}  "
              f"left {left_speed:5.0f} ({left_cmd:.2f})  right {right_speed:5.0f} ({right_cmd:.2f})")

finally:
    # Always runs, even after Ctrl+C: stop motors, sleep driver, stop decoder.
    robot.stop()
    slp.off()
    enc.close()