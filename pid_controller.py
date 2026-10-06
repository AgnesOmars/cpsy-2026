import time
import csv
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

# ---------------------------------------------------------------
# Settings: change these between tests
# ---------------------------------------------------------------
SETPOINT = 2000    # Target speed for both wheels, in encoder steps per second.
                   # 692 steps = one wheel rotation, so 1500 is about 2.2 rotations/s.
KP = 0.0008        # Proportional gain: reacts to the current error.
KI = 0.002          # Integral gain: reacts to error that has built up over time.
KD = 0.0           # Derivative gain: reacts to how fast the error is changing.
RUN_TIME = 5       # How long the test runs, in seconds.
SAMPLE = 0.05      # Time between each measurement and motor update (50 ms).
LOGFILE = f"pid_sp{SETPOINT}_kp{KP}_ki{KI}_kd{KD}.csv"  # Where the measurements are saved for plotting.

# ---------------------------------------------------------------
# Hardware setup
# ---------------------------------------------------------------
# The motor driver has a sleep pin. It must be on for the motors to run.
slp = DigitalOutputDevice('GPIO26')
slp.on()

# The two motors, each with a forward and a backward pin.
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))

# Starts the C++ decoder with our encoder pins and talks to it over UDP.
enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

# ---------------------------------------------------------------
# PID controllers: one per wheel
# ---------------------------------------------------------------
# Each controller compares the measured speed of its wheel with the setpoint
# and calculates a motor power to reduce the difference.
# sample_time=None: calculate a new output every time we call it,
#   using the real time that has passed since the last call.
# output_limits=(0, 1): motor power from 0 (stopped) to 1 (full speed forward).
#   This also stops the integral term from growing forever (integral windup).
pid_left = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))
pid_right = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))

log = []  # Each row: time, setpoint, speeds and motor powers

try:
    # Read the starting counts and time, so we can measure changes from here.
    left_prev, right_prev = enc.read()
    t_prev = time.perf_counter()
    t_start = t_prev

    # Control loop: measure -> calculate -> set motors, repeated every SAMPLE seconds.
    while time.perf_counter() - t_start < RUN_TIME:
        time.sleep(SAMPLE)

        # 1. MEASURE: how far did each wheel turn since last time?
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev  # Actual time passed (sleep is never exactly SAMPLE)

        # Speed = change in steps / change in time  ->  steps per second
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt

        # Remember these values for the next round.
        left_prev, right_prev, t_prev = left, right, t_now

        # 2. CALCULATE: give each PID the measured speed, get back a motor power.
        #    If a wheel is too slow, the output goes up; too fast, it goes down.
        left_out = pid_left(left_speed)
        right_out = pid_right(right_speed)

        # 3. ACT: send the new power to the motors (left, right).
        robot.value = (left_out, right_out)

        # Save and print this step so we can see how the controller behaves.
        t = t_now - t_start
        log.append([round(t, 3), SETPOINT, round(left_speed), round(right_speed),
                    round(left_out, 3), round(right_out, 3)])
        print(f"t={t:5.2f}  left {left_speed:6.0f} ({left_out:.2f})  "
              f"right {right_speed:6.0f} ({right_out:.2f})")

finally:
    # Always runs, even after Ctrl+C or an error:
    # stop the motors, put the driver to sleep and stop the decoder.
    robot.stop()
    slp.off()
    enc.close()

# ---------------------------------------------------------------
# Save the log as a CSV file to plot speed against time in a spreadsheet.
# ---------------------------------------------------------------
with open(LOGFILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "setpoint", "left_speed", "right_speed", "left_out", "right_out"])
    writer.writerows(log)
print("Saved", LOGFILE)
