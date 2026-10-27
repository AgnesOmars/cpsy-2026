import time
import board
import adafruit_tcs34725
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

# ---------------------------------------------------------------
# Settings: change these between tests
# ---------------------------------------------------------------
#FAST = 550         # Speed on straights (steps/s)
SLOW = FAST = 250         # Speed in turns
K = 0.5            # Steering strength near the edge
PIVOT = 0.4        # If |error| is above this, pivot
PIVOT_INNER = -0.2 # Inner wheel power when pivoting (0 = stop, negative = turn tighter)
LOST_TIME = 0.6    # Seconds off the edge before we spin the other way
KP = 0.0008
KI = 0.002
KD = 0.0

BLACK = 300
WHITE = 4700
TH = (BLACK + WHITE) / 2

# ---------------------------------------------------------------
# Hardware setup
# ---------------------------------------------------------------
sensor = adafruit_tcs34725.TCS34725(board.I2C())
sensor.integration_time = 24
sensor.gain = 4

slp = DigitalOutputDevice('GPIO26')
slp.on()

robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))
enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

pid_left = PID(KP, KI, KD, setpoint=SLOW, sample_time=None, output_limits=(0, 1))
pid_right = PID(KP, KI, KD, setpoint=SLOW, sample_time=None, output_limits=(0, 1))

try:
    left_prev, right_prev = enc.read()
    t_prev = time.perf_counter()
    turning = 0.0    # 0 = driving straight, 1 = turning hard
    off_time = 0.0   # how long we've been fully on black or white

    while True:
        # 1. SENSE
        r, g, b, c = sensor.color_raw
        error = (c - TH) / (WHITE - BLACK)

        # 2. MEASURE
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt
        left_prev, right_prev, t_prev = left, right, t_now

        # 3. SPEED: brake at once when turning, speed up slowly after
        turning = max(min(abs(error) / PIVOT, 1), turning - dt)
        base = FAST - (FAST - SLOW) * turning

        # 4. DECIDE
        if abs(error) > PIVOT:
            off_time += dt
        else:
            off_time = 0

        left_cmd = pid_left(left_speed)
        right_cmd = pid_right(right_speed)

        if abs(error) <= PIVOT:           # near the edge -> gentle steering
            pid_left.setpoint = base * (1 + K * error)
            pid_right.setpoint = base * (1 - K * error)
        else:
            turn_left = error < 0         # on black -> turn left, on white -> turn right
            if off_time > LOST_TIME:      # pivot isn't finding the line -> we crossed it,
                turn_left = not turn_left #   so spin the other way
            if turn_left:
                pid_right.setpoint = base
                left_cmd = PIVOT_INNER
                pid_left.reset()
            else:
                pid_left.setpoint = base
                right_cmd = PIVOT_INNER
                pid_right.reset()

        # 5. ACT
        robot.value = (left_cmd, right_cmd)

finally:
    robot.stop()
    slp.off()
    enc.close()