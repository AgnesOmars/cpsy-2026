import time
import board
import adafruit_tcs34725
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

# ---------------------------------------------------------------
# Settings: change these between tests
# ---------------------------------------------------------------
#MAX_SPEED = 550    # Speed on straights (steps/s). 692 steps = one wheel rotation.
MIN_SPEED = MAX_SPEED = 250    # Speed in sharp turns
SLOW_START = 0.15  # |error| below this counts as "straight" -> MAX_SPEED
SLOW_FULL = 0.40   # |error| at or above this -> MIN_SPEED
RELAX = 0.8        # How fast the robot "forgets" a turn (per second).
                   # Lower = stays slow longer after a turn before speeding up.

K = 0.5            # Steering strength near the edge
PIVOT = 0.4        # If |error| is above this, pivot
PIVOT_INNER = -0.2 # Inner wheel power during a pivot. 0 = old behaviour,
                   # negative = turns tighter (almost in place)

LOST_TIME = 0.6    # Seconds fully on black or white before we say "line lost"
SPIN = 0.35        # Motor power when searching for the line (spin in place)
SWEEP_TIME = 0.4   # First search sweep length in seconds (doubles each time)

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

pid_left = PID(KP, KI, KD, setpoint=MIN_SPEED, sample_time=None, output_limits=(0, 1))
pid_right = PID(KP, KI, KD, setpoint=MIN_SPEED, sample_time=None, output_limits=(0, 1))

try:
    left_prev, right_prev = enc.read()
    t_prev = time.perf_counter()

    turn_level = 0.0   # "How much are we turning lately?" 0..0.5
    off_time = 0.0     # How long we've been fully off the edge
    searching = False
    search_dir = 0     # -1 = spin left, +1 = spin right
    sweep = SWEEP_TIME
    sweep_end = 0.0
    left_was_forced = right_was_forced = False

    while True:
        # 1. SENSE
        r, g, b, c = sensor.color_raw
        error = (c - TH) / (WHITE - BLACK)
        abs_err = min(abs(error), 0.5)

        # 2. MEASURE
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt
        left_prev, right_prev, t_prev = left, right, t_now

        # 3a. SPEED: jump up instantly when turning, fade slowly afterwards
        turn_level = max(abs_err, turn_level - RELAX * dt)
        x = (turn_level - SLOW_START) / (SLOW_FULL - SLOW_START)
        x = min(max(x, 0.0), 1.0)
        base = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * x

        # 3b. LOST-LINE CHECK
        if abs(error) > PIVOT:
            off_time += dt
        else:
            off_time = 0.0
            searching = False

        left_force = right_force = None   # None = let the PID drive that wheel

        if off_time > LOST_TIME:
            # SEARCH: the normal pivot failed, so the line is probably on the
            # other side (we crossed it). Search the opposite way first, then
            # sweep back and forth with longer and longer sweeps.
            if not searching:
                searching = True
                search_dir = -1 if error > 0 else +1   # on white we pivoted right -> now try left
                sweep = SWEEP_TIME
                sweep_end = t_now + sweep
            elif t_now > sweep_end:
                search_dir = -search_dir
                sweep *= 2
                sweep_end = t_now + sweep
            left_force = SPIN * search_dir
            right_force = -SPIN * search_dir

        elif error < -PIVOT:              # on black -> pivot left
            left_force = PIVOT_INNER
            pid_right.setpoint = base
        elif error > PIVOT:               # on white -> pivot right
            right_force = PIVOT_INNER
            pid_left.setpoint = base
        else:                             # near the edge -> gentle steering
            pid_left.setpoint = base * (1 + K * error)
            pid_right.setpoint = base * (1 - K * error)

        # 4. CALCULATE
        # When a wheel goes from "forced" back to PID, reset its PID so old
        # integral doesn't give a jerk.
        if left_force is None:
            if left_was_forced:
                pid_left.reset()
            left_cmd = pid_left(left_speed)
        else:
            left_cmd = left_force
        if right_force is None:
            if right_was_forced:
                pid_right.reset()
            right_cmd = pid_right(right_speed)
        else:
            right_cmd = right_force
        left_was_forced = left_force is not None
        right_was_forced = right_force is not None

        # 5. ACT
        robot.value = (left_cmd, right_cmd)

        # print(f"c={c:5d} err={error:+.2f} base={base:4.0f} "
        #       f"{'SEARCH' if searching else ''} L={left_cmd:+.2f} R={right_cmd:+.2f}")

finally:
    robot.stop()
    slp.off()
    enc.close()