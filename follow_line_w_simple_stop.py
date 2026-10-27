import time
import board
import busio
import digitalio
import adafruit_tcs34725
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

# ---------------------------------------------------------------
# Settings
# ---------------------------------------------------------------
SPEED = 500        # Target wheel speed in encoder steps per second (692 = 1 rotation)
TH = 3000          # Threshold for finding the line, c
KP = 0.0008        # Wheel PID gains
KI = 0.002
KD = 0.0

V_STOP = 1.94      # Distance sensor voltage at ~30 cm -> stop
V_GO = 1.5         # Drive again below this (lower than V_STOP)
GO_COUNT = 5       # Readings in a row below V_GO before driving again

# ---------------------------------------------------------------
# Hardware setup
# ---------------------------------------------------------------
# Color sensor (I2C)
sensor = adafruit_tcs34725.TCS34725(board.I2C())
sensor.integration_time = 24   # ms, short = faster loop
sensor.gain = 4

# Distance sensor through the MCP3008 ADC (SPI), channel 0
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.CE0)
mcp = MCP.MCP3008(spi, cs)
chan0 = AnalogIn(mcp, MCP.P0)

# Motors and encoders
slp = DigitalOutputDevice('GPIO26')
slp.on()
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))
enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

# One PID per wheel: input = measured speed, output = motor power 0..1
pid_left = PID(KP, KI, KD, setpoint=SPEED, sample_time=None, output_limits=(0, 1))
pid_right = PID(KP, KI, KD, setpoint=SPEED, sample_time=None, output_limits=(0, 1))

def brake():
    # Both motor pins high = active brake (stops faster than just cutting power)
    for m in (robot.left_motor, robot.right_motor):
        m.forward_device.value = 1
        m.backward_device.value = 1

try:
    left_prev, right_prev = enc.read()
    t_prev = time.perf_counter()
    stopped = False
    clear = 0          # how many readings in a row have been below V_GO
    loop = 0

    while True:
        # 1. SENSE: line or floor? obstacle ahead?
        r, g, b, c = sensor.color_raw
        loop += 1
        if loop % 3 == 0:
            v = chan0.voltage

        # 2. MEASURE: wheel speeds in steps per second
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt
        left_prev, right_prev, t_prev = left, right, t_now

        # 3. CHECK OBSTACLE: too close -> stop; clear long enough -> drive again
        if v >= V_STOP:
            stopped = True
            clear = 0
        elif stopped and v < V_GO:
            clear += 1
            if clear >= GO_COUNT:
                stopped = False
        else:
            clear = 0

        # 4. DECIDE + CALCULATE + ACT
        if stopped:
            brake()
            pid_left.reset()               # no lurch when driving again
            pid_right.reset()
        elif c < TH:                       # dark = on line -> turn left
            pid_left.reset()               # stopped wheel starts fresh next time
            robot.value = (-0.1, pid_right(right_speed))
        else:                              # bright = floor -> turn right
            pid_right.reset()
            robot.value = (pid_left(left_speed), 0)

        # print(f"c={c:5d} v={v:.2f} {'STOPPED' if stopped else ''}")

finally:
    robot.stop()
    slp.off()
    enc.close()