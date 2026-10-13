import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from simple_pid import PID
from gpiozero import Robot, Motor, DigitalOutputDevice
from encoder import Encoders

V_STOP = 2.4081       # voltage at 20 cm - use your own value from Lab 4
V_GO = 2.0         # drive again below this (about 30 cm) - lower than V_STOP
GO_COUNT = 5       # readings in a row below V_GO before driving again
SETPOINT = 1500    # target speed for both wheels, in encoder steps per second
KP = 0.0008
KI = 0.002
KD = 0.0
SAMPLE = 0.05      # time between each measurement and motor update (50 ms)

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.CE0)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

# motor driver sleep pin, motors and encoders
slp = DigitalOutputDevice('GPIO26')
slp.on()
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))
enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

# one PID controller per wheel
pid_left = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))
pid_right = PID(KP, KI, KD, setpoint=SETPOINT, sample_time=None, output_limits=(0, 1))

stopped = False
clear = 0          # how many readings in a row have been below V_GO

try:
    left_prev, right_prev = enc.read()
    t_prev = time.perf_counter()

    while True:
        time.sleep(SAMPLE)

        # 1. MEASURE: distance sensor voltage and wheel speeds
        v = chan0.voltage
        left, right = enc.read()
        t_now = time.perf_counter()
        dt = t_now - t_prev
        left_speed = (left - left_prev) / dt
        right_speed = (right - right_prev) / dt
        left_prev, right_prev, t_prev = left, right, t_now

        # 2. CHECK: obstacle at 20 cm -> stop; path clear long enough -> drive again
        if v >= V_STOP:
            stopped = True
            clear = 0
        elif stopped and v < V_GO:
            clear += 1
            if clear >= GO_COUNT:
                stopped = False
        else:
            clear = 0

        # 3. ACT: stop directly (not through the PID), or let the PID drive
        if stopped:
            robot.stop()
            pid_left.reset()     # forget the built-up integral, so no lurch when driving again
            pid_right.reset()
            print(f'ADC Voltage: {v:.4f}V  STOPPED')
        else:
            robot.value = (pid_left(left_speed), pid_right(right_speed))
            print(f'ADC Voltage: {v:.4f}V  left {left_speed:5.0f}  right {right_speed:5.0f}')

finally:
    robot.stop()
    slp.off()
    enc.close()
