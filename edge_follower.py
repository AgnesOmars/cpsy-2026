import time, board, adafruit_tcs34725
from gpiozero import Robot, DigitalOutputDevice, Motor

s = adafruit_tcs34725.TCS34725(board.I2C())
s.integration_time = 24   # ms, short = faster loop
s.gain = 4

slp = DigitalOutputDevice('GPIO26')
slp.on()
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))
SPEED = 0.1   # lowest speed that still turns reliably
TH = 2500    # midpoint from your measurements

try:
    while True:
        r, g, b, c = s.color_raw
        print(c, "line" if c < TH else "floor")
        if c < TH:                     # dark = on line -> turn left
            robot.value = (0, SPEED)
        else:                          # bright = floor -> turn right
            robot.value = (SPEED, 0)
finally:
    robot.stop()
