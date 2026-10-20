import time, board, adafruit_tcs34725
from gpiozero import Robot

s = adafruit_tcs34725.TCS34725(board.I2C())
s.integration_time = 24   # ms, short = faster loop
s.gain = 4

robot = Robot(left=(13, 19), right=(12, 18))  # check pin order against your lab 9 script
SPEED = 0.3   # lowest speed that still turns reliably
TH = 2500    # midpoint from your measurements

try:
    while True:

        r, g, b, c = s.color_raw
        if c < TH:                     # dark = on line -> turn left
            robot.value = (0, SPEED)
        else:                          # bright = floor -> turn right
            robot.value = (SPEED, 0)
finally:
    robot.stop()