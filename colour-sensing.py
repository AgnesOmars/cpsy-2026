import time
import board
import busio
import adafruit_tcs34725
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)

while True:
    r, g, b = sensor.color_rgb_bytes

    if r > g and r > b:
        name = "red"
    elif g > r and g > b:
        name = "green"
    elif b > r and b > g:
        name = "blue"
    else:
        name = "unknown"

    print(r, g, b, name)

    time.sleep(1)
