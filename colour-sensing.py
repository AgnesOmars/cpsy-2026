import time
import board
import busio
import adafruit_tcs34725
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)
oled = SSD1306_I2C(128, 64, i2c, addr=0x3C)

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

    image = Image.new("1", (128, 64))
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), f"R:{r} G:{g} B:{b}", fill=255)
    draw.text((0, 16), name, fill=255)
    oled.image(image)
    oled.show()

    time.sleep(1)
