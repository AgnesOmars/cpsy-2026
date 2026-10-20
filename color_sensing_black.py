import time, board, adafruit_tcs34725, busio

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)

sensor.integration_time = 24
sensor.gain = 4

while True:
    r, g, b, c = sensor.color_raw
    print(r, g, b, c)
    time.sleep(0.2)