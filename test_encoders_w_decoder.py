from time import sleep
from encoder import Encoders

enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

try:
    while True:
        left, right = enc.read()
        print("left:", left, " right:", right)
        sleep(0.2)
finally:
    enc.close()
