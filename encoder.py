from gpiozero import RotaryEncoder
from time import sleep

left = RotaryEncoder(6,5 , max_steps=0)
right = RotaryEncoder(17, 27, max_steps=0)

while True:
    print("left:", left.steps, " right:", right.steps)
    sleep(0.2)
