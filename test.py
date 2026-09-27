from gpiozero import DigitalInputDevice
from time import sleep

a = DigitalInputDevice(5, pull_up=True)
b = DigitalInputDevice(6, pull_up=True)

while True:
    print(a.value, b.value)
    sleep(0.05)
