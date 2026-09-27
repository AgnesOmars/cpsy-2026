#! /usr/bin/python3

from gpiozero import LED
from time import sleep

ledPin = 24
led = LED(ledPin)

for _ in range(10):
    led.on()
    sleep(0.5)
    led.off()
    sleep(0.5)
