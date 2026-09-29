from gpiozero import DigitalOutputDevice
from time import sleep

slp = DigitalOutputDevice(26)
slp.on()
pin19 = DigitalOutputDevice(19)

pin19.on()
sleep(2)
pin19.off()
slp.off()
