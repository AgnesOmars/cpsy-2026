from gpiozero import DigitalOutputDevice, DigitalInputDevice, Motor, Robot
import time
import numpy as np

T = 4          # period of the sine wave (s)
u0 = 0.3       # amplitude
tstop = 4      # run one full period (s)
tsample = 0.05 # time between samples (s)
PPR = 693      # check again with the new counting, see below

slp = DigitalOutputDevice('GPIO26')
slp.on()
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))

# Only one signal per wheel
left_pin = DigitalInputDevice(6, pull_up=True)
right_pin = DigitalInputDevice(17, pull_up=True)

left_count = 0
right_count = 0
direction = 1  # +1 forward, -1 backward, set from the motor command

def left_pulse():
    global left_count
    left_count += direction

def right_pulse():
    global right_count
    right_count += direction

left_pin.when_activated = left_pulse
right_pin.when_activated = right_pulse

t = []
u_out = []
theta_left = []
theta_right = []

tcurr = 0
tstart = time.perf_counter()

try:
    while tcurr <= tstop:
        time.sleep(tsample)
        tcurr = time.perf_counter() - tstart

        u = u0 * np.sin(2 * np.pi / T * tcurr)
        direction = 1 if u >= 0 else -1
        robot.value = (u, u)

        t.append(tcurr)
        u_out.append(u)
        theta_left.append(360 / PPR * left_count)
        theta_right.append(360 / PPR * right_count)
finally:
    robot.stop()
    slp.off()

w_left = 60 / 360 * np.gradient(theta_left, t)
w_right = 60 / 360 * np.gradient(theta_right, t)

np.savetxt('sine_test.csv',
           np.column_stack([t, u_out, theta_left, theta_right, w_left, w_right]),
           delimiter=',', fmt='%.3f',
           header='time,output,left_angle,right_angle,left_rpm,right_rpm',
           comments='')
print('Done. Results in sine_test.csv')
