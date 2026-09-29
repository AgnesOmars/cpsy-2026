from gpiozero import DigitalOutputDevice, Motor, Robot, RotaryEncoder
import time
import numpy as np

T = 4          # period of the sine wave (s)
u0 = 0.3       # amplitude, kept low so no steps are missed
tstop = 4      # run one full period (s)
tsample = 0.05 # time between samples (s)
PPR = 693      # average of left (694.8) and right (691.2)

slp = DigitalOutputDevice('GPIO26')
slp.on()
robot = Robot(right=Motor('GPIO12', 'GPIO18'), left=Motor('GPIO19', 'GPIO13'))

left_enc = RotaryEncoder(6, 5, max_steps=0)
right_enc = RotaryEncoder(17, 27, max_steps=0)

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
        robot.value = (u, u)

        t.append(tcurr)
        u_out.append(u)
        theta_left.append(360 / PPR * left_enc.steps)
        theta_right.append(360 / PPR * right_enc.steps)
finally:
    robot.stop()
    slp.off()

# Angular velocity (rpm) from the recorded angles
w_left = 60 / 360 * np.gradient(theta_left, t)
w_right = 60 / 360 * np.gradient(theta_right, t)

np.savetxt('sine_test.csv',
           np.column_stack([t, u_out, theta_left, theta_right, w_left, w_right]),
           delimiter=',', fmt='%.3f',
           header='time,output,left_angle,right_angle,left_rpm,right_rpm',
           comments='')
print('Done. Results in sine_test.csv')
