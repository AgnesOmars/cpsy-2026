import time
import math
from gpiozero import RotaryEncoder

ppr = 700      # guess, from your earlier hand test (~690)
tstop = 20     # run for 20 s
tsample = 0.02
tdisp = 0.5

left = RotaryEncoder(6, 5, max_steps=0)
right = RotaryEncoder(17, 27, max_steps=0)

tprev = 0
tcurr = 0
tstart = time.perf_counter()

print('Running code for', tstop, 'seconds ...')
print('(Turn each wheel exactly one revolution forward.)')
while tcurr <= tstop:
    time.sleep(tsample)
    tcurr = time.perf_counter() - tstart
    left_angle = 360 / ppr * left.steps
    right_angle = 360 / ppr * right.steps
    if math.floor(tcurr / tdisp) - math.floor(tprev / tdisp) == 1:
        print(f"Left: {left.steps} steps, {left_angle:.0f} deg   "
              f"Right: {right.steps} steps, {right_angle:.0f} deg")
    tprev = tcurr

print('Done.')
left.close()
right.close()
