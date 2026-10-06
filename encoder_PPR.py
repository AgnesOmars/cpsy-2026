import time
import math
from encoder import Encoders

ppr = 700       # guess, from your earlier hand test (~690)
tstop = 12      # run for 12 s
tsample = 0.02
tdisp = 0.5

enc = Encoders(left_pins=(5, 6), right_pins=(27, 17))

tprev = 0
tcurr = 0
tstart = time.perf_counter()

print('Running code for', tstop, 'seconds ...')
print('(Turn each wheel exactly one revolution forward.)')
try:
    while tcurr <= tstop:
        time.sleep(tsample)
        tcurr = time.perf_counter() - tstart
        left_steps, right_steps = enc.read()
        left_angle = 360 / ppr * left_steps
        right_angle = 360 / ppr * right_steps
        if math.floor(tcurr / tdisp) - math.floor(tprev / tdisp) == 1:
            print(f"Left: {left_steps} steps, {left_angle:.0f} deg   "
                  f"Right: {right_steps} steps, {right_angle:.0f} deg")
        tprev = tcurr
finally:
    print('Done.')
    enc.close()
