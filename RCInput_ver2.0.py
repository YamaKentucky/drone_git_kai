import sys, time

import navio.rcinput
import navio.util

navio.util.check_apm()

rcin = navio.rcinput.RCInput()
max = 15 #max angle [deg]

while (True):
#    period0 = rcin.read(0) # yaw
#    period1 = rcin.read(1) # pitch
#    period2 = rcin.read(2) # throttle
#    period3 = rcin.read(3) # roll

#    yaw = -(float(period0) - 1514) * max / 410
#    throt = (float(rcin.read(2))-1104) * 14000 / 820
    roll  =  (float(rcin.read(3))-1514) * 20 / 410
    pitch =  (float(rcin.read(1))-1514) * 20 / 410
    yaw   = -(float(rcin.read(0))-1514) * 20 / 410
    throt = 6500+ (float(rcin.read(2))-1104) * 7500 / 820





    print throt#pitch, roll , yaw
    time.sleep(0.05)
