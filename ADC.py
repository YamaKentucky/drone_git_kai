import sys, time

import navio.adc
import navio.util

navio.util.check_apm()

adc = navio.adc.ADC()
results = [0] * adc.channel_count

while (True):
    
    results1 = adc.read(4)
    results2 = adc.read(0)
#    s += 'A{0}: {1:6.4f}V '.format(i, results[5] / 1000)
    
#    duty=0.001*int(1000+900*results[5]/5000)
    print results1#-0.0658*results1+171.9 , results2
    time.sleep(0.1)
