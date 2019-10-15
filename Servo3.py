import sys
import time

import navio.pwm
import navio.rcinput
import navio.util

navio.util.check_apm()

rcin = navio.rcinput.RCInput()

pwm1 = navio.pwm.PWM(0)
pwm1.set_period(400)

pwm2 = navio.pwm.PWM(1)
pwm2.set_period(400)

#pwm.set_duty_cycle(SERVO_MIN)
#time.sleep(10)

while (True):

    period2 = rcin.read(2) # throttle
    
    

    throt = (float(period2) - 1104)/820
    
    duty=0.001*int(1000+900*throt) #ms

    pwm1.set_duty_cycle(duty)
    pwm2.set_duty_cycle(duty)

    print int(duty*1000)    

    time.sleep(0.2)
#    pwm.set_duty_cycle(SERVO_MAX)
#    time.sleep(0.2)
