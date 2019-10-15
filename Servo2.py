import sys
import time

import navio.pwm
import navio.adc
import navio.util

navio.util.check_apm()

adc = navio.adc.ADC()
results = [0] * adc.channel_count

PWM_OUTPUT = 0
SERVO_MIN = 1.000 #ms
SERVO_MAX = 1.500 #ms

pwm = navio.pwm.PWM(PWM_OUTPUT)
pwm.set_period(400)
#pwm.set_duty_cycle(SERVO_MIN)
#time.sleep(10)

while (True):

    results[5] = adc.read(5)
    duty=0.001*int(1000+900*results[5]/5000) #ms

    pwm.set_duty_cycle(duty)
    print duty    

    time.sleep(0.1)
#    pwm.set_duty_cycle(SERVO_MAX)
#    time.sleep(0.2)
