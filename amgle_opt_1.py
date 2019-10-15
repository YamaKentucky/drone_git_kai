import serial
import navio.util
import navio.mpu9250
import navio.adc
import time


global DD

ser = serial.Serial("/dev/ttyAMA0" , 9600,timeout = None)
data = None
DD = None
print ("Start!!")

while 1:
    data = ser.readline()
    DD=data.split(",")


    DD[4]= DD[4].strip('\r\n')

    height  = int(DD[0]) * 0.001
    deltaX = float(DD[1])*0.001
    deltaY = float(DD[2])*0.001
    deltaX_sum = float(DD[3])*0.001
    deltaY_sum = float(DD[4])*0.001

    print "{:7.3f}".format(height),",","{:7.3f}".format(deltaX),",","{:7.3f}".format(deltaY),",","{:7.3f}".format(deltaX_sum),",","{:7.3f}".format(deltaY_sum)
    time.sleep(0.0001)