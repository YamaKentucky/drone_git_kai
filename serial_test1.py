import serial
import time 

ser = serial.Serial("/dev/ttyAMA0" , 115200 , timeout=1) 

while 1:
    data = ser.readline()
    #DD=data.split(",")
    #DD[3]= DD[3].strip('\r\n')
    #d1 = int(DD[0])*0.01
    #d2 = int(DD[1])*0.01
    #d3 = int(DD[2])*0.01
    #d4 = int(DD[3])*0.01
    print data#d1, d2, d3, d4
    time.sleep(0.01)
