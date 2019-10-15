import serial
import time 

ser = serial.Serial("/dev/ttyAMA0" , 115200 , timeout=10) 

ser.write(b'A')

#while 1:
#    data = ser.readline()
#    DD=data.split(",")
#    DD[2]= DD[2].strip('\r\n')
#    d1 = int(DD[0])*0.01*0.8718 - 0.1834
#    d2 = int(DD[1])*0.01*0.8587 - 0.1882
#    d3 = int(DD[2])*0.01*0.8821 - 0.1896
#    print d1, d2, d3
#    time.sleep(0.05)
