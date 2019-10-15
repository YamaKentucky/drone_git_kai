import serial
import time

ser = serial.Serial("/dev/ttyAMA0" , 115200 , timeout=10)

while 1:
    data = ser.readline()
    print data
    time.sleep(1)
