import time
from math import *
import numpy as np
from dronekit import connect, VehicleMode
from DronePilot.modules.utils import *
import GPS_test_function
import threading
import serial
import VL53L0X

ser = serial.Serial("/dev/ttyAMA0" , 115200)
tof = VL53L0X.VL53L0X()
tof.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)

global DD
global hh

def compute():
    #global dd1
    #global dd2
    #global dd3

    global DD
    global hh

    data = None
    DD = None

    while 1:

        data = ser.readline()
        DD=data.split(",")
        #DD[2]= DD[2].strip('\r\n')

        #dd1 = int(DD[0])*0.01*0.8718 - 0.1834
        #dd2 = int(DD[1])*0.01*0.8587 - 0.1882
        #dd3 = int(DD[2])*0.01*0.8821 - 0.1896
        #print dd1,dd2,dd3

        h_distance =  int(tof.get_distance())
        if (h_distance > 0 and h_distance < 2000):
            hh = h_distance

        time.sleep(0.05)

# def compute():
#     while True:
#         print("compute")
#         time.sleep(0.05)



# def control():
#     while True:
#         print("control")
#         time.sleep(0.05)






if __name__ == "__main__":
    try:
        
        th1 = threading.Thread(target=compute)
        th1.start()
        #control()
        GPS_test_function.pos_estimate()

    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        vehicle.close()
        exit()
