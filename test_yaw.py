#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-showdata.py -> Script that shows data from a vehicle. DroneKit 2.0 related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "2.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time
from dronekit import connect, VehicleMode
#import modules.UDPserver as udp
from DronePilot.modules.utils import *
from DronePilot.modules.pixVehicle import *
import datetime,csv
del_t = 0.0166

# Connection to the vehicle
# SITL via TCP
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# SITL via UDP 
vehicle = connect('/dev/ttyACM0', wait_ready=True)
# Real vehicle via Serial Port 
#vehicle = connect('/dev/tty.usbmodem1', wait_ready=False)

def calibration():
    i =0
    yaw_cal = 0
    while i <100:
        yaw_cal = yaw_cal + vehicle.attitude.yaw
        i = i +1
        time.sleep(0.05)
    
    bias_yaw = yaw_cal / 100
    print(bias_yaw)
    time.sleep(1)
    return bias_yaw

def print_yaw(bias_yaw):
    while True:
        start =time.time()
        yaw_true = vehicle.attitude.yaw - bias_yaw
        yaw_deg = yaw_true * 180 / pi
        print(yaw_true)
        raw = "{:6.3f}".format(time.time()),"{:6.3f}".format(yaw_true),"{:6.3f}".format(yaw_deg)
        logger.writerow(raw)

        elapsed_time = time.time() - start

        if elapsed_time < del_t:
            sleep_time = del_t - elapsed_time
            time.sleep(sleep_time)

	

if __name__ == "__main__":
    bias_yaw = calibration()
    st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
    f = open("Yaw_test"+st, "w")
    logger = csv.writer(f)
    print_yaw(bias_yaw)
    

vehicle.close()

''' 
param set SR2_EXTRA1 50
param set SR2_POSITION 50
param set SR2_RAW_CTRL 50
param set SR2_RC_CHAN 50

''' 
