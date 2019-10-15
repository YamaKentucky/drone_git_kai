
import time, datetime, csv, threading
from math import *
from dronekit import connect, VehicleMode
from DronePilot.modules.utils import *
from DronePilot.modules.pixVehicle import *
import GPS_test_function

def control():
    while True:
        print("control")
	print (GPS_test_function.x_new[:,0][0])
        time.sleep(0.05)



if __name__ == "__main__":

    try:
        th1 = threading.Thread(target=control)
        th1.start()
        GPS_test_function.pos_estimate()   
        

    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        #vehicle.close()
        exit()
