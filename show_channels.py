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
from DronePilot.modules.utils import *
from DronePilot.modules.pixVehicle import *
import datetime,csv

vehicle = connect('/dev/ttyACM0', wait_ready=True, baud = 921600)

st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
f = open("Logs_Channels"+st, "w")
logger = csv.writer(f)
logger.writerow(('timestamp', 'Cannels'))



while True:
    print "%s" % vehicle.channels 
    row = ("{:6.3f}".format(time.time()), vehicle.channels['3'] )
    logger.writerow(row)
    time.sleep(0.001)

vehicle.close()

