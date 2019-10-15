from DronePilot.modules.utils import *
from DronePilot.modules.pixVehicle import *
import datetime,csv
from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/ttyACM0', wait_ready=True)

while True:
	
	if vehicle.channels['6'] >1000:
		print('Arm')
		if vehicle.channels['5'] > 1000:
			 print ('Auto')
		
		else:
			vehicle.channels.overrides = {}
			print ('Manual')


	else:
		print('Disarm')
		vehicle.channels.overrides={"3":1104}

	print "%s" % vehicle.channels
	
	

	time.sleep(0.1)

vehicle.close()
