import time
from dronekit import connect, VehicleMode
#import modules.UDPserver as udp
from DronePilot.modules.utils import *
from DronePilot.modules.pixVehicle import *

# Connection to the vehicle
# SITL via TCP
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# SITL via UDP 
vehicle = connect('/dev/ttyACM0', wait_ready=True,baud=921600)
# Real vehicle via Serial Port 
#vehicle = connect('/dev/tty.usbmodem1', wait_ready=False)

while True:
	print "%s" % vehicle.attitude #SR2_EXTRA1
	print "%s" % vehicle.velocity #SR2_POSITION
	print "%s" % vehicle.channels #SR2_RC_CHAN
	print "Altitude (global frame): %s" % vehicle.location.global_frame.alt
	print "Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt
	print "%s" % vehicle.mode.name
	#print "%s" % udp.message
	time.sleep(0.01)

vehicle.close()
