import time
from math import *
import numpy as np
from dronekit import connect, VehicleMode
from DronePilot.modules.utils import *
import GPS_test_function

update_rate = 0.02
vehicle_weight = 0.84 # Kg
u0 = 1000 # Zero throttle command
uh = 1360 # Hover throttle command
kt = vehicle_weight * g / (uh-u0)
ky = 500 / np.pi # Yaw controller gain
vehicle = connect('/dev/ttyACM0', wait_ready=True)

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':1.0} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

rcCMD = [1515,1507,1515,1100]
desiredRoll = 1515
desiredPitch = 1507
desiredThrottle = 1100
desiredYaw = 1515

p_gains = {'kp': 2.61, 'ki':0.57, 'kd':3.41, 'iMax':2, 'filter_bandwidth':50} # Position Controller gains
h_gains = {'kp': 4.64, 'ki':1.37, 'kd':4.55, 'iMax':2, 'filter_bandwidth':50} # Height Controller gains
y_gains = {'kp': 1.0,  'ki':0.0,  'kd':0.0,  'iMax':2, 'filter_bandwidth':50} # Yaw Controller gains

rollPID =   PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
rPIDvalue = 0.0
pitchPID =  PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
pPIDvalue = 0.0
heightPID = PID(h_gains['kp'], h_gains['ki'], h_gains['kd'], h_gains['filter_bandwidth'], 0, 0, update_rate, h_gains['iMax'], -h_gains['iMax'])
hPIDvalue = 0.0
yawPID =    PID(y_gains['kp'], y_gains['ki'], y_gains['kd'], y_gains['filter_bandwidth'], 0, 0, update_rate, y_gains['iMax'], -y_gains['iMax'])
yPIDvalue = 0.0

# Filters initialization
f_yaw   = low_pass(20,update_rate)
f_pitch = low_pass(20,update_rate)
f_roll  = low_pass(20,update_rate)


def control():
    global rcCMD
    global rollPID, pitchPID, heightPID, yawPID
    global desiredPos, currentPos
    global desiredRoll, desiredPitch, desiredThrottle
    global rPIDvalue, pPIDvalue, yPIDvalue
    global f_yaw, f_pitch, f_roll
    global ky

    while True:
        current = time.time()
        elapsed = 0

        GPS_test_function.pos_estimate()
        # Update current position of the vehicle
        currentPos['x'] = GPS_test_function.x_new[:,0][0]
        currentPos['y'] = GPS_test_function.x_new[:,0][1]
        currentPos['z'] = GPS_test_function.x_new[:,0][2]
        

        heading = f_yaw.update(GPS_test_function.x_new[:,0][8])
        rPIDvalue = rollPID.update(desiredPos['y'] - currentPos['y'])
        pPIDvalue = pitchPID.update(desiredPos['x'] - currentPos['x'])
        hPIDvalue = heightPID.update(desiredPos['z'] - currentPos['z'])
        yPIDvalue = yawPID.update(0.0 - GPS_test_function.x_new[:,0][8])

        sinYaw = sin(heading)
        cosYaw = cos(heading)

            # Conversion from desired accelerations to desired angle commands
        desiredRoll  = toPWM(degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / g) ),1)
        desiredPitch = toPWM(degrees( (pPIDvalue * cosYaw - rPIDvalue * sinYaw) * (1 / g) ),1)
        # Change this one to correspondent attitude commands
        desiredThrottle = ((hPIDvalue + g) * vehicle_weight) / (cos(f_pitch.update(radians(vehicle.attitude['angx'])))*cos(f_roll.update(radians(vehicle.attitude['angy']))))
        desiredThrottle = (desiredThrottle / kt) + u0
        desiredYaw = 1500 - (yPIDvalue * ky)  

        # Limit commands for safety
        if udp.message[7] == 1:
            rcCMD[0] = limit(desiredRoll,1000,2000)
            rcCMD[1] = mapping(limit(desiredPitch,1000,2000),1000,2000,2000,1000) # Mapping to invert channel (used on pix joystick)
            rcCMD[2] = limit(desiredYaw,1000,2000)
            rcCMD[3] = limit(desiredThrottle,968,2000)
            mode = 'Auto'
            rcCMD = [limit(n,1000,2000) for n in rcCMD]
            vehicle.channels.overrides = { "1" : rcCMD[0], "2" : rcCMD[1], "3" : rcCMD[3], "4" : rcCMD[2] }
        else:
            # Prevent integrators/derivators to increase if they are not in use
            #rollPID.resetIntegrator()
            #pitchPID.resetIntegrator()
            #heightPID.resetIntegrator()
            #yawPID.resetIntegrator()
            vehicle.channels.overrides = {}
            mode = 'Manual'
        

        print(mode,currentPos['x'],currentPos['y'],currentPos['z'])

        while elapsed < update_rate:
                    elapsed = time.time() - current



if __name__ == "__main__":
    try:
        control()
    
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        vehicle.close()
        exit()



