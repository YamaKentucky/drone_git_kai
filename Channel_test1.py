from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/ttyACM0', wait_ready=True)

while True:
    if vehicle.channels['5'] > 1000:
        print("Auto")

        print ("Channel values from RC Tx:", vehicle.channels)
        print ("Set Ch4 override to 1400")
        vehicle.channels.overrides = {'4':1300}
        time.sleep(2)

        # print ("Channel values from RC Tx:", vehicle.channels)
        # print ("Set Ch2 override to 1700")
        # vehicle.channels.overrides = {'2':1700}
        # time.sleep(3)

        print ("Channel values from RC Tx:", vehicle.channels)
        print ("Set Ch4 override to 1600")
        vehicle.channels.overrides = {'4':1700}
        time.sleep(2)


    else:
        print("Manual")
        print ("Channel values from RC Tx:", vehicle.channels)
        print ("Clear all overrides")
        vehicle.channels.overrides = {}
        time.sleep(1)

