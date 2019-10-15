import time
import VL53L0X

# Create a VL53L0X object
tof = VL53L0X.VL53L0X()

# Start ranging
tof.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)

timing = tof.get_timing()
if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))

while 1:
    distance = tof.get_distance()
    if (distance > 0 and distance < 2000):
        d = distance
        
    print ("%d mm" % (d))
    time.sleep(0.1)
#    time.sleep(timing/1000000.00)

tof.stop_ranging()
