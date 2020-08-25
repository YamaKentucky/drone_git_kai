import threading, serial, time

ser = serial.Serial("/dev/ttyAMA0", 115200)

def uart():
    global DD, OPT, height, deltaX, deltaY, deltaX_sum, deltaY_sum, time_lap

    DD = [0, 0, 0, 0]
    DD_b = [0, 0, 0, 0]
    data = None
    OPT = None
    height = deltaX = deltaY = deltaX_sum = deltaY_sum = time_lap = time_b = 0

    while 1:
        data = ser.readline()
        OPT = data.split(",")
        OPT[8] = OPT[8].strip('\r\n')
        height  = int(OPT[0]) * 0.001 #m
        deltaX = -float(OPT[2])  * 0.001  #m/s
        deltaY = -float(OPT[1])  * 0.001  #m/s
        deltaX_sum_ar = float(OPT[3])  * 0.001 ##m
        deltaY_sum_ar = float(OPT[4])  * 0.001 ##m
        DD_b[0] = int(OPT[5])
        DD_b[1] = int(OPT[6])
        DD_b[2] = int(OPT[7])
        DD_b[3] = int(OPT[8])

        for i in range(4):
            if(DD_b[i] < 2000):
                DD[i] = DD_b[i]

        time_lap = time.time() - time_b
        time_b = time.time()
        deltaX_sum = deltaX_sum + deltaX * 0.042  ##m
        deltaY_sum = deltaY_sum + deltaY * 0.042  ##m 
        

def startup():
    global DD_old
    DD_old = 0
    th1 = threading.Thread(target = uart)
    th1.start()

    print('SYSTEM ALL GREEN')
    time.sleep(2)
    DD_old = DD
    print  "DD_old[0]: {:5.0f}, DD_old[1]: {:5.0f}, DD_old[2]: {:5.0f}, DD_old[3]: {:5.0f}".format(DD_old[0], DD_old[1], DD_old[2], DD_old[3])
    time.sleep(2)


if __name__ == "__main__":
    startup()
    
    while True:
       print "Height:{:+5.3f}, deltaX:{:+5.3f}, deltaY:{:+5.3f}, DD[0]:{:5.0f}, DD[1]:{:5.0f}, DD[2]:{:5.0f}, DD[3]:{:5.0f}" .format(height, deltaX, deltaY, DD[0], DD[1], DD[2], DD[3])
       time.sleep(0.042)