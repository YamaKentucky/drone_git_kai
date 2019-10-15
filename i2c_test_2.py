import smbus
import time
import datetime, csv
import threading, serial

adress = 0x04
bus = smbus.SMBus(1)


def uwb():
    global DD, DD_b
    DD = [0,0,0,0]
    DD_b = [0,0,0,0]
  
    while True:
        result = bus.read_i2c_block_data(adress,0,8)
        time.sleep(0.03)

        DD_b[0] = result[1] << 8 | result[0]
        DD_b[1] = result[3] << 8 | result[2]
        DD_b[2] = result[5] << 8 | result[4]
        DD_b[3] = result[7] << 8 | result[6]

        for i in range(4):
            if(DD_b[i] < 2000):
                DD[i] = DD_b[i]

        time.sleep(0.03)


def optical():

    global OPT, height, deltaX, deltaY, deltaX_sum, deltaY_sum, time_lap
    data = None
    OPT = None
    time_b = 0
    deltaX_sum = 0
    deltaY_sum = 0

    while 1:
        data = ser.readline()
        OPT = data.split(",")
        OPT[4] = OPT[4].strip('\r\n')
        height  = int(OPT[0]) * 0.001 #m
        deltaX = -float(OPT[2])  * 0.001  #m/s
        deltaY = -float(OPT[1])  * 0.001  #m/s
        deltaX_sum_ar = float(OPT[3])  * 0.001
        deltaY_sum_ar = float(OPT[4])  * 0.001
        time_lap = time.time() - time_b
        time_b = time.time()
        deltaX_sum = deltaX_sum + deltaX * 0.042
        deltaY_sum = deltaY_sum + deltaY * 0.042 

        #time.sleep(0.042)


ser = serial.Serial("/dev/ttyAMA0" , 115200)

th1 = threading.Thread(target = uwb)
th1.start()

th2 = threading.Thread(target = optical)
th2.start()

print('SYSTEM ALL GREEN')
time.sleep(2)



if __name__ == '__main__':

    while True:
        print "{:+7.3f}".format(height), "{:+7.3f}".format(deltaX), "{:+7.3f}".format(deltaY), "{:5.0f}".format(DD[0]), "{:5.0f}".format(DD[1]), "{:5.0f}".format(DD[2]), "{:5.0f}".format(DD[3])
        #uwb()
        #print (DD[0], DD[1], DD[2], DD[3])
        time.sleep(0.042)
        
        # result = bus.read_i2c_block_data(adress,0,8)
        # time.sleep(0.03)

        # num[0] = result[1] << 8 | result[0]
        # num[1] = result[3] << 8 | result[2]
        # num[2] = result[5] << 8 | result[4]
        # num[3] = result[7] << 8 | result[6]

        # for i in range(4):
        #     if(num[i] < 2000):
        #         num_true[i] = num[i]

        # print(num_true[0],num_true[1],num_true[2],num_true[3])
        # time.sleep(0.042)