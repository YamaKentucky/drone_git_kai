import time
import serial
import numpy as np
import datetime,csv

anchor1 = np.array([-2.0,  2.306, 2.227],dtype=float)
anchor2 = np.array([ 2.0,  2.306, 2.227],dtype=float)
anchor3 = np.array([ 2.0, -2.306, 2.227],dtype=float)
anchor4 = np.array([-2.0, -2.306, 2.227],dtype=float)

ser = serial.Serial("/dev/ttyAMA0" , 115200)

def getDD():
    global DD
    DD = None

    while True:
        data = ser.readline()
        DD=data.split(",")
        DD[3]= DD[3].strip('\r\n')
        print ("DD1 = {0:s}".format(DD[0]),"DD2 = {0:s}".format(DD[1]),"DD3 = {0:s}".format(DD[2]),"DD4 = {0:s}".format(DD[3]))
        row = (DD[0],DD[1],DD[2],DD[3])
        logger.writerow(row)
        

st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
f = open("UWB__DD_test"+st, "w")
logger = csv.writer(f)

print u'start? (y/n)'

start_flag = raw_input('>>')

if start_flag != "y":
    print('end!!')


while start_flag == "y":
    print("Start!!")
    getDD()