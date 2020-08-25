import serial
import time
import datetime, csv

ser = serial.Serial("/dev/ttyAMA0" , 115200 , timeout=1)
index = 0

buf_tic = 100
tp = [0]*buf_tic
row = [0]*buf_tic

def log():
    global logger, f
    print ("Logging mode")
    st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
    f = open("./logs/position_opt/serial_opt_test"+st, "w")
    logger = csv.writer(f)
    logger.writerow(("timestamp" ,"Range","deltaX","deltaY","" "DD[0]", "DD[1]", "DD[2]", "DD[3]" ))

log()

while 1:
    try:
        data = ser.readline()
        #DD=data.split(",")
        #DD[3]= DD[3].strip('\r\n')
        #d1 = int(DD[0])*0.01
        #d2 = int(DD[1])*0.01
        #d3 = int(DD[2])*0.01
        #d4 = int(DD[3])*0.01
        print data#d1, d2, d3, d4
        time.sleep(0.05)

        #row[index] = ("{:6.3f}".format(time.time()) , data)
        row[index] = (time.time() , data)
        #tp[index] = time.time()

        if index >= buf_tic-1:
            logger.writerow(row)
            index = 0
        else:
            index = index+1


    except KeyboardInterrupt:
        print "Keyboard Interrupt!! Close this file!!"
        f.close()
        break
        exit()
