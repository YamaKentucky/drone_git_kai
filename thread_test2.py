import threading 
import time 
import serial

global d1
global d2
global d3

d1 = 0
d2 = 0
d3 = 0

ser = serial.Serial("/dev/ttyAMA0" , 115200 , timeout=10)

def compute():
    global d1
    global d2
    global d3

    while 1:
        
        data = ser.readline()
        DD=data.split(",")
        DD[2]= DD[2].strip('\r\n')

        d1 = int(DD[0])*0.01*0.8718 - 0.1834
        d2 = int(DD[1])*0.01*0.8587 - 0.1882
        d3 = int(DD[2])*0.01*0.8821 - 0.1896
        
        time.sleep(0.1)
        
    
th1 = threading.Thread(target=compute) 
th1.start() 

while 1:
    start = time.time()
    
    print d1, d2, d3

    elapsed_time = time.time() - start
    time.sleep(0.05-elapsed_time)
