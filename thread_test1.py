import threading 
import time 

global xx 
xx = 0 

def compute():
    global xx
    while 1:
        xx = 0
        time.sleep(1)
    
th1 = threading.Thread(target=compute) 
th1.start() 

while 1:
    start = time.time()
    xx = xx + 1
    
    elapsed_time = time.time() - start
    print xx
    time.sleep(0.1-elapsed_time)
