import time, sys

if __name__ == '__main__':
    
    while True:
        try:
            print "loop!!"
            time.sleep(1)

        except Exception, error:
            print "error occur!!"
            print "Error on main:{}".format(str(error))

        except KeyboardInterrupt:
            print "Keyboard Interrupt!! Close this file!!"
            #sys.exit()
            break
            #exit()