import smbus
import time
import datetime,csv

if __name__ == '__main__':
    bus = smbus.SMBus(1)
    time.sleep(1)
    adress = 0x04
    msg = [0,0,0,0]
    msg_t = [0,0,0,0]
    num = [0,0,0,0]
    num_true = [0,0,0,0]

    st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"

    #f = open("./logs/UWB_i2c/Logs_test"+st, "w")
    #logger = csv.writer(f)

    try:
        while True:
            # bus.write_byte(adress, ord('1'))
            # #time.sleep(0.01)
           
            # msg[0] = bus.read_word_data(adress, 0)

            # if(msg[0]>0):
            #     msg_t[0] = msg[0]
            
            # time.sleep(0.01)

            # bus.write_byte(adress, ord('2'))
            # #time.sleep(0.01)
            
            # msg[1] = bus.read_word_data(adress, 0)
            # if(msg[1]>0):
            #     msg_t[1] = msg[1]
           
            # time.sleep(0.01)

            # bus.write_byte(adress, ord('3'))
            # #time.sleep(0.01)
            
            # msg[2] = bus.read_word_data(adress, 0)
            # if(msg[2]>0):
            #     msg_t[2] = msg[2]
           
            # time.sleep(0.01)

            # bus.write_byte(adress, ord('4'))
            # #time.sleep(0.01)
           
            # msg[3] = bus.read_word_data(adress, 0)
            # if(msg[3]>0):
            #     msg_t[3] = msg[3]
            
            # print(msg_t[0],msg_t[1],msg_t[2],msg_t[3])
            # time.sleep(0.01)

         

            result = bus.read_i2c_block_data(adress,0,8)
            time.sleep(0.03)

            num[0] = result[1] << 8 | result[0]
            num[1] = result[3] << 8 | result[2]
            num[2] = result[5] << 8 | result[4]
            num[3] = result[7] << 8 | result[6]

            for i in range(4):
                if(num[i] < 2000):
                    num_true[i] = num[i]

            print(num_true[0],num_true[1],num_true[2],num_true[3])
            time.sleep(0.03)

            #logger.writerow((num_true[0],num_true[1],num_true[2],num_true[3]))

      

    except KeyboardInterrupt:
        print('finish')
