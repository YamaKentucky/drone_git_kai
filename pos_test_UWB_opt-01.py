##modules for UWB(i2c) opt(serial)
import smbus
import time
import datetime, csv
import threading, serial

##modules for positoon estimate
import numpy as np
from numpy import linalg as la
import spidev
import sys
import navio.util
import navio.mpu9250
import navio.adc
import navio.util

##modules for dronekit
from dronekit import connect, VehicleMode

##for opt serial
ser = serial.Serial("/dev/ttyAMA0" , 115200)

##for position estimate
del_t = 0.0166 #sec
#del_t = 0.05 #sec
g = 9.81 #m/s^2
x_old = np.array([0,0,0,0,0,0,0,0,0],dtype=np.float) #m, m/s, rad
omega = np.array([0,0,0],dtype=np.float) #rad/sec
acc = np.array([0,0,-g],dtype=np.float) #m/s^2
gamma = 800 # 0~1000
P_old = np.identity(9)*gamma
I_9 = np.identity(9)
acc_noise=0.001
gyro_noise=0.0003468268
QQ = np.diag([0,0,0,acc_noise,acc_noise,acc_noise,gyro_noise,gyro_noise,gyro_noise])
RR = np.diag([0.4,0.4,0.4,0.0001,0.0001,0.0001,0.0001,0.01,0.008,0.005,0.005]) #add yaw_pix noise, opt noise

alfa = np.array([0.8244,0.8244,0.8244],dtype=np.float)
m9a_low_old = np.array([0, 0, 0], dtype = np.float)
m9g_low_old = np.array([0, 0, 0], dtype = np.float)

# anchor1 = np.array([-3.5,  2.0, 1.820],dtype=float)
# anchor2 = np.array([ 3.5,  2.0, 1.820],dtype=float)
# anchor3 = np.array([ 3.5, -2.0, 1.820],dtype=float)
# anchor4 = np.array([-3.5, -2.0, 1.820],dtype=float)

anchor1 = np.array([-1,   1.5, 0.73],dtype=float)
anchor2 = np.array([ 1,   1.5, 0.73],dtype=float)
anchor3 = np.array([ 1,  -1.5, 0.73],dtype=float)
anchor4 = np.array([-1,  -1.5, 0.73],dtype=float)

##for logging
logging = True

##for position control
vehicle = connect('/dev/ttyACM0', wait_ready = True, baud = 921600)


def uart():
    global DD, OPT, height, deltaX, deltaY, deltaX_sum, deltaY_sum, time_lap, deltaX_sum_ar, deltaY_sum_ar

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
        deltaX = -float(OPT[1])  * 0.001  #m/s
        deltaY = float(OPT[2])  * 0.001  #m/s
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
        #time.sleep(0.042)                         ##delay

def startup():
    global DD_old
    DD_old = 0
    th1 = threading.Thread(target = uart)
    th1.start()

    print('SYSTEM ALL GREEN')
    log()
    time.sleep(2)
    DD_old = DD
    print  "DD_old[0]: {:5.0f}, DD_old[1]: {:5.0f}, DD_old[2]: {:5.0f}, DD_old[3]: {:5.0f}".format(DD_old[0], DD_old[1], DD_old[2], DD_old[3])
    time.sleep(2)


def IMU():
    global g, bias_gyro_x, bias_gyro_y, bias_gyro_z
    global imu
    g = bias_gyro_x = bias_gyro_y = bias_gyro_z = 0

    gyro_x = gyro_y = gyro_z = 0

    imu = navio.mpu9250.MPU9250()
    navio.util.check_apm()
    adc = navio.adc.ADC()
    results = [0] * adc.channel_count

    if imu.testConnection():
        print("Connection established: True")

    else:
        sys.exit("Connection established: False")

    imu.initialize()
    time.sleep(1)

    for i in range(100):
        m9a, m9g, m9m = imu.getMotion9() #acc, gyro, mag

        gyro_x = gyro_x + m9g[0]
        gyro_y = gyro_y + m9g[1]
        gyro_z = gyro_z + m9g[2]

        g = g + np.sqrt(pow(m9a[0], 2) + pow(m9a[1], 2) + pow(m9a[2], 2))
        time.sleep(0.03)

    bias_gyro_x = gyro_x / 100
    bias_gyro_y = gyro_y / 100
    bias_gyro_z = gyro_z / 100
    g = g / 100
    
    print("Calibration end!!")
    print ("g: {:5.3f}, bias_gyro_x: {:5.3f}, bias_gyro_y: {:5.3f}, bias_gyro_z: {:5.3f}".format(g, bias_gyro_x, bias_gyro_y, bias_gyro_z))
    time.sleep(1)

def yaw_calibration():
    global bias_yaw
    bias_yaw = yaw_cal = 0

    for i in range(100):
        yaw_cal = yaw_cal + vehicle.attitude.yaw
    
    bias_yaw = yaw_cal / 100
    print "yaw_cal: {:3.3f}".format(bias_yaw)
    time.sleep(1)

def yaw_filter(yaw):
    yaw_true = - yaw + bias_yaw

    if  np.pi <= yaw_true  <= 2 * np.pi:
        yaw_sign = yaw_true - 2 * np.pi

    else:
        yaw_sign = yaw_true 

    return yaw_sign

def pos_cal():
    bias_x = bias_gyro_y = bias_gyro_z = 0
    pos_x = pos_y = pos_z = 0

    for i in range(100):
        result = pos_estimate()
        pos_x = pos_x + result[0]
        pos_y = pos_y + result[1]
        pos_z = pos_z + result[2]
        time.sleep(0.02)

    bias_x = pos_x / 100
    bias_y = pos_y / 100
    bias_z = pos_z / 100

    print "Position cal finish!!"
    print "bias_x:{:+7.3f}, bias_y:{:+7.3f}, bias_z:{:+7.3f}". format(bias_x, bias_y, bias_z)
    return bias_x, bias_y, bias_z

def log():
    global logger
    if logging:
        print ("Logging mode")
        st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
        f = open("./logs/position_opt/Logs_opt_test"+st, "w")
        logger = csv.writer(f)
        logger.writerow(("timestamp", "x", "y", "z", "deltaX", "deltaY","deltaX_sum", "deltaY_sum", "v_x", "v_y", "DD[0]", "DD[1]", "DD[2]", "DD[3]", "Pitch", "Roll", "Yaw"))


def pos_estimate(bias_x = 0,bias_y = 0,bias_z = 0):
    global x_old, acc, omega, P_old, m9a_low_old, m9g_low_old, x_new

    start = time.time()

    s_phi = np.sin(x_old[6])
    c_phi = np.cos(x_old[6])
    s_the = np.sin(x_old[7])
    c_the = np.cos(x_old[7])
    t_the = np.tan(x_old[7])
    s_psi = np.sin(x_old[8])
    c_psi = np.cos(x_old[8])


    x_pre = np.array([[x_old[0]+x_old[3]*del_t],
                    [x_old[1]+x_old[4]*del_t],
                    [x_old[2]+x_old[5]*del_t],
                    [x_old[3]+c_the*c_psi*acc[0]*del_t+(s_the*c_psi*s_phi-s_psi*c_phi)*acc[1]*del_t+(s_the*c_psi*c_phi+s_psi*s_phi)*acc[2]*del_t],
                    [x_old[4]+c_the*s_psi*acc[0]*del_t+(s_the*s_psi*s_phi+c_psi*c_phi)*acc[1]*del_t+(s_the*s_psi*c_phi-c_psi*s_phi)*acc[2]*del_t],
                    [x_old[5]-s_the*acc[0]*del_t+c_the*s_phi*acc[1]*del_t+c_the*c_phi*acc[2]*del_t+g*del_t],
                    [x_old[6] + omega[0]*del_t+s_phi*t_the*omega[1]*del_t + c_phi*t_the*omega[2]*del_t],
                    [x_old[7] + c_phi*omega[1]*del_t - s_phi*omega[2]*del_t],
                    [x_old[8] + s_phi/c_the*omega[1]*del_t + c_phi/c_the*omega[2]*del_t]
                    ],dtype=np.float)
    #print x_old[3]+c_the*c_psi*acc[0]*del_t+(s_the*c_psi*s_phi-s_psi*c_phi)*acc[1]*del_t+(s_the*c_psi*c_phi+s_psi*s_phi)*acc[2]*del_t
########################################################################################

    AA = np.array([[1,0,0,del_t,0,0,0,0,0],
                [0,1,0,0,del_t,0,0,0,0],
                [0,0,1,0,0,del_t,0,0,0],
                [0,0,0,1,0,0, (s_the*c_psi*c_phi+s_psi*s_phi)*acc[1]*del_t + (-s_the*c_psi*s_phi+s_psi*c_phi)*acc[2]*del_t,
                                -s_the*c_psi*acc[0]*del_t + c_the*c_psi*s_phi*acc[1]*del_t + c_the*c_psi*c_phi*acc[2]*del_t,
                                -c_the*s_psi*acc[0]*del_t + (-s_the*s_psi*s_phi-c_psi*c_phi)*acc[1]*del_t + (-s_the*s_psi*c_phi+c_psi*s_phi)*acc[2]*del_t],
                [0,0,0,0,1,0, (s_the*s_psi*c_phi-c_psi*s_phi)*acc[1]*del_t + (-s_the*s_psi*s_phi-c_psi*c_phi)*acc[2]*del_t,
                                -s_the*s_psi*acc[0]*del_t + c_the*s_psi*s_phi*acc[1]*del_t + c_the*s_psi*s_phi*acc[2]*del_t,
                                c_the*c_psi*acc[0]*del_t + (s_the*s_psi*s_phi-s_psi*c_phi)*acc[1]*del_t + (s_the*c_psi*c_phi+s_psi*s_phi)*acc[2]*del_t],
                [0,0,0,0,0,1, c_the*c_phi*acc[1]*del_t - c_the*s_phi*acc[2]*del_t, -c_the*acc[0]*del_t - s_the*s_phi*acc[1] - s_the*c_phi*acc[2]*del_t,0],
                [0,0,0,0,0,0, 1+c_phi*t_the*omega[1]*del_t-s_phi*t_the*omega[2]*del_t, s_phi/(c_the*c_the)*omega[1]*del_t+c_phi/(c_the*c_the)*omega[2]*del_t,0],
                [0,0,0,0,0,0, -s_phi*omega[1]*del_t-c_phi*omega[2]*del_t,1,0],
                [0,0,0,0,0,0, c_phi/c_the*omega[1]*del_t-s_phi/c_the*omega[2]*del_t, s_phi*t_the/c_the*omega[1]*del_t+c_phi*t_the/c_the*omega[2]*del_t,0]
                ],dtype=np.float)

    s_phi_pre = np.sin(x_pre[6])
    c_phi_pre = np.cos(x_pre[6])
    s_the_pre = np.sin(x_pre[7])
    c_the_pre = np.cos(x_pre[7])

    AA1 = np.sqrt(pow(x_pre[:,0][0]-anchor1[0] ,2) + pow(x_pre[:,0][1]-anchor1[1] ,2) + pow(x_pre[:,0][2]-anchor1[2] ,2))
    AA2 = np.sqrt(pow(x_pre[:,0][0]-anchor2[0] ,2) + pow(x_pre[:,0][1]-anchor2[1] ,2) + pow(x_pre[:,0][2]-anchor2[2] ,2))
    AA3 = np.sqrt(pow(x_pre[:,0][0]-anchor3[0] ,2) + pow(x_pre[:,0][1]-anchor3[1] ,2) + pow(x_pre[:,0][2]-anchor3[2] ,2))
    AA4 = np.sqrt(pow(x_pre[:,0][0]-anchor4[0] ,2) + pow(x_pre[:,0][1]-anchor4[1] ,2) + pow(x_pre[:,0][2]-anchor4[2] ,2))

    CC = np.array([[0,0,0,0,0,0,0,c_the_pre*g,0],
                [0,0,0,0,0,0,-c_the_pre*c_phi_pre*g,s_the_pre*s_phi_pre*g,0],
                [0,0,0,0,0,0, c_the_pre*s_phi_pre*g,s_the_pre*c_phi_pre*g,0],
                [(x_pre[:,0][0]-anchor1[0])/AA1, (x_pre[:,0][1]-anchor1[1])/AA1, (x_pre[:,0][2]-anchor1[2])/AA1, 0,0,0,0,0,0],
                [(x_pre[:,0][0]-anchor2[0])/AA2, (x_pre[:,0][1]-anchor2[1])/AA2, (x_pre[:,0][2]-anchor2[2])/AA2, 0,0,0,0,0,0],
                [(x_pre[:,0][0]-anchor3[0])/AA3, (x_pre[:,0][1]-anchor3[1])/AA3, (x_pre[:,0][2]-anchor3[2])/AA3, 0,0,0,0,0,0],
                [(x_pre[:,0][0]-anchor4[0])/AA4, (x_pre[:,0][1]-anchor4[1])/AA4, (x_pre[:,0][2]-anchor4[2])/AA4, 0,0,0,0,0,0],
                [0,0,1,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,1],   #yaw jakob
                [0,0,0,1,0,0,0,0,0],   #v_x jakob 
                [0,0,0,0,1,0,0,0,0]    #v_y jakob
                ],dtype=np.float)

#########################################################################################
    P_pre = AA.dot(P_old.dot(AA.T)) + QQ

    G1 = la.inv(CC.dot(P_pre.dot(CC.T)) + RR)
    GG = P_pre.dot((CC.T).dot(G1))
#########################################################################################
    h_pre = np.array([[s_the_pre*g],
                    [-c_the_pre*s_phi_pre*g],
                    [-c_the_pre*c_phi_pre*g],
                    [np.sqrt(pow(x_pre[:,0][0]-anchor1[0] ,2) + pow(x_pre[:,0][1]-anchor1[1] ,2) + pow(x_pre[:,0][2]-anchor1[2] ,2))],
                    [np.sqrt(pow(x_pre[:,0][0]-anchor2[0] ,2) + pow(x_pre[:,0][1]-anchor2[1] ,2) + pow(x_pre[:,0][2]-anchor2[2] ,2))],
                    [np.sqrt(pow(x_pre[:,0][0]-anchor3[0] ,2) + pow(x_pre[:,0][1]-anchor3[1] ,2) + pow(x_pre[:,0][2]-anchor3[2] ,2))],
                    [np.sqrt(pow(x_pre[:,0][0]-anchor4[0] ,2) + pow(x_pre[:,0][1]-anchor4[1] ,2) + pow(x_pre[:,0][2]-anchor4[2] ,2))],
                    [x_pre[:,0][2]],
                    [x_pre[:,0][8]],     #yaw
                    [x_pre[:,0][3]],     #v_x
                    [x_pre[:,0][4]]      #v_y
                    ], dtype=np.float)

    m9a, m9g, m9m = imu.getMotion9() #measure

    m9a_low = m9a_low_old * alfa + m9a * (1-alfa)
    m9g_low = m9g_low_old * alfa + m9g * (1-alfa)

    #print h_pre[5]

    m9a_low_old = m9a_low
    m9g_low_old = m9g_low

    acc = np.array([-m9a_low[0],-m9a_low[1],-m9a_low[2]])

    #results[5] = adc.read(5) #measure

    omega = np.array([(m9g[0]-bias_gyro_x),(m9g[1]-bias_gyro_y),m9g[2]-bias_gyro_z])


    if abs(int(DD_old[0]) - int(DD[0])) < 120:
            dd1 = int(DD[0])*0.01   ## mm -> m
            DD_old[0] = DD[0]

    if abs(int(DD_old[1]) - int(DD[1])) < 120:
            dd2 = int(DD[1])*0.01
            DD_old[1] = DD[1]

    if abs(int(DD_old[2]) - int(DD[2])) < 120:
            dd3 = int(DD[2])*0.01
            DD_old[2] = DD[2]
        
    if abs(int(DD_old[3]) - int(DD[3])) < 120:
            dd4 = int(DD[3])*0.01
            DD_old[3] = DD[3]

    v_xopt = deltaX * c_psi - deltaY * s_psi
    v_yopt = deltaX * s_psi + deltaY * c_psi

    yy = np.array([[-m9a_low[0]],
                [-m9a_low[1]],
                [-m9a_low[2]],
                [dd1],
                [dd2],
                [dd3],
                [dd4],
                [height],
                [yaw_filter(vehicle.attitude.yaw)],    #yaw_pix
                [v_xopt],                              #opt x
                [v_yopt]                               #opt y
                ],dtype=np.float)

    P_new = (I_9-GG.dot(CC)).dot(P_pre)
    x_new = x_pre + GG.dot(yy-h_pre)

    ##estimated Positions
    pos_x = x_new[:,0][0] - bias_x
    pos_y = x_new[:,0][1] - bias_y
    pos_z = x_new[:,0][2] - bias_z
    
    #print "pos_x:{:+7.3f}, pos_y:{:+7.3f}, pos_z:{:+7.3f}".format(pos_x, pos_y, pos_z)
    print "pos_x:{:+7.3f}, pos_y:{:+7.3f}, pos_z:{:+7.3f}, deltaX_sum: {:+5.3f}, deltaY_sum:{:+5.3f}".format(pos_x, pos_y, pos_z, deltaX_sum, deltaY_sum)
    row = ("{:6.3f}".format(time.time()), "{:.3f}".format(pos_x), "{:.3f}".format(pos_y) , "{:.3f}".format(pos_z)
            ,"{:.3f}".format(deltaX), "{:.3f}".format(deltaY)
            , "{:.3f}".format(deltaX_sum), "{:.3f}".format(deltaY_sum)
            ,"{:.3f}".format(x_new[:,0][3]), "{:.3f}".format(x_new[:,0][4])
            ,"{:.3f}".format(DD[0]), "{:.3f}".format(DD[1]), "{:.3f}".format(DD[2]), "{:.3f}".format(DD[3])
            ,"{:.3f}".format(x_new[:,0][6]), "{:.3f}".format(x_new[:,0][7]), "{:.3f}".format(x_new[:,0][8])
            )
    
    if logging:
        logger.writerow(row)

    elapsed_time = time.time() - start
   

    if elapsed_time < del_t:
        sleep_time = del_t - elapsed_time
        time.sleep(sleep_time)

    P_old = P_new#[:,0]
    x_old = x_new[:,0]

    return pos_x, pos_y, pos_z, x_new[:,0][3], x_new[:,0][4], x_new[:,0][5], x_new[:,0][6], x_new[:,0][7], x_new[:,0][8]




if __name__ == '__main__':
    startup()
    IMU()
    yaw_calibration()
    bias_pos = pos_cal()
    #print ("g: {:5.3f}, bias_gyro_x: {:5.3f}, bias_gyro_y: {:5.3f}, bias_gyro_z: {:5.3f}".format(g, bias_gyro_x, bias_gyro_y, bias_gyro_z))
    time.sleep(1)

    while True:
        pos_estimate(bias_pos[0], bias_pos[1], bias_pos[2])
        print "Height:{:+5.3f}, deltaX:{:+5.3f}, deltaY:{:+5.3f}, DD[0]:{:5.0f}, DD[1]:{:5.0f}, DD[2]:{:5.0f}, DD[3]:{:5.0f}" .format(height, deltaX, deltaY, DD[0], DD[1], DD[2], DD[3])
        time.sleep(0.042)

    
