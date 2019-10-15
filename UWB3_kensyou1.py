import numpy as np
from numpy import linalg as la
import time
import spidev
import sys
import navio.util
import navio.mpu9250
import navio.adc
import navio.util
import serial
import threading
import VL53L0X

del_t = 0.01 #sec
g = 0 #m/s^2
x_old = np.array([0,0,0.8,0,0,0,0,0,0],dtype=np.float) #m, m/s, rad
omega = np.array([0,0,0],dtype=np.float) #rad/sec
acc = np.array([0,0,-g],dtype=np.float) #m/s^2
gamma = 800 # 0~1000
P_old = np.identity(9)*gamma
I_9 = np.identity(9)
acc_noise=0.01
gyro_noise=0.0003468268
QQ = np.diag([0,0,0,acc_noise,acc_noise,acc_noise,gyro_noise,gyro_noise,gyro_noise])
RR = np.diag([0.1,0.1,0.1,0.02,0.02,0.02,0.01])

alfa = np.array([0.8244,0.8244,0.8244],dtype=np.float)
m9a_low_old = np.array([0,0,0],dtype=np.float)
m9g_low_old = np.array([0,0,0],dtype=np.float)

anchor1 = np.array([-1, 1, 1],dtype=float)
anchor2 = np.array([ 1, 1, 1],dtype=float)
anchor3 = np.array([ 1,-1, 1],dtype=float)
anchor4 = np.array([-1,-1, 1],dtype=float)


#global dd1
#global dd2
#global dd3

global DD
global hh

dd1 = 1.732
dd2 = 1.732
dd3 = 1.732
dd4 = 1.732

ser = serial.Serial("/dev/ttyAMA0" , 115200)
tof = VL53L0X.VL53L0X()
tof.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)

def compute():
    #global dd1
    #global dd2
    #global dd3

    global DD
    global hh

    data = None
    DD = None

    while 1:

        #data = ser.readline()
        #DD=data.split(",")
        #DD[2]= DD[2].strip('\r\n')

        #dd1 = int(DD[0])*0.01*0.8718 - 0.1834
        #dd2 = int(DD[1])*0.01*0.8587 - 0.1882
        #dd3 = int(DD[2])*0.01*0.8821 - 0.1896
        #print dd1,dd2,dd3

        #h_distance =  int(tof.get_distance())
        #if (h_distance > 0 and h_distance < 2000):
            hh = h_distance

        #time.sleep(0.05)

        #n = n + 1

        #time.sleep(5)

#th1 = threading.Thread(target=compute)
#th1.start()

##############################################
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
##############################################
i = 0
gyro_x=0
gyro_y=0
gyro_z=0

while i<100:

    m9a, m9g, m9m = imu.getMotion9()

    gyro_x = gyro_x + m9g[0]
    gyro_y = gyro_y + m9g[1]
    gyro_z = gyro_z + m9g[2]

    g = g + np.sqrt(m9a[0]*m9a[0] + m9a[1]*m9a[1] + m9a[2]*m9a[2])

    i= i+1

    time.sleep(0.1)

bias_gyro_x = gyro_x/100
bias_gyro_y = gyro_y/100
bias_gyro_z = gyro_z/100
g=g/100

print(bias_gyro_x,bias_gyro_y,bias_gyro_z,g)
print('Calibration end')

time.sleep(0.5)

#th1 = threading.Thread(target=compute)
#th1.start()

time.sleep(1)

print u'start? (y/n)'

start_flag = raw_input('>>')

if start_flag != "y":
    print('end!!')


ser.write(b'A')

while start_flag == "y":
    start =time.time()

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
                   [0,0,1,0,0,0,0,0,0]
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
                      [x_pre[:,0][2]]
                      ], dtype=np.float)

    m9a, m9g, m9m = imu.getMotion9() #measure

    m9a_low = m9a_low_old * alfa + m9a * (1-alfa)
    m9g_low = m9g_low_old * alfa + m9g * (1-alfa)

    #print h_pre[5]

    m9a_low_old = m9a_low
    m9g_low_old = m9g_low

    acc = np.array([m9a_low[0],m9a_low[1],-m9a_low[2]])

    #results[5] = adc.read(5) #measure

    omega = np.array([-(m9g_low[0]-bias_gyro_y),-(m9g_low[1]-bias_gyro_x),m9g_low[2]-bias_gyro_z])

    #DD[3]= DD[3].strip('\r\n')
    #if (int(DD[0]) < 2000 and int(DD[1]) < 2000 and int(DD[2]) < 2000 and int(DD[3]) < 2000):
    dd1 = 1.732#int(DD[0])*0.01
    dd2 = 1.732#int(DD[1])*0.01
    dd3 = 1.732#int(DD[2])*0.01
    dd4 = 1.732#int(DD[3])*0.01

    height = 1

    yy = np.array([[ m9a_low[0]],
                   [ m9a_low[1]],
                   [-m9a_low[2]],
                   [dd1],
                   [dd2],
                   [dd3],
                   [height]
                   ],dtype=np.float)

    P_new = (I_9-GG.dot(CC)).dot(P_pre)
    x_new = x_pre + GG.dot(yy-h_pre)
    #SS=GG.dot(yy-h_pre)
    #print dd1,dd2,dd3
#    phi = np.arctan(-m9a[1]/m9a[2])*180/np.pi
    print "{:+7.3f}".format(x_new[:,0][0]),",","{:+7.3f}".format(x_new[:,0][1]),",","{:+7.3f}".format(x_new[:,0][8]*180/np.pi)#,",","{:+7.3f}".format(dd1),",","{:+7.3f}".format(dd2),",","{:+7.3f}".format(dd3)
#    print "{:+7.3f}".format(x_new[:,0][0]),",","{:+7.3f}".format(x_new[:,0][1])

    #print(x_new[:,0][6]*180/np.pi,x_new[:,0][7]*180/np.pi)
    elapsed_time = time.time() - start

    #print(x_new[:,0][6]*180/np.pi,x_new[:,0][7]*180/np.pi,x_new[:,0][8]*180/np.pi)
    #print(elapsed_time)
#    sleep_time = 0.01 - elapsed_time
#    time.sleep(sleep_time)

    if elapsed_time < 0.01:

        sleep_time = 0.01 - elapsed_time
        time.sleep(sleep_time)

    P_old = P_new#[:,0]
    x_old = x_new[:,0]
