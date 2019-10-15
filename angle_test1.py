import numpy as np
from numpy import linalg as la
import time
import spidev
import sys
import navio.util
import navio.mpu9250
import navio.adc
import navio.util
import navio.rcinput
import navio.pwm

del_t = 0.01 #sec
g = 0 #m/s^2
x_old = np.array([0,0,0,0,0,0,0,0,0],dtype=np.float) #m, m/s, rad
omega = np.array([0,0,0],dtype=np.float) #rad/sec
acc = np.array([0,0,-g],dtype=np.float) #m/s^2
gamma = 500 # 0~1000
P_old = np.identity(9)*gamma
I_9 = np.identity(9)
acc_noise=0.001#0.6
gyro_noise=0.0003468268
QQ = np.diag([0,0,0,acc_noise,acc_noise,acc_noise,gyro_noise,gyro_noise,gyro_noise])
RR = np.diag([0.05,0.05,0.05])


roll_max_angle  = 20 *np.pi/180 #rad
pitch_max_angle = 20 *np.pi/180 #rad
yawrate_max     = 0#45 *np.pi/180 #rad/sec

e_roll_old  = 0 #default
e_pitch_old = 0

ll = 165 #mm
Ct = 0.0374 #mN , 1/sec
Cm = 0.3428 #mNmm , 1/sec
CC1 = Cm / Ct
Kp_roll  = 145#18.9
Kd_roll  = 56#46
Kp_pitch = 170#16.08
Kd_pitch = 64#54#
Kp_yaw   = 63
beta = 1.8
c = np.array([2*beta, 0.5*beta, 2*beta, 0.5*beta, 3.5*beta, 1.5*beta, 11*beta, 5*beta, 11*beta, 5*beta, 2*beta, 2*beta])

#c = np.array([2*beta, 0.5*beta, 2*beta, 0.5*beta, 3.5*beta, 1.5*beta, 11*beta, 5*beta, 11*beta, 5*beta, 2*beta, 2*beta])
#c = np.array([20,5,20,5,35,15,100,20,100,20,20,20])
ramda1  = 0.005
ramda3  = 0.005
ramda5  = 0.005
ramda7  = 0.005
ramda9  = 0.005
ramda11 = 0.005

e_1_pre  = 0
e_3_pre  = 0
e_5_pre  = 0
e_7_pre  = 0
e_9_pre  = 0
e_11_pre = 0

I_x = 0.001915
I_y = 0.001630
I_z = 0.003147

m = 0.603 #kg

a_1 = (I_y - I_z)/I_x
a_2 = (I_z - I_x)/I_y
a_3 = (I_x - I_y)/I_z

imu = navio.mpu9250.MPU9250()
rcin = navio.rcinput.RCInput()

alfa = np.array([0.9,0.9,0.9],dtype=np.float)
m9a_low_old = np.array([0,0,9.81],dtype=np.float)
m9g_low_old = np.array([0,0,0],dtype=np.float)


##############################################
navio.util.check_apm()

pwm1 = navio.pwm.PWM(0)
pwm1.set_period(400)

pwm2 = navio.pwm.PWM(1)
pwm2.set_period(400)

pwm3 = navio.pwm.PWM(2)
pwm3.set_period(400)

pwm4 = navio.pwm.PWM(3)
pwm4.set_period(400)


if imu.testConnection():
    print("Connection established: True")
else:
    sys.exit("Connection established: False")
imu.initialize()
adc = navio.adc.ADC()
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

while True:
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

    CC = np.array([[0,0,0,0,0,0,0,c_the_pre*g,0],
                   [0,0,0,0,0,0,-c_the_pre*c_phi_pre*g,s_the_pre*s_phi_pre*g,0],
                   [0,0,0,0,0,0, c_the_pre*s_phi_pre*g,s_the_pre*c_phi_pre*g,0],
                   ],dtype=np.float)

#########################################################################################
    P_pre = AA.dot(P_old.dot(AA.T)) + QQ

    G1 = la.inv(CC.dot(P_pre.dot(CC.T)) + RR)
    GG = P_pre.dot((CC.T).dot(G1))
#########################################################################################
    h_pre = np.array([s_the_pre*g,
                      -c_the_pre*s_phi_pre*g,
                      -c_the_pre*c_phi_pre*g
                      ], dtype=np.float)
    m9a, m9g, m9m = imu.getMotion9() #measure
    #results[5] = adc.read(5) #measure

    m9a_low = m9a_low_old * alfa + m9a * (1-alfa)
    m9g_low = m9g_low_old * alfa + m9g * (1-alfa)

    m9a_low_old = m9a_low
    m9g_low_old = m9g_low
    
    acc = np.array([-m9a_low[0],-m9a_low[1],-m9a_low[2]])

    omega = np.array([(m9g[0]-bias_gyro_x),(m9g[1]-bias_gyro_y),m9g[2]-bias_gyro_z])

    yy = np.array([[-m9a_low[0]],
                   [-m9a_low[1]],
                   [-m9a_low[2]]])

    P_new = (I_9-GG.dot(CC)).dot(P_pre)
    x_new = x_pre + GG.dot(yy-h_pre)
    
#    angle = -0.07416*adc.read(4)+172.73
#    angle = 0.0491*adc.read(4)-64.354
#    angle =-0.0658*adc.read(4)+171.9

#    print "{:+7.3f}".format(x_new[:,0][6]*180/np.pi),",","{:+7.3f}".format(x_new[:,0][7]*180/np.pi),",","{:+7.3f}".format(x_new[:,0][8]*180/np.pi)
    print "{:+7.3f}".format(x_new[:,0][6]*180/np.pi),",","{:+7.3f}".format(x_new[:,0][7]*180/np.pi),",","{:+7.3f}".format(x_new[:,0][8]*180/np.pi)#,",","{:+7.3f}".format(angle)

#########################################################################################


    roll  =  (float(rcin.read(3))-1514) * roll_max_angle / 410
    pitch =  (float(rcin.read(1))-1514) * pitch_max_angle / 410
    yaw   = 0#-(float(rcin.read(0))-1514) * yawrate_max / 410
    throt =  (float(rcin.read(2))-1104) * 14000 / 820
#    print throt
    if throt > 10:
        x_d = 1 #m
        y_d = 1 #m
        z_d = 1 #m

        #ftotal
        e_5 = e_5_pre + (z_d - x_new[2])*del_t

        U_1 = m * ( (1+ramda5+c[4]*c[5])*(z_d-x_new[:,0][2])+(c[4]+c[5])*(-x_new[:,0][5])+c[5]*ramda5*e_5 + g ) / (np.cos(x_new[:,0][6])*np.cos(x_new[:,0][7]))

        #f_total = U_1*1000
        e_5_pre = e_5

        #disired atittude
        e_1 = e_1_pre + (x_d - x_new[0])*del_t
        e_3 = e_3_pre + (y_d - x_new[1])*del_t

        U_x = m * ((1+ramda1+c[0]*c[1])*(x_d-x_new[:,0][0])+(c[0]+c[1])*(-x_new[:,0][3])+c[1]*ramda1*e_1) / U_1
        U_y = m * ((1+ramda3+c[2]*c[3])*(x_d-x_new[:,0][1])+(c[2]+c[3])*(-x_new[:,0][4])+c[3]*ramda3*e_3) / U_1

        e_1_pre = e_1
        e_3_pre = e_3

        phi_d   = np.arcsin(U_x*np.sin( yaw )-U_y*np.cos( yaw ))
        theta_d = np.arcsin((U_x*np.cos( yaw )+U_y*np.sin( yaw ))/np.cos(phi_d))
        psi_d   = yaw


        #atittude control
        e_7  = e_7_pre  + (pitch  - x_new[:,0][6])*del_t
        e_9  = e_9_pre  + (roll - x_new[:,0][7])*del_t
        e_11 = e_11_pre + (yaw   - x_new[:,0][8])*del_t

        omega1 = m9g_low[0]-bias_gyro_x
        omega2 = m9g_low[1]-bias_gyro_y
        omega3 = m9g_low[2]-bias_gyro_z
        
        tau_pitch = 1000 * 1000 * I_x * ((1+ramda7+c[6]*c[7])*(pitch-x_new[:,0][6])+(c[6]+c[7])*(-omega1)+c[7]*ramda7*e_7-a_1*omega2*omega3)
        
        tau_roll = 1000 * 1000 * I_y * ((1+ramda9+c[8]*c[9])*(roll-x_new[:,0][7])+(c[8]+c[9])*(-omega2)+c[9]*ramda9*e_9-a_2*omega1*omega3)
        #print (roll - x_new[:,0][7])*180/np.pi
        tau_yaw = 1000 * 1000 * I_z * ((1+ramda11+c[10]*c[11])*(yaw-x_new[:,0][8])+(c[10]+c[11])*(-omega3)+c[11]*ramda11*e_11-a_3*omega1*omega2)

        e_7_pre  = e_7
        e_9_pre  = e_9
        e_11_pre = e_11

        f_total    = throt #U_1

        omega2_1 = (f_total + tau_roll/ll + tau_pitch/ll + tau_yaw/CC1) / (4*Ct)
        omega2_2 = (f_total - tau_roll/ll + tau_pitch/ll - tau_yaw/CC1) / (4*Ct)
        #print omega2_1 - omega2_2
        omega2_3 = (f_total - tau_roll/ll - tau_pitch/ll + tau_yaw/CC1) / (4*Ct)
        omega2_4 = (f_total + tau_roll/ll - tau_pitch/ll - tau_yaw/CC1) / (4*Ct)

        if   omega2_1 < 0:
            omega2_1 = 0
        if omega2_1 > 110000:
            omega2_1 = 110000
        if omega2_2 < 0:
            omega2_2 = 0
        if omega2_2 > 110000:
            omega2_2 = 110000
        if omega2_3 < 0:
            omega2_3 = 0
        if omega2_3 > 110000:
            omega2_3 = 110000
        if omega2_4 < 0:
            omega2_4 = 0
        if omega2_4 > 110000:
            omega2_4 = 110000

        omega_1 = np.sqrt(omega2_1+1)
        omega_2 = np.sqrt(omega2_2+1)
        omega_3 = np.sqrt(omega2_3+1)
        omega_4 = np.sqrt(omega2_4+1)

        duty1 = 1.8616*omega_1 + 1056.5
        duty2 = 1.8616*omega_2 + 1056.5
        duty3 = 1.8616*omega_3 + 1056.5
        duty4 = 1.8616*omega_4 + 1056.5

#        pwm1.set_duty_cycle(int(duty1)*0.001)
#        pwm2.set_duty_cycle(int(duty2)*0.001)
#        pwm3.set_duty_cycle(int(duty3)*0.001)
#        pwm4.set_duty_cycle(int(duty4)*0.001)

    else:
        duty1 = 1056.5
        duty2 = 1056.5
        duty3 = 1056.5
        duty4 = 1056.5

#        pwm1.set_duty_cycle(int(duty1)*0.001)
#        pwm2.set_duty_cycle(int(duty2)*0.001)
#        pwm3.set_duty_cycle(int(duty3)*0.001)
#        pwm4.set_duty_cycle(int(duty4)*0.001)


    elapsed_time = time.time() - start
    #print elapsed_time

    if elapsed_time < 0.01:

        sleep_time = 0.01 - elapsed_time
        time.sleep(sleep_time)

    P_old = P_new[:,0]
    x_old = x_new[:,0]
