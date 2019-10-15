import serial

ser = serial.Serial("/dev/ttyAMA0" , 115200)

ser.write(b'A')
print('LED')