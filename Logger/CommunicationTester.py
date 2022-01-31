import serial
import time
import datetime

s = serial.Serial(port='COM8', baudrate=115200)
s.flush()
while s.is_open:
    inputchar = input("Enter r for gyro and magnetometer readings.\nEnter w to change magnetorquer values: ")
    if inputchar == 'r':
        print("Sending ""read"" to MSP430")
        s.write(bytes("read!", 'ascii'));
    elif inputchar == 'w':
        print("Sending ""write"" to MSP430")
        s.write(bytes("write!", 'ascii'));
    else:
        print("Invalid\n")
        
        
    try :
        print("Reading String...\n")
        print(s.readline().decode('UTF-8'))
    except:
        print("Error\n")
