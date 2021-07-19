import serial
s = serial.Serial(port='COM15', timeout=None, baudrate=9600, xonxoff=False, rtscts=False, dsrdtr=False) 
s.flushInput()
while 1:
    res = s.read()
    print(res)
    
    #Note: This software does not properly read UART. When given the correct port all it does is return 0xff