import serial
s = serial.Serial(port='COM15', baudrate=115200)
s.flush()
s.write(bytes(str('m'),'ascii')) # enter mode select
s.write(bytes(str('3'),'ascii')) # select uart mode
s.write(bytes(str('5'),'ascii')) # 9600 baudrate
#s.write(bytes(b'1')) # parity
#s.write(bytes(b'1')) # 1 stop bit
#s.write(bytes(b'1')) # "set 1 == idle level"
#s.write(bytes(b'2')) # output mode level == normal
#s.write(bytes(b'(1)'))
#s.write(bytes(b'y'))

s.flush()
print('written successfully')
cycles = 0
while s.is_open:
    print(cycles)
    res = s.read(1)
    print(res)
    cycles+=1
print('port closed')