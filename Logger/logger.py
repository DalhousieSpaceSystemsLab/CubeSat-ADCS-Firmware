import serial
s = serial.Serial(port='COM15', baudrate=115200)
s.write(bytes(b'm')) # enter mode select

s.write(bytes(b'3')) # select uart mode
s.write(bytes(b'5')) # 9600 baudrate
s.write(bytes(b'1')) # parity
s.write(bytes(b'1')) # 1 stop bit
s.write(bytes(b'1')) # "set 1 == idle level"
s.write(bytes(b'2')) # output mode level == normal
s.write(bytes(b'['))
#s.write(bytes(b'o')) # configure output mode
#s.write(bytes(b'4')) # output serialized usb data as ascii representation

print('written successfully')
cycles = 0
while s.is_open:
    print(cycles)
    res = s.read(10)
    print(res)
    cycles+=1
print('port closed')