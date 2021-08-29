import serial
import time
import datetime

s = serial.Serial(port='COM15', baudrate=115200)
s.flush()
s.write(bytes(str('m') + '\n','ascii')) # enter mode select
time.sleep(0.1)
s.write(bytes(str('3') + '\n','ascii')) # select uart mode
time.sleep(0.1)
s.write(bytes('5' + '\n','ascii')) # 9600 baudrate
time.sleep(0.1)
s.write(bytes('1' + '\n','ascii')) # parity
time.sleep(0.1)
s.write(bytes('1' + '\n','ascii')) # 1 stop bit
time.sleep(0.1)
s.write(bytes('1' + '\n','ascii')) # "set 1 == idle level"
time.sleep(0.1)
s.write(bytes('2' + '\n','ascii')) # output mode level == normal
time.sleep(0.1)
s.write(bytes('(1)' + '\n','ascii'))
time.sleep(0.1)
s.write(bytes('y' + '\n','ascii'))
time.sleep(0.1)
print("Finished writing\n")

f = open("log12hour.txt", "w")

while s.is_open:
    try:
        timestamp = datetime.datetime.now().strftime("[ %H : %M : %S : %f ]")
        data = s.readline()
        try:
            data = data.decode('UTF-8')
            f.write(f"{timestamp} : {data}")
            print(data)
        except:
           f.write(f"{timestamp} : ERROR\n")
    except KeyboardInterrupt:
        f.close()
        break
print('port closed')


