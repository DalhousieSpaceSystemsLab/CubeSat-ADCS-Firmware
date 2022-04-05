#define HWSERIAL Serial1
#define IMX6SERIAL Serial2

//Set these

#include <string.h>
#include <DFRobot_BMX160.h>
#define MAX_VOLTAGE 5.0

#define XFIN 2
#define XRIN 3

#define YFIN 4
#define YRIN 5

#define ZFIN 6
#define ZRIN 9

DFRobot_BMX160 bmx160;
const int8_t i2c_addr = 0x69;
int ADC_CLK = 13;  // SPI master out/slave in pin assignment
int ADC_MISO = 12; // SPI master in/slave out pin assignment
int ADC_MOSI = 11; // SPI slave select/chip select pin assignment
#define XplusSS 37
#define XminusSS 36
#define YplusSS 35
#define YminusSS 34
#define ZplusSS 38
#define ZminusSS 33
const byte ch0_code = 0b10010100; // ADC channel 0: used by LM20BIM7 temperature sensor
const byte ch1_code = 0b11010100; // ADC channel 1: used by SFH-2430 photodiode (240 degree configuration)
const byte ch2_code = 0b10100100; // ADC channel 2: used by SFH-2430 photodiode   (0 degree configuration)
const byte ch3_code = 0b11100100; // ADC channel 3: used by SFH-2430 photodiode (120 degree configuration)

int setSS = 38;

void setup(){
 IMX6SERIAL.begin(115200);
 HWSERIAL.begin(115200);
 
 pinMode(XFIN, OUTPUT);
 pinMode(XRIN, OUTPUT);
 pinMode(YFIN, OUTPUT);
 pinMode(YRIN, OUTPUT);
 pinMode(ZFIN, OUTPUT);
 pinMode(ZRIN, OUTPUT);
 
 pinMode(XplusSS, OUTPUT);
 pinMode(XminusSS, OUTPUT);
 pinMode(YplusSS, OUTPUT);
 pinMode(YminusSS, OUTPUT);
 pinMode(ZplusSS, OUTPUT);
 pinMode(ZminusSS, OUTPUT);
 pinMode(ADC_MOSI, OUTPUT);
 pinMode(ADC_MISO, INPUT);
 pinMode(ADC_CLK, OUTPUT);
 SPI.begin();
 SPI.setBitOrder(MSBFIRST);
 SPI.setDataMode(SPI_MODE0);
 SPI.setClockDivider(SPI_CLOCK_DIV16);
 delay(100);
 
}
void loop(){
  //Get string from IMX6
 String receivedString = IMX6SERIAL.readString();
 IMX6SERIAL.println(receivedString);
 if(receivedString.indexOf("sunSen") > 0){
  if(receivedString.indexOf("x+")){
    setSS = XplusSS;
  }
  else if(receivedString.indexOf("x-")){
    setSS = XminusSS;
  }
  else if(receivedString.indexOf("y+")){
    setSS = YplusSS;
  }
  else if(receivedString.indexOf("y-")){
    setSS = YminusSS;
  }
  else if(receivedString.indexOf("z+")){
    setSS = ZplusSS;
  }
  else if(receivedString.indexOf("z-")){
    setSS = ZminusSS;
  }
 } 
 //Get sunsen data
 int temp_bit = ads7841(ch0_code); // raw bit reading
 float temp_volt = 3.3*(temp_bit/4096.0); // bit-to-voltage conversion
 float temp_val = 153.857 + (-73.154*temp_volt) + (-2.3654*temp_volt*temp_volt);
 int SS0deg_bit = ads7841(ch2_code);
 float SS0deg_volt = 3.3*(SS0deg_bit/4096.0);
 int SS120deg_bit = ads7841(ch3_code);
 float SS120deg_volt = 3.3*(SS120deg_bit/4096.0);
 int SS240deg_bit = ads7841(ch1_code);
 float SS240deg_volt = 3.3*(SS240deg_bit/4096.0);

 int Omagnx, Omagny, Omagnz, Ogyrox, Ogyroy, Ogyroz, Oaccelx, Oaccely, Oaccelz;
 String gyroString;
 float XVoltage, YVoltage, ZVoltage;

 //Check for different commands
 if(receivedString == "{\"imu\" : \"omega\"}\n"){
  //Print gyro readings
  gyroString = HWSERIAL.readString();
  HWSERIAL.flush();
  HWSERIAL.readString();
  HWSERIAL.readString();
  sscanf(gyroString.c_str(), "IMU:[%d %d %d %d %d %d %d %d %d]", &Omagnx, &Omagny, &Omagnz, &Ogyrox, &Ogyroy, &Ogyroz, &Oaccelx, &Oaccely, &Oaccelz);
  IMX6SERIAL.print("{\"imu\" : [ ");
  IMX6SERIAL.print(Ogyrox);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(Ogyroy);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(Ogyroz);
  IMX6SERIAL.print("] }");
 }
 else if(receivedString.indexOf("sunSen") > 0){
  //Chosen sun sensor has already been specified and read
  IMX6SERIAL.print("{\"sunSen\" : \"z+\", \"Raw value from 0 to 2^12\": [");
  IMX6SERIAL.print(SS0deg_volt);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(SS120deg_volt);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(SS240deg_volt);
  if((receivedString.indexOf("z+") > 0) && (receivedString.indexOf("z-") > 0)){
    IMX6SERIAL.print("], \"temp\" : ");
    IMX6SERIAL.print(temp_val);
    IMX6SERIAL.print("}");
  }
  else{
    IMX6SERIAL.print("]}");
  }
 }
 else if(receivedString == "{\"magSen\" : \"read\"}\n"){
  //Print magn readings
  gyroString = HWSERIAL.readString();
  sscanf(gyroString.c_str(), "IMU:[%d %d %d %d %d %d %d %d %d]", &Omagnx, &Omagny, &Omagnz, &Ogyrox, &Ogyroy, &Ogyroz, &Oaccelx, &Oaccely, &Oaccelz);
  HWSERIAL.flush();
  HWSERIAL.readString();
  HWSERIAL.readString();
  IMX6SERIAL.print("{\"magsen\" : [\"uT\", ");
  IMX6SERIAL.print(Omagnx);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(Omagny);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(Omagnz);
  IMX6SERIAL.print("]}");
 }
 else if(receivedString == "{\"imu\" : \"accel\"}"){
  sscanf(gyroString.c_str(), "IMU:[%d %d %d %d %d %d %d %d %d]", &Omagnx, &Omagny, &Omagnz, &Ogyrox, &Ogyroy, &Ogyroz, &Oaccelx, &Oaccely, &Oaccelz);
  HWSERIAL.flush();
  HWSERIAL.readString();
  HWSERIAL.readString();
  IMX6SERIAL.print("{\"accel\" : [ ");
  IMX6SERIAL.print(Oaccelx);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(Oaccely);
  IMX6SERIAL.print(", ");
  IMX6SERIAL.print(Oaccelz);
  IMX6SERIAL.print("]}");
 }
 else if(receivedString.indexOf("\"mqtr_volts\" : \"write\"") > 0){
  //Needs some work
  sscanf(receivedString.c_str(),"{\"mqtr_volts\" : \"write\", \"value\" : [ %f, %f, %f]}", &XVoltage, &YVoltage, &ZVoltage);
  if(XVoltage < 0){
    analogWrite(XRIN,((XVoltage / MAX_VOLTAGE)) * 255);
  }
  else{
    analogWrite(XFIN,((XVoltage / MAX_VOLTAGE)) * 255);
  }
  if(YVoltage < 0){
    analogWrite(YRIN,((YVoltage / MAX_VOLTAGE)) * 255);
  }
  else{
    analogWrite(YFIN,((YVoltage / MAX_VOLTAGE)) * 255);
  }
  if(ZVoltage < 0){
    analogWrite(ZRIN,((ZVoltage / MAX_VOLTAGE)) * 255);
  }
  else{
    analogWrite(ZFIN, ((ZVoltage / MAX_VOLTAGE)) * 255);
  }
  IMX6SERIAL.print("{\"mqtr_volts\" : \"set\"}");
}
}
unsigned int ads7841(const byte control) {
  int bitnum;                  // return value
  digitalWrite(setSS, LOW);       // activate ADS7841
  SPI.transfer(control);       // transfer control byte
  byte msb = SPI.transfer(0);  // read MSB & LSB
  byte lsb = SPI.transfer(0);
  digitalWrite(setSS, HIGH);      // deactivate ADS7841
  msb = msb & 0x7F;            // isolate readings and form final reading
  lsb = lsb >> 3;
  bitnum = ((word)msb << 5) | lsb;
  return bitnum;
}
