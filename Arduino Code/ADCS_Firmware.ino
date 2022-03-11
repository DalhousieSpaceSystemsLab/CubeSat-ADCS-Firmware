#define HWSERIAL Serial1
#include <string.h>
#include <DFRobot_BMX160.h>
#define MAX_VOLTAGE 5.0

#define XFIN 7
#define XRIN 6

#define YFIN 5
#define YRIN 4

#define ZFIN 3
#define ZRIN 2

DFRobot_BMX160 bmx160;
const int8_t i2c_addr = 0x69;
int ADC_CLK = 13;  // SPI master out/slave in pin assignment
int ADC_MISO = 12; // SPI master in/slave out pin assignment
int ADC_MOSI = 11; // SPI slave select/chip select pin assignment
int ADC_CS = 9;   // SPI serial clock pin assignment
const byte ch0_code = 0b10010100; // ADC channel 0: used by LM20BIM7 temperature sensor
const byte ch1_code = 0b11010100; // ADC channel 1: used by SFH-2430 photodiode (240 degree configuration)
const byte ch2_code = 0b10100100; // ADC channel 2: used by SFH-2430 photodiode   (0 degree configuration)
const byte ch3_code = 0b11100100; // ADC channel 3: used by SFH-2430 photodiode (120 degree configuration)



void setup(){
 Serial.begin(115200);
 Serial1.begin(115200);
 
 pinMode(XFIN, OUTPUT);
 pinMode(XRIN, OUTPUT);
 pinMode(YFIN, OUTPUT);
 pinMode(YRIN, OUTPUT);
 pinMode(ZFIN, OUTPUT);
 pinMode(ZRIN, OUTPUT);
 
 pinMode(ADC_CS, OUTPUT);
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
 String receivedString = Serial.readString();
 Serial.println(receivedString);

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

 int Omagnx, Omagny, Omagnz, Ogyrox, Ogyroy, Ogyroz;
 String gyroString;
 float XVoltage, YVoltage, ZVoltage;

 //Check for different commands
 if(receivedString == "{\"imu\" : \"omega\"}\n"){
  //Print gyro readings
  gyroString = Serial1.readString();
  Serial1.flush();
  Serial1.readString();
  Serial1.readString();
  sscanf(gyroString.c_str(), "IMU:[%d %d %d %d %d %d]", &Omagnx, &Omagny, &Omagnz, &Ogyrox, &Ogyroy, &Ogyroz);
  Serial.print("{\"imu\" : [ ");
  Serial.print(Ogyrox);
  Serial.print(", ");
  Serial.print(Ogyroy);
  Serial.print(", ");
  Serial.print(Ogyroz);
  Serial.print("] }");
 }
 else if(receivedString == "{\"sunSen\": \"read\", \"face\":\"z+\"}\n"){
  //Turn all other sun sensor CS to high and this one to low
  //Print sunsen readings
  Serial.print("{\"sunSen\" : \"z+\", \"Raw value from 0 to 2^12\": [");
  Serial.print(SS0deg_volt);
  Serial.print(", ");
  Serial.print(SS120deg_volt);
  Serial.print(", ");
  Serial.print(SS240deg_volt);
  Serial.print("], \"temp\" : ");
  Serial.print(temp_val);
  Serial.print("}");
 }
 else if(receivedString == "{\"magSen\" : \"read\"}\n"){
  //Print magn readings
  gyroString = Serial1.readString();
  sscanf(gyroString.c_str(), "IMU:[%d %d %d %d %d %d]", &Omagnx, &Omagny, &Omagnz, &Ogyrox, &Ogyroy, &Ogyroz);
  Serial1.flush();
  Serial1.readString();
  Serial1.readString();
  Serial.print("{\"magsen\" : [\"uT\", ");
  Serial.print(Omagnx);
  Serial.print(", ");
  Serial.print(Omagny);
  Serial.print(", ");
  Serial.print(Omagnz);
  Serial.print("]}");
 }
 else if(receivedString.indexOf("\"mqtr_volts\" : \"write\"") > 0){
  //Needs some work
  sscanf(receivedString.c_str(),"{\"mqtr_volts\" : \"write\", \"value\" : [ %f, %f, %f]}", &XVoltage, &YVoltage, &ZVoltage);
  if(XVoltage < 0){
    analogWrite(XFIN,255);
    analogWrite(XRIN,(1.0 - (XVoltage / MAX_VOLTAGE)) * 255);
  }
  else{
    analogWrite(XFIN,(1.0 - (XVoltage / MAX_VOLTAGE)) * 255);
    analogWrite(XRIN, 255);
  }
  if(YVoltage < 0){
    analogWrite(YFIN,255);
    analogWrite(YRIN,(1.0 - (YVoltage / MAX_VOLTAGE)) * 255);
  }
  else{
    analogWrite(YFIN,(1.0 - (YVoltage / MAX_VOLTAGE)) * 255);
    analogWrite(YRIN, 255);
  }
  if(ZVoltage < 0){
    analogWrite(ZFIN, 255);
    analogWrite(ZRIN,(1.0 - (ZVoltage / MAX_VOLTAGE)) * 255);
  }
  else{
    analogWrite(ZFIN, (1.0 - (ZVoltage / MAX_VOLTAGE)) * 255);
    analogWrite(ZRIN, 255);
  }
  Serial.print("{\"mqtr_volts\" : \"set\"}");

 }
}
unsigned int ads7841(const byte control) {
  int bitnum;                  // return value
  digitalWrite(SS, LOW);       // activate ADS7841
  SPI.transfer(control);       // transfer control byte
  byte msb = SPI.transfer(0);  // read MSB & LSB
  byte lsb = SPI.transfer(0);
  digitalWrite(SS, HIGH);      // deactivate ADS7841
  msb = msb & 0x7F;            // isolate readings and form final reading
  lsb = lsb >> 3;
  bitnum = ((word)msb << 5) | lsb;
  return bitnum;
}
