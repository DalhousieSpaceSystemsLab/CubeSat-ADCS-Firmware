

#include <DFRobot_BMX160.h>
DFRobot_BMX160 bmx160;
const int8_t i2c_addr = 0x69;
void setup(){
 Serial.begin(115200);
 delay(100);
 //init the hardware bmx160
 while (bmx160.begin() != true){
  Serial.print("init false");
 }
}
void loop(){
 bmx160SensorData Omagn, Ogyro, Oaccel;
 /* Get a new sensor event */
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
 /* Display the magnetometer results (magn is magnetometer in uTesla) */
 Serial.print("IMU:[ ");
 Serial.print(Omagn.x); Serial.print(" ");
 Serial.print(Omagn.y); Serial.print(" ");
 Serial.print(Omagn.z); Serial.print(" ");
 
 /* Display the gyroscope results (gyroscope data is in g)*/

 Serial.print(Ogyro.x); Serial.print(" ");
 Serial.print(Ogyro.y); Serial.print(" ");
 Serial.print(Ogyro.z); Serial.print(" ]\n");
 

 delay(500);
}
