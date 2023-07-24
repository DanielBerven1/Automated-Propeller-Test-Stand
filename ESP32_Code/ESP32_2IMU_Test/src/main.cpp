#include <Arduino.h>

#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define bno_1_I2C_ADDRESS 0x28
#define bno_2_I2C_ADDRESS 0x29

Adafruit_BNO055 bno_1 = Adafruit_BNO055(55, bno_1_I2C_ADDRESS);
Adafruit_BNO055 bno_2 = Adafruit_BNO055(55, bno_2_I2C_ADDRESS);

float offset = 90.0;

void setup(void) 
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno_1.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 1 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  if(!bno_2.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 2 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno_1.setExtCrystalUse(true);
  bno_2.setExtCrystalUse(true);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  imu::Quaternion quat = bno_1.getQuat();
  imu::Vector<3> gravity = bno_1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  // Serial.printf("%3.2f %3.2f %3.2f %3.2f \n", quat.x(), quat.y(), quat.z(), quat.w());
  Serial.printf("%3.2f %3.2f %3.2f \n", gravity.x(), gravity.y(), gravity.z());
  double angle_1 = atan2(gravity.z(), gravity.y())*180/3.14;
  
  // bno_1.getEvent(&event);
  // float z_angle_1 = -1*(event.orientation.z + offset);

  
  /* Display the floating point data */
  // Serial.print("\tZ_1: ");
  // Serial.print(z_angle_1, 4);


  // bno_2.getEvent(&event);
  quat = bno_2.getQuat();
  gravity = bno_2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  Serial.printf("%3.2f %3.2f %3.2f \n", gravity.x(), gravity.y(), gravity.z());
  double angle_2 = (atan2(gravity.z(), gravity.y()) *180/3.14);

  // if (angle_1 > 0){
  //   angle_1 = angle_1 - 180;
  // }
  // else{
  //   angle_1 = angle_1 + 180;

  // }

  angle_1 = angle_1 - 90;

  Serial.println(angle_1);
  Serial.print("    ");
  Serial.println(angle_2);
   
  // Serial.printf("%3.2f %3.2f %3.2f %3.2f \n", quat.x(), quat.y(), quat.z(), quat.w());
  // float z_angle_2 = -1*(event.orientation.z - offset);
  /* Display the floating point data */
  // Serial.print("\tZ_2: ");
  // Serial.println(z_angle_2, 4);
  
//   while (Serial.available() > 0) 
//   {
//     // read the incoming byte:
//     int incomingByte = Serial.read();

//     // say what you got:
//     if (incomingByte == 'p')
//     {      
//       Serial.println(angle_1);
//       Serial.print("    ");
//       Serial.println(angle_2);
//     }

 delay(100);
 } 