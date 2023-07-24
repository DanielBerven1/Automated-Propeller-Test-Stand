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




void setup(void) 
{
  Serial.begin(115200);
  pinMode(36, INPUT);
  pinMode(39, INPUT);

}

void loop(void) 
{

  Serial.print("IR1:"); Serial.print(" "); Serial.print(analogRead(36)); Serial.print("IR2:"); Serial.print(" "); Serial.println(analogRead(39)); 
  
  delay(100);
}