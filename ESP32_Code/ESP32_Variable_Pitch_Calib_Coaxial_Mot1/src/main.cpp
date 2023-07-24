/*********
  Subscribed Topics:  DCSetpoint
                      SetDCZero

  Published Topics:   IMU_1
                      IMU_2
                      DC_Position
                      DC_Zero
*********/

#include <Arduino.h>

#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "stdint.h"
#include "math.h"
#include <iostream>
#include <string>
using namespace std;

#include <WiFi.h>
#include <PubSubClient.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Replace the SSID/Password details as per your wifi router
const char* ssid = "TP-Link_692A";
const char* password = "42108368";
const char* IMU_1_topic = "IMU1";
const char* IMU_2_topic = "IMU2";
const char* DC_MOTOR_topic = "DC_Position";
const char* DC_zero_topic = "DC_Zero";
const char* Zero_Switch_topic = "Zero_Switch";

// Replace your MQTT Broker IP address here:
const char* mqtt_server = "10.105.76.221";

WiFiClient espClient;
PubSubClient client(espClient);

#define bno_1_I2C_ADDRESS 0x28
#define bno_2_I2C_ADDRESS 0x29

Adafruit_BNO055 bno_1 = Adafruit_BNO055(55, bno_1_I2C_ADDRESS);
Adafruit_BNO055 bno_2 = Adafruit_BNO055(55, bno_2_I2C_ADDRESS);


long lastMsg = 0;

#define ledPin 2
#define IR_pin 36

double setpoint = 0; 
double current_error = 0;
double current_position = 0;
double c_position;
float offset = 90.0;
float DC_zero = 0;

bool new_setpoint_flag = false; 
bool DC_zero_flag = false;
bool Set_DC_zero_flag = false;
bool Get_Data_flag = false;

bool zero_flag = false;
bool first_zero = true;
bool reset_zero = false;

#define Switch_Pin 33

// variable for storing zero status on rasberry PI
int zero = 0; 

imu::Vector<3> gravity;

// From dc_motors.cpp
double ku_1 = 290.0; double tu_1 = 0.3;
double kp_1=354.5,ki_1=0.0,kd_1=0.1*ku_1*tu_1;

void blink_led(unsigned int times, unsigned int duration){
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(duration);
    digitalWrite(ledPin, LOW); 
    delay(200);
  }
}

// Setup WIFI connection
void setup_wifi() {
  delay(50);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int c=0;
  while (WiFi.status() != WL_CONNECTED) {
    blink_led(2,200); //blink LED twice (for 200ms ON time) to indicate that wifi not connected
    delay(1000); //
    Serial.print(".");
    c=c+1;
    if(c>10){
        ESP.restart(); //restart ESP after 10 seconds
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}


// Connect to mqtt server
void connect_mqttServer() {
  // Loop until we're reconnected
  while (!client.connected()) {

        //first check if connected to wifi
        if(WiFi.status() != WL_CONNECTED){
          //if not connected, then first connect to wifi
          setup_wifi();
        }

        //now attemt to connect to MQTT server
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32_client1", "ubuntu", "asimov7749")) { // Change the name of client here if multiple ESP32 are connected
          //attempt successful
          Serial.println("connected");
          // Subscribe to topics here
          client.subscribe("rpi/DCSetpoint");
          client.subscribe("rpi/SetDCZero");
          client.subscribe("rpi/GetData");
          client.subscribe("rpi/reset_zero");
          //client.subscribe("rpi/xyz"); //subscribe more topics here
          
        } 
        else {
          //attempt not successful
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" trying again in 2 seconds");
    
          blink_led(3,200); //blink LED three times (200ms on duration) to show that MQTT server connection attempt failed
          // Wait 2 seconds before retrying
          delay(2000);
        }
  }
  
}

//this function will be executed whenever there is data available on subscribed topics
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  string messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
    
  }
  Serial.println();

  // Check if a message is received on the topic "rpi/broadcast"
  if (String(topic) == "rpi/DCSetpoint") {

    
    
    // Convert message string into type double
    setpoint = stod(messageTemp);

    // Set DC motor setpoint
    setSetpoint1(setpoint);
    current_error=getError1();
    c_position = getPosition1();
    D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(c_position); D_print("    ERR:"); D_print(current_error); D_println("");
    // set new message flag to TRUE
    new_setpoint_flag = true;

  //Similarly add more if statements to check for other subscribed topics 
  }

  if (String(topic) == "rpi/SetDCZero") {


    Set_DC_zero_flag = true;

  //Similarly add more if statements to check for other subscribed topics 
  }

  if (String(topic) == "rpi/GetData"){

    Get_Data_flag = true;
    
  }

  if (String(topic) == "rpi/reset_zero"){

    reset_zero = true;


  }

}

void publish_IMU_1(String angle_1){
  // PUBLISH to the MQTT Broker (topic = IMU_1)
  if (client.publish(IMU_1_topic, String(angle_1).c_str())) {
    Serial.println("Angle 1 sent!");
  }
  else {
    Serial.println("Angle failed to send. Reconnecting to MQTT Broker and trying again");
    connect_mqttServer();
    delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
    client.publish(IMU_1_topic, String(angle_1).c_str());
  }
}

void publish_IMU_2(String angle_2){
  // PUBLISH to the MQTT Broker (topic = IMU_1)
  if (client.publish(IMU_2_topic, String(angle_2).c_str())) {
    Serial.println("Angle 2 sent!");
  }
  else {
    Serial.println("Angle failed to send. Reconnecting to MQTT Broker and trying again");
    connect_mqttServer();
    delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
    client.publish(IMU_2_topic, String(angle_2).c_str());
  }
}

void publish_DC_Position(String position){
  // PUBLISH to the MQTT Broker (topic = DC_Position)
  if (client.publish(DC_MOTOR_topic, String(position).c_str())) {
    Serial.println("DC Position sent!");
  }
  else {
    Serial.println("DC Position failed to send. Reconnecting to MQTT Broker and trying again");
    connect_mqttServer;
    delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
    client.publish(DC_MOTOR_topic, String(current_position).c_str());
  }
}

void publish_DC_zero_state(String DC_zero){
// PUBLISH to the MQTT Broker (topic = DC_zero)
    if (client.publish(DC_zero_topic, String(DC_zero).c_str())) {
      Serial.println("DC zero state sent!");
    }
    else {
      Serial.println("DC zero state failed to send. Reconnecting to MQTT Broker and trying again");
      connect_mqttServer();
      delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
      client.publish(DC_zero_topic, String(DC_zero).c_str());
    }
}


void publish_Limit_Switch_Zero(String limit_zero){
    if (client.publish(Zero_Switch_topic, String(limit_zero).c_str())) {
      Serial.println("zero notifier sent!");
    }
    else {
        Serial.println("zero failed to send. Reconnecting to MQTT Broker and trying again");
        connect_mqttServer();
        delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
        client.publish(Zero_Switch_topic, String(limit_zero).c_str());
    }


  }




void setup() {

  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server,1883);//1883 is the default port for MQTT server
  client.setCallback(callback);


  setupDCMotors();

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

  if (!client.connected()) {
    connect_mqttServer();
  }
  
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno_1.getEvent(&event);
  
  // Wait for sensor to read non-zero values upon startup
  float test_angle = event.orientation.z;
  while (test_angle == 0){
    /* Get a new sensor event */ 
    bno_1.getEvent(&event);
    test_angle = event.orientation.z;

  }

  bno_2.getEvent(&event);

  // Wait for sensor to read non-zero values upon startup
  test_angle = event.orientation.z;
  while (test_angle == 0){
    /* Get a new sensor event */ 
    bno_2.getEvent(&event);
    test_angle = event.orientation.z;

  }

  // // send initial readings
  // gravity = bno_1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  // double pitch_angle_1 = atan2(gravity.z(), gravity.y())*57.324841;


  // gravity = bno_2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  // double pitch_angle_2 = atan2(gravity.z(), gravity.y())*57.324841;

  // if (pitch_angle_1 > 0){
  //   pitch_angle_1 = pitch_angle_1 - 180;
  // }
  // else{
  //   pitch_angle_1 = pitch_angle_1 + 180;
  // }


  // current_position = getPosition1();

  // String angle_1 = String((float)pitch_angle_1, 8);  
  // String angle_2 = String((float)pitch_angle_2, 8); 
  // String position = String((float)current_position);
  // String DC_zero_state = String((float)DC_zero);


  // publish_IMU_1(angle_1);
  // delay(1000);
  // publish_IMU_2(angle_2);
  // delay(1000);
  // publish_DC_Position(position);
  // delay(1000);
  // publish_DC_zero_state(DC_zero_state);
}

void loop() {
  
  if (!client.connected()) {
    connect_mqttServer();
  }

  long now = millis();
  if (now - lastMsg > 4000) {
    lastMsg = now;
    
  }
  current_error = getError1();
  current_position = getPosition1();
  /* Get a new sensor event */ 
  // send initial readings
  gravity = bno_1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  double pitch_angle_1 = atan2(gravity.z(), gravity.y())*57.324841;


  gravity = bno_2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  double pitch_angle_2 = atan2(gravity.z(), gravity.y())*57.324841;

  if (pitch_angle_1 > 0){
    pitch_angle_1 = pitch_angle_1 - 180;
  }
  else{
    pitch_angle_1 = pitch_angle_1 + 180;
  }

  // pitch_angle_1 = pitch_angle_1 - 90;

  if (pitch_angle_1 >= 0 and pitch_angle_2 >= 0){
    if (pitch_angle_1 >= 20.0 or pitch_angle_2 >= 20.0){
      setSetpoint1(0);
      setPIDgains1(0, 0, 0);
    }
    
  }

  if (pitch_angle_1 <= 0 or pitch_angle_2 <= 0){
    if (pitch_angle_1 <= -18.0 or pitch_angle_2 <= -18.0){
      setSetpoint1(0);
      setPIDgains1(0, 0, 0);
    }
  }


  if (Set_DC_zero_flag == true and analogRead(IR_pin) < 3000){
    
    setPIDgains1(0, 0, 0);
    setPosition1_zero();
    setSetpoint1(0);
    setPIDgains1(kp_1, ki_1, kd_1);

    new_setpoint_flag = false;
    DC_zero_flag = true;
    Set_DC_zero_flag = false;
    current_error = getError1();
    current_position = getPosition1();
    DC_zero = 1;
  }
  



  D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(current_position); D_print("    ERR:"); D_print(current_error); 
  D_print("    Pitch_1:"); D_print(pitch_angle_1); D_print("    Pitch_2:"); D_print(pitch_angle_2); D_println("");
  D_print("    IR:"); D_print(analogRead(IR_pin)); D_println("");
  
  
  if (DC_zero_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Zero'd");
    

    /* Get a new sensor event */ 
    // send initial readings
    gravity = bno_1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double pitch_angle_1 = atan2(gravity.z(), gravity.y())*57.324841;


    gravity = bno_2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double pitch_angle_2 = atan2(gravity.z(), gravity.y())*57.324841;

    if (pitch_angle_1 > 0){
      pitch_angle_1 = pitch_angle_1 - 180;
    }
    else{
      pitch_angle_1 = pitch_angle_1 + 180;

    }

    // pitch_angle_1 = pitch_angle_1 - 90;

    current_position = getPosition1();

    String angle_1 = String((float)pitch_angle_1, 8);  
    String angle_2 = String((float)pitch_angle_2, 8); 
    String position = String((float)current_position);
    String DC_zero_state = String((int)DC_zero);
    // D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(current_error); D_println("");
    
    // PUBLISH to the MQTT Broker (topic = DC_zero)
    publish_DC_zero_state(DC_zero_state);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = IMU_1)
    publish_IMU_1(angle_1);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = IMU_1)
    publish_IMU_2(angle_2);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = Position)
    publish_DC_Position(position);
    delay(1000);

    DC_zero_flag = false;
    

  }

  
  
  
  if (abs(current_error) < 5.0 and new_setpoint_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Setpoint Reached");
    

        /* Get a new sensor event */ 
    // send initial readings
    gravity = bno_1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double pitch_angle_1 = atan2(gravity.z(), gravity.y())*57.324841;


    gravity = bno_2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double pitch_angle_2 = atan2(gravity.z(), gravity.y())*57.324841;

    if (pitch_angle_1 > 0){
      pitch_angle_1 = pitch_angle_1 - 180;
    }
    else{
      pitch_angle_1 = pitch_angle_1 + 180;

    }

    // pitch_angle_1 = pitch_angle_1 - 90;

    current_position = getPosition1();

    String angle_1 = String((float)pitch_angle_1, 8);  
    String angle_2 = String((float)pitch_angle_2, 8); 
    String position = String((float)current_position);

    // D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(current_error); D_println("");
    
    // PUBLISH to the MQTT Broker (topic = IMU_1)
    publish_IMU_1(angle_1);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = IMU_1)
    publish_IMU_2(angle_2);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = Position)
    publish_DC_Position(position);
    delay(1000);

    new_setpoint_flag = false;

  }
  

  if(Get_Data_flag == true){

    // send initial readings
    gravity = bno_1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double pitch_angle_1 = atan2(gravity.z(), gravity.y())*57.324841;


    gravity = bno_2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double pitch_angle_2 = atan2(gravity.z(), gravity.y())*57.324841;

    if (pitch_angle_1 > 0){
      pitch_angle_1 = pitch_angle_1 - 180;
    }
    else{
      pitch_angle_1 = pitch_angle_1 + 180;
    }

    // pitch_angle_1 = pitch_angle_1 - 90;


    current_position = getPosition1();

    String angle_1 = String((float)pitch_angle_1, 8);  
    String angle_2 = String((float)pitch_angle_2, 8); 
    String position = String((float)current_position);
    String DC_zero_state = String((float)DC_zero);


    publish_IMU_1(angle_1);
    delay(1000);
    publish_IMU_2(angle_2);
    delay(1000);
    publish_DC_Position(position);
    delay(1000);
    publish_DC_zero_state(DC_zero_state);

    Get_Data_flag = false;


  }

  if (digitalRead(Switch_Pin)==HIGH and first_zero == true){


    Serial.println("zero set");
    zero = 1;

    // set the zero flag to true
    zero_flag = true;
    first_zero = false;
  

    
  }

  // if the zero switch has been hit
  if (zero_flag == true){



    publish_Limit_Switch_Zero(String(zero));
  
    // set zero flag backto false and wait for zero to be hit again
    zero_flag = false;

  }



  if (reset_zero == true){

    zero = 0;
    first_zero = true;
    reset_zero = false;

  }

  client.loop();

}