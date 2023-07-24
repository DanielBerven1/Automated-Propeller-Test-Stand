/*********
  Subscribed Topics:  PitchSetpoint
                      GetPitchData
                      SetDCZero

  Published Topics:   DC_Position
                      DC_Zero
*********/

#include <Arduino.h>

#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "stdint.h"

#include <iostream>
#include <string>
using namespace std;

#include <WiFi.h>
#include <PubSubClient.h>


// MQTT variables
// Replace the SSID/Password details as per your wifi router
const char* ssid = "TP-Link_692A";
const char* password = "42108368";

// Published Topics 
const char* Front_DC_MOTOR_topic = "DC_Front_Position";
const char* Front_DC_zero_topic = "DC_Front_Zero";
const char* Back_DC_MOTOR_topic = "DC_Back_Position";
const char* Back_DC_zero_topic = "DC_Back_Zero";
const char* Dynamixel_Zero_Switch_topic = "Zero_Switch";

// Replace your MQTT Broker IP address here:
const char* mqtt_server = "10.105.76.221";

WiFiClient espClient;
PubSubClient client(espClient);



long lastMsg = 0;


#define ledPin 2
#define Front_IR_pin 36
#define Back_IR_pin 39


#define Switch_Pin 33



// Front DC motor variables // 
double setpoint_1 = 0; 
double current_error_1 = 0;
double current_position_1 = 0;
int Front_DC_zero = 0;

// Back DC motor 2 variables //
double setpoint_2 = 0; 
double current_error_2 = 0;
double current_position_2 = 0;
int Back_DC_zero = 0;

// Dynamixel zero switch //
int Dynamixel_Zero = 0; // variable for storing zero status on rasberry PI


// Front motor opic flags //
bool Front_new_setpoint_flag = false; 
bool Front_DC_zero_flag = false;
bool Front_Set_DC_zero_flag = false;

// Back motor opic flags //
bool Back_new_setpoint_flag = false; 
bool Back_DC_zero_flag = false;
bool Back_Set_DC_zero_flag = false;

// Dynamixel zero switch Flags //
bool zero_flag = false;
bool DynamixelFirstZero = true;
bool ResetDynamixelZero_flag = true;

// flag for setting Dynamixel Zeroing Routine //
bool SetDynamixelZero_flag = false;

// flag for accessing data //
bool Get_Data = false;


// PID Gains From dc_motors.cpp //
double ku_1 = 290.0; double tu_1 = 0.3;
double kp_1=354.5,ki_1=0.0,kd_1=0.1*ku_1*tu_1;

double ku_2 = ku_1; double tu_2 = tu_1;
double kp_2=kp_1,ki_2=ki_1,kd_2=kd_1;



// Setup WIFI connection //
void setup_wifi() {
  delay(50);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int c=0;
  while (WiFi.status() != WL_CONNECTED) {
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


// Connect to mqtt server //
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

          // Front motor Topics //
          client.subscribe("rpi/FrontPitchSetpoint");
          client.subscribe("rpi/FrontSetDCZero");

          // Back motor topics //
          client.subscribe("rpi/BackPitchSetpoint");
          client.subscribe("rpi/BackSetDCZero");

          // Dynamixel Zero Topic //
          client.subscribe("rpi/SetDynamixelZero");
          client.subscribe("rpi/ResetDynamixelZero");


          // Data access topic // 
          client.subscribe("rpi/GetData");

          //client.subscribe("rpi/xyz"); //subscribe more topics here
          
        } 
        else {
          //attempt not successful
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" trying again in 2 seconds");
  
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

  // Check if a message is received on the topic "rpi/FrontPitchSetpoint"
  if (String(topic) == "rpi/FrontPitchSetpoint") {

    // Convert message string into type double
    setpoint_1 = stod(messageTemp);

    // Set DC motor setpoint
    setSetpoint1(setpoint_1);
    
    
    // set new message flag to TRUE
    Front_new_setpoint_flag = true;

  }
  // Check if a message is received on the topic "rpi/FrontSetDCZero"
  if (String(topic) == "rpi/FrontSetDCZero") {


    Front_Set_DC_zero_flag = true;

  //Similarly add more if statements to check for other subscribed topics 
  }


    // Check if a message is received on the topic "rpi/BackPitchSetpoint"
  if (String(topic) == "rpi/BackPitchSetpoint") {

    // Convert message string into type double
    setpoint_2 = stod(messageTemp);

    // Set DC motor setpoint
    setSetpoint2(setpoint_2);
    
    
    // set new message flag to TRUE
    Back_new_setpoint_flag = true;

  }
  // Check if a message is received on the topic "rpi/BackSetDCZero"
  if (String(topic) == "rpi/BackSetDCZero") {


    Back_Set_DC_zero_flag = true;


  }

  // Check if a message is received on the topic "rpi/SetDynamixelZero"
  if (String(topic) == "rpi/SetDynamixelZero") {

    SetDynamixelZero_flag = true;

  }

  // Check if a message is received on the topic "rpi/ResetDynamixelZero"
  if (String(topic) == "rpi/ResetDynamixelZero") {

    ResetDynamixelZero_flag = true;

  }  


  // Check if a message is received on the topic "rpi/Get_Data"
  if (String(topic) == "rpi/GetData") {

    Get_Data = true;
    
  }
}

void publish_Front_DC_Position(String position){
  // PUBLISH to the MQTT Broker (topic = DC_Position)
  if (client.publish(Front_DC_MOTOR_topic, String(position).c_str())) {
    Serial.println("DC Position sent!");
  }
  else {
    Serial.println("DC Position failed to send. Reconnecting to MQTT Broker and trying again");
    connect_mqttServer;
    delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
    client.publish(Front_DC_MOTOR_topic, String(position).c_str());
  }
}

void publish_Front_DC_zero_state(String DC_zero){
// PUBLISH to the MQTT Broker (topic = DC_zero)
    if (client.publish(Front_DC_zero_topic, String(DC_zero).c_str())) {
      Serial.println("DC zero state sent!");
    }
    else {
      Serial.println("DC zero state failed to send. Reconnecting to MQTT Broker and trying again");
      connect_mqttServer();
      delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
      client.publish(Front_DC_zero_topic, String(DC_zero).c_str());
    }
}

void publish_Back_DC_Position(String position){
  // PUBLISH to the MQTT Broker (topic = DC_Position)
  if (client.publish(Back_DC_MOTOR_topic, String(position).c_str())) {
    Serial.println("DC Position sent!");
  }
  else {
    Serial.println("DC Position failed to send. Reconnecting to MQTT Broker and trying again");
    connect_mqttServer;
    delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
    client.publish(Back_DC_MOTOR_topic, String(position).c_str());
  }
}

void publish_Back_DC_zero_state(String DC_zero){
// PUBLISH to the MQTT Broker (topic = DC_zero)
    if (client.publish(Back_DC_zero_topic, String(DC_zero).c_str())) {
      Serial.println("DC zero state sent!");
    }
    else {
      Serial.println("DC zero state failed to send. Reconnecting to MQTT Broker and trying again");
      connect_mqttServer();
      delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
      client.publish(Back_DC_zero_topic, String(DC_zero).c_str());
    }
}

void publish_Zero_Switch(String limit_zero){
    if (client.publish(Dynamixel_Zero_Switch_topic, String(limit_zero).c_str())) {
      Serial.println("zero notifier sent!");
    }
    else {
        Serial.println("zero failed to send. Reconnecting to MQTT Broker and trying again");
        connect_mqttServer();
        delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
        client.publish(Dynamixel_Zero_Switch_topic, String(limit_zero).c_str());
    }


  }



void setup() {

  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server,1883);//1883 is the default port for MQTT server
  client.setCallback(callback);

  setupDCMotors();
  
  if (!client.connected()) {
    connect_mqttServer();
  }

  pinMode(Switch_Pin, INPUT);



}

void loop() {
  
  /////////////////// Loop Iterations ////////////////////////////////
  if (!client.connected()) {
    connect_mqttServer();
  }

  long now = millis();
  if (now - lastMsg > 4000) {
    lastMsg = now;
    
  }

  current_error_1 = getError1();
  current_position_1 = getPosition1();
  current_error_2 = getError2();
  current_position_2 = getPosition2();

  // Needed for some reason (DONT COMMENT OUT!)
  D_print("SETPOINT 1:"); D_print(getSetpoint1()); D_print("   POS 1:"); D_print(current_position_1); D_print("    ERR 1:"); D_print(current_error_1); 
  D_print("    IR 1:"); D_print(analogRead(Front_IR_pin)); D_println("");

  // Needed for some reason (DONT COMMENT OUT!)
  D_print("SETPOINT 2:"); D_print(getSetpoint2()); D_print("   POS 2:"); D_print(current_position_2); D_print("    ERR 2:"); D_print(current_error_2); 
  D_print("    IR 2:"); D_print(analogRead(Back_IR_pin)); D_println("");

  /////////////// Topic Execution: SetDynamixelZero ///////////////////////////// 

  while (SetDynamixelZero_flag == true){

    Serial.print("Switch pin:   "); Serial.print(digitalRead(Switch_Pin)); Serial.print("First zero?:   "); Serial.println(DynamixelFirstZero);


    if (digitalRead(Switch_Pin)==HIGH and DynamixelFirstZero == true){


        Serial.println("zero set");
        Dynamixel_Zero = 1;

        // set the zero flag to true
        zero_flag = true;
        DynamixelFirstZero = false;
      

        
      }

      // if the zero switch has been hit
      if (zero_flag == true){


        publish_Zero_Switch(String(Dynamixel_Zero));
      
        // set zero flag backto false and wait for zero to be hit again
        zero_flag = false;
        SetDynamixelZero_flag = false;

      }


  }

  /////////////// Topic Execution: ResetDynanixelZero ///////////////////////////// 

  if (ResetDynamixelZero_flag == true){

    Dynamixel_Zero = 0;
    DynamixelFirstZero = true;
    ResetDynamixelZero_flag = false;

  }



  /////////////// Topic Execution: FrontSetDCZero ///////////////////////////// 

  if (Front_Set_DC_zero_flag == true and analogRead(Front_IR_pin) < 3000){
    
    setPIDgains1(0, 0, 0);
    setPosition1_zero();
    setSetpoint1(0);
    setPIDgains1(kp_1, ki_1, kd_1);

    Front_new_setpoint_flag = false;
    Front_DC_zero_flag = true;
    Front_Set_DC_zero_flag = false;
    current_error_1 = getError1();
    current_position_1 = getPosition1();
    Front_DC_zero = 1;
  }

  if (Front_DC_zero_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Zero'd");

    current_position_1 = getPosition1();

    String Front_position = String((float)current_position_1);
    String Front_DC_zero_state = String((int)Front_DC_zero);
    
    // PUBLISH to the MQTT Broker (topic = DC_zero)
    publish_Front_DC_zero_state(Front_DC_zero_state);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = Position)
    publish_Front_DC_Position(Front_position);
    delay(1000);

    Front_DC_zero_flag = false;

    Front_DC_zero = 0;
    
  }


  /////////////// Topic Execution: BackSetDCZero ///////////////////////////// 

  if (Back_Set_DC_zero_flag == true and analogRead(Back_IR_pin) < 3500){
    
    setPIDgains2(0, 0, 0);
    setPosition2_zero();
    setSetpoint2(0);
    setPIDgains2(kp_2, ki_2, kd_2);

    Back_new_setpoint_flag = false;
    Back_DC_zero_flag = true;
    Back_Set_DC_zero_flag = false;
    current_error_2 = getError2();
    current_position_2 = getPosition2();
    Back_DC_zero = 1;
  }

  if (Back_DC_zero_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Zero'd");

    current_position_2 = getPosition2();

    String Back_position = String((float)current_position_2);
    String Back_DC_zero_state = String((int)Back_DC_zero);
    
    // PUBLISH to the MQTT Broker (topic = DC_zero)
    publish_Back_DC_zero_state(Back_DC_zero_state);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = Position)
    publish_Back_DC_Position(Back_position);
    delay(1000);

    Back_DC_zero_flag = false;
    
  }

  
  /////////////// Topic Execution: FrontPitchSetpoint ///////////////////////////// 
  
  if (abs(current_error_1) < 5.0 and Front_new_setpoint_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Reached");
    

    current_position_1 = getPosition1();

    String position_1 = String((float)current_position_1);
    D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(current_error_1); D_println("");
    
    delay(1000);

    publish_Front_DC_Position(position_1);
    Front_new_setpoint_flag = false;

  }


  /////////////// Topic Execution: BackPitchSetpoint ///////////////////////////// 
  
  if (abs(current_error_2) < 5.0 and Back_new_setpoint_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Reached");
    

    current_position_2 = getPosition2();

    String position_2 = String((float)current_position_2);
    D_print("SETPOINT:"); D_print(getSetpoint2()); D_print("   POS:"); D_print(getPosition2()); D_print("    ERR:"); D_print(current_error_2); D_println("");
    
    delay(1000);

    publish_Back_DC_Position(position_2);
    Back_new_setpoint_flag = false;

  }

  if (Get_Data == true){


    current_position_1 = getPosition1();
    current_position_2 = getPosition2();

    String position_1 = String((float)current_position_1);
    String position_2 = String((float)current_position_2);

    delay(1000);

    publish_Front_DC_Position(position_1);
    publish_Back_DC_Position(position_2);


    Get_Data = false;
  }



  // End of loop and update MQTT client
  client.loop();
  
}