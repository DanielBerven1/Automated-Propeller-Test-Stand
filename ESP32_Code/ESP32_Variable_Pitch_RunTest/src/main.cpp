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
const char* DC_MOTOR_topic = "DC_Position";
const char* DC_zero_topic = "DC_Zero";

// Replace your MQTT Broker IP address here:
const char* mqtt_server = "10.105.76.221";

WiFiClient espClient;
PubSubClient client(espClient);



long lastMsg = 0;


#define ledPin 2
#define IR_pin 36

// DC motor variables
double setpoint = 0; 
double current_error = 0;
double current_position = 0;
double c_position;
float offset = 90.0;
float DC_zero = 0;

// Topic flags
bool new_setpoint_flag = false; 
bool Get_Pitch_Data = false;
bool DC_zero_flag = false;
bool Set_DC_zero_flag = false;

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
          client.subscribe("rpi/PitchSetpoint");
          client.subscribe("rpi/GetPitchData");
          client.subscribe("rpi/SetDCZero");
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

  // Check if a message is received on the topic "rpi/Pitch_Setpoint"
  if (String(topic) == "rpi/PitchSetpoint") {

    // Convert message string into type double
    setpoint = stod(messageTemp);

    // Set DC motor setpoint
    setSetpoint1(setpoint);
    
    // Debugging print statements 
    // current_error=getError1();
    // c_position = getPosition1();
    // D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(c_position); D_print("    ERR:"); D_print(current_error); D_println("");

    // set new message flag to TRUE
    new_setpoint_flag = true;

  }

  // Check if a message is received on the topic "rpi/Get_Data"
  if (String(topic) == "rpi/GetPitchData") {

    Get_Pitch_Data = true;
    
  }

  if (String(topic) == "rpi/SetDCZero") {


    Set_DC_zero_flag = true;

  //Similarly add more if statements to check for other subscribed topics 
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



void setup() {

  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server,1883);//1883 is the default port for MQTT server
  client.setCallback(callback);

  setupDCMotors();
  
  if (!client.connected()) {
    connect_mqttServer();
  }



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

  if (Set_DC_zero_flag == true and analogRead(IR_pin) < 2457){
    
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

  if (DC_zero_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Zero'd");

    current_position = getPosition1();

    String position = String((float)current_position);
    String DC_zero_state = String((int)DC_zero);
    // D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(current_error); D_println("");
    
    // PUBLISH to the MQTT Broker (topic = DC_zero)
    publish_DC_zero_state(DC_zero_state);
    delay(1000);
    // PUBLISH to the MQTT Broker (topic = Position)
    publish_DC_Position(position);
    delay(1000);

    DC_zero_flag = false;
    
  }


  current_error = getError1();
  current_position = getPosition1();

  // Needed for some reason (DONT COMMENT OUT!)
  D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(current_position); D_print("    ERR:"); D_print(current_error); 
  D_print("    IR:"); D_print(analogRead(IR_pin)); D_println("");


  /////////////////////////////////////////////////////////////////////////////////////////////


  /////////////// Topic Execution: Get_Pitch_Data ///////////////////////////// 
  if (Get_Pitch_Data == true){

    current_position = getPosition1();

    String position = String((float)current_position);
    D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(current_error); D_println("");
    
    delay(1000);

    // PUBLISH to the MQTT Broker (topic = Position)
    publish_DC_Position(position);
    Get_Pitch_Data = false;

  }  
  
  /////////////// Topic Execution: Pitch_Setpoint ///////////////////////////// 
  
  if (abs(current_error) < 5.0 and new_setpoint_flag == true){

    // wait for steady state
    delay(1000);
    Serial.println("Reached");
    

    current_position = getPosition1();

    String position = String((float)current_position);
    D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(current_error); D_println("");
    
    delay(1000);

    publish_DC_Position(position);
    new_setpoint_flag = false;

  }

  // End of loop and update MQTT client
  client.loop();
  
}