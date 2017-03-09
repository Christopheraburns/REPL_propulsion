/*
 * REPL Arduino Propulsion sketch - 3/7/2017
 * Chris Burn @Forecast_Cloudy
 * 
 */

#include <SPI.h>
#include <BridgeClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Network Values
byte mac[]    = { 0x90, 0xA2, 0xDA, 0xF7, 0x0A, 0xEC };
IPAddress ip(192, 168, 1, 123);
IPAddress server(107, 22, 10, 157);
BridgeClient ethClient;
PubSubClient client(ethClient);
//Motor Values
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 



//Adafruit_DCMotor *leftWheel = AFMS.getMotor(1); //1 = M1
//Adafruit_DCMotor *rightWheel = AFMS.getMotor(2); //2 = M2
//Adafruit_DCMotor *leftTread = AFMS.getMotor(3); //3 = M3
//Adafruit_DCMotor *rightTread = AFMS.getMotor(4); //4 = M4

void callback(char* topic, byte* payload, unsigned int length) {
  // Memory pool for JSON object tree.
  String jsonStr;
  StaticJsonBuffer<200> jsonBuffer;
  Serial.print("REPL instruction set arrived [");
  Serial.print(topic);
  Serial.println("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    jsonStr += (char)payload[i];
  }
  Serial.println();
  
  char json[length + 1];
  jsonStr.toCharArray(json,sizeof(json));
  JsonObject& root = jsonBuffer.parseObject(json);
  if (!root.success()) {
    Serial.println("parseObject() failed");
  }
  //const char* type = root["type"];
  String type = root["type"];
  //const char* side = root["side"];
  String side = root["side"];
  //const char* dir = root["direction"];
  String dir = root["direction"];
  double speed = root["speed"];
  double duration = root["duration"];

  propel(type,side,dir,speed,duration);
}

void propel(String type, String side, String dir, double speed, double duration) {

  type.trim();
  side.trim();
  dir.trim();
  
  uint8_t i; //0-255 for speed.
  boolean tankMode = false;
  int motorNum;
  Adafruit_DCMotor *driveMotor;
  Adafruit_DCMotor *tankMotor;
  
  //Which drive?
  if(type == "tread")
  {
    Serial.println("Tread(s) selected");
    if(side == "right")
    {
      driveMotor = AFMS.getMotor(4);
      motorNum = 4;
    }
    else if(side == "left")
    {
      driveMotor = AFMS.getMotor(3); 
      motorNum = 3;
    }
    else //tankMode
    {
      driveMotor = AFMS.getMotor(4);
      tankMotor = AFMS.getMotor(3); 
      motorNum = 43;
      tankMode = true;
    }
  }
  else if(type == "wheel")
  {
    Serial.println("Wheel(s) selected");
    if(side == "right")
    {
      driveMotor = AFMS.getMotor(2);
      motorNum = 2;
    }
    else if(side == "left")
    {
      driveMotor = AFMS.getMotor(1);
      motorNum = 1;
    }
    else //tankMode
    {
      driveMotor = AFMS.getMotor(2);
      tankMotor = AFMS.getMotor(1);
      motorNum = 21;
      tankMode = true;
    }
  }

  Serial.println(motorNum);
  int motorSpeed = 0;
  
  AFMS.begin(); 
  //How Fast (.25, .5, 1.0) (quarter, half, full)
  //Set the speed to start, from 0 (off) to 255 (max speed)
  if(speed == 0.25)
  {
    motorSpeed = 75;
  }
  else if(speed == 0.50)
  {
    motorSpeed = 150;
  }
  else
  {
    motorSpeed = 255;
  }

  //intialize the motor
  driveMotor->setSpeed(motorSpeed);
  driveMotor->run(FORWARD);
  driveMotor->run(RELEASE);
  if(tankMode = true)
  {
    tankMotor->setSpeed(motorSpeed);
    tankMotor->run(FORWARD);
    tankMotor->run(RELEASE);
  }


  if(tankMode = true)
  {
    driveMotor->run(FORWARD);
    tankMotor->run(FORWARD);
    for (i=0; i <= duration; i++) { 
       driveMotor->setSpeed(motorSpeed);  
       tankMotor->setSpeed(motorSpeed);
        delay(duration * 1000/duration);
    }
      //Reduce Speed
     for (i=255; i!=0; i--) {
       driveMotor->setSpeed(i);  
       tankMotor->setSpeed(i);
       delay(10);
    }
  }
  else
  {
    driveMotor->run(FORWARD);
    for (i=0; i <= duration; i++) { 
       driveMotor->setSpeed(motorSpeed);  
        delay(duration * 1000/duration);
    }
      //Reduce Speed
     for (i=255; i!=0; i--) {
       driveMotor->setSpeed(i);  
       delay(10);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic","hello world");
      // ... and resubscribe
      Serial.println("Subscribing to REPL topic");
      client.subscribe("REPL");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  //Start the Serial monitor
  Serial.begin(57600);
  Bridge.begin();
  
 
  //Start Mosquitto
  client.setServer(server, 1883);
  client.setCallback(callback);
  
  // Allow the hardware to sort itself out
  delay(1500);
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
