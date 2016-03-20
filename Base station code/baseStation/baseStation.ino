/****************************************************************************************
 *  
 *  Version: 0.1
 *  Date: 18-2-2016
 *  Author: Camiel Kerkhofs
 *  
 *  Description: This arduino script serves as base station for cummincation between flight controller and pc
 *  Parts:  - Arduino Uno
 *          - wireless module (Nrf2401+), for communication with flight controller
 *          
 ****************************************************************************************/

//includes
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"

//Create an instance for the radio, specifying the CE and CS pins.
RF24 myRadio (9, 10);
byte addresses[][6] = {"1Node","2Node"};
float dataReceived;  // Data that will be received from the transmitter
int dataTransmitted;  // Data that will be Transmitted from the transmitter


//current metrics
float heading = 0; //heading in degrees from magnetic north
float elevation = 0; //height
float yaw = 0;
float pitch = 0;
float roll = 0;
float m1Throttle = 0; //front left rotor
float m2Throttle = 0; //front right rotor
float m3Throttle = 0; //back right rotor
float m4Throttle = 0; //back left rotor


//timing variables
unsigned long lastRunTime = micros();
int logicUpdatesPerSecond = 60;
int updateInterval = 1000000 / logicUpdatesPerSecond;
int loopCount = 0;


void setup() {
  Serial.begin(115200);
  Serial.println("base station entered setup");

  myRadio.begin();  // Start up the physical nRF24L01 Radio
  myRadio.setChannel(108);  // Above most Wifi Channels
  //myRadio.setPALevel(RF24_PA_MIN);
  myRadio.setPALevel(RF24_PA_MAX);

  myRadio.openWritingPipe(addresses[1]);
  myRadio.openReadingPipe(1, addresses[0]); // Use the first entry in array 'addresses' (Only 1 right now)
  myRadio.startListening();
  Serial.println("base station finished setup");
}



void loop() {

  Serial.println("Base station is executing.");

  //timing loop
  int ticks = 0;
  long lastTimer = millis();
  while (true) {
    
    ticks++; lastRunTime = micros();
    tick();
    if (millis() - lastTimer >= 300) { //update counters and log information when second passes
      lastTimer += 200;
      //Serial.println(ticks);
      sendLogMsg();
      ticks = 0;
    }
  }
}

void tick() {

  checkPcInput();

  forwardPcInput();

  checkFCInput();
  
}

void checkPcInput() {
    //check for pc serial input
    if ( Serial.available() ) {
    //    char c = toupper(Serial.read());
    //    if ( c == 'T' && role == 0 ){      
    //      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
    //      role = 1;                  // Become the primary transmitter (ping out)
    //    
    //   }else
    //    if ( c == 'R' && role == 1 ){
    //      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
    //       role = 0;                // Become the primary receiver (pong back)
    //       radio.startListening();
    //       
    //    }
    }
}

void forwardPcInput() {
    //forward pc input to flight controller
    //myRadio.stopListening();
  
    //send data to flight controller
    
    //myRadio.startListening();
}

void checkFCInput() {
    //check for flight controller log message
    if ( myRadio.available()) // Check for incoming data from transmitter
    {
      while (myRadio.available())  // While there is data ready
      {
        myRadio.read( &dataReceived, sizeof(dataReceived) ); // Get the data payload (You must have defined that already!)
      }
      // DO something with the data, like print it
      Serial.print("Data received = ");
      Serial.print(dataReceived);
      if ((dataReceived - 10000) < 500) { heading = dataReceived - 10000; } //heading
      else if ((dataReceived - 20000) < 500) { elevation = dataReceived - 20000; } //elevation
      else if ((dataReceived - 30000) < 500) { yaw = dataReceived - 30000; } //yaw
      else if ((dataReceived - 40000) < 500) { pitch = dataReceived - 40000; } //pitch
      else if ((dataReceived - 50000) < 500) { roll = dataReceived - 50000; } //roll
      else if ((dataReceived - 60000) < 500) { m1Throttle = dataReceived - 60000; } //m1t
      else if ((dataReceived - 70000) < 500) { m2Throttle = dataReceived - 70000; } //m2t
      else if ((dataReceived - 80000) < 500) { m3Throttle = dataReceived - 80000; } //m3t
      else if ((dataReceived - 90000) < 500) { m4Throttle = dataReceived - 90000; } //m4t
    } //END Radio available
}

void sendLogMsg() {
  String logMsg = String(
    "|h" + String(heading) 
  + "|e" + String(elevation) 
  + "|y" + String(yaw) 
  + "|p" + String(pitch) 
  + "|r" + String(roll) 
  + "|m1" + String(m1Throttle)
  + "|m2" + String(m2Throttle)
  + "|m3" + String(m3Throttle)
  + "|m4" + String(m4Throttle)
  + "|");
  Serial.println(logMsg);
}







