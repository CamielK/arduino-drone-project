/****************************************************************************************
 *  
 *  Version: 0.1
 *  Date: 18-2-2016
 *  Author: Camiel Kerkhofs
 *  
 *  Description: This arduino script serves as flight controller for a custom made quadcopter drone
 *  Parts:  - Arduino Uno, main controller board
 *          - Magnetic compass (HMC5883L), for calcualting heading
 *          - Gyroscope+accelerometer (GY-521 MPU-6050), for maintaining orientation while having six full degrees of freedom
 *          - Ultrasone sensor (HC-SR04), for measuring distance to ground
 *          - wireless module (Nrf2401+), for communication with base station arduino
 *          - 4 x 800Kv motors
 *          - 4 x 30A ECS
 *          
 ****************************************************************************************/

//includes
#include <Wire.h>
#include <Adafruit_Sensor.h> //used for compass module
#include <Adafruit_HMC5883_U.h> //used for compass module

//assign unique ID to compass sensor
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

//sensor variables
float heading; //heading in degrees from magnetic north
float elevation = 0; //height


void setup() {
  Serial.begin(9600);
}



void loop() {
  Serial.println("test");
  delay(500);
}







