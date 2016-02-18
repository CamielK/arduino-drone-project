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

void setup() {
  Serial.begin(9600);
}



void loop() {
  Serial.println("test");
  delay(500);
}







