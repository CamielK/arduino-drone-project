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

//timing variables
unsigned long lastRunTime = micros();
int logicUpdatesPerSecond = 60;
int updateInterval = 1000000 / logicUpdatesPerSecond;

//sensor variables
float heading; //heading in degrees from magnetic north
float elevation = 0; //height


void setup() {
  Serial.begin(9600); Serial.println("(M:S:ms)  -   Log Message:");
  Serial.print(getCurrentTime()); Serial.println("Flight Controller entered setup.");

  //initialise compass sensor
  if(!compass.begin()) {
    Serial.print(getCurrentTime()); Serial.println(">>> CRITICAL ERROR <<< unable to connect with HMC5883");
    while(1);
  }
  sensor_t sensor;
  compass.getSensor(&sensor);
}



void loop() {
  Serial.print(getCurrentTime()); Serial.println("Flight Controller is executing.");
  int ticks = 0;
  long lastTimer = millis();
  
  while (true) {
    if ((micros() - lastRunTime) >= updateInterval) {
      ticks++; lastRunTime = micros();

      tick();
    }
    if (millis() - lastTimer >= 1000) { //update counters and log information when second passes
      lastTimer += 1000;
      Serial.print(getCurrentTime()); Serial.print(String("(t:" + String(ticks) + ")")); Serial.println(getLogInfo());
      ticks = 0;
    }
  }
}




void tick() { //main function to update sensors and adjust flightplan
  updateSensorData();
  updateFlightPlan();
}



void updateSensorData() { //update sensor information
}


void updateFlightPlan() { //calculate adjustments according to sensor data
}


String getCurrentTime() {
  float millisSinceStartup = millis();
  float secondsSinceStartup = int(millisSinceStartup / 1000);
  millisSinceStartup -= (secondsSinceStartup*1000);
  float minutesSinceStartup = int(secondsSinceStartup / 60);
  secondsSinceStartup -= (minutesSinceStartup*60);
  String mill = String(int(millisSinceStartup));
  String secs = String(int(secondsSinceStartup));
  String mins = String(int(minutesSinceStartup));
  if (minutesSinceStartup < 10) {mins = String("0" + mins);}
  if (secondsSinceStartup < 10) {secs = String("0" + secs);}
  if (millisSinceStartup < 10) {mill = String("00" + mill);}
  else if (millisSinceStartup < 100) {mill = String("0" + mill);}
  return String(mins + ":" + secs + ":" + mill + " -   ");
}

String getLogInfo() {
  return String("(h:" + String(heading) + ")(e:" + String(elevation) + ")");
}







