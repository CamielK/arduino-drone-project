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

int loopCount = 0;


void setup() {
  Serial.begin(115200); Serial.println(""); Serial.println("Flight Controller entered setup.");

  //initialise compass sensor
  if(!compass.begin()) {
    Serial.println(">>>ERROR<<< unable to connect with HMC5883");
    while(1);
  }
  sensor_t sensor;
  compass.getSensor(&sensor);
}



void loop() {
  Serial.println("Flight Controller is executing.");
  int ticks = 0;
  long lastTimer = millis();
  while (true) {
    
    if ((micros() - lastRunTime) >= updateInterval) {
      ticks++; lastRunTime = micros();
      tick();
    }
    if (millis() - lastTimer >= 200) { //update counters and log information when second passes
      lastTimer += 200;
      sendLogMsg();
      ticks = 0;
    }
  }
}




void tick() { //main function to update sensors and adjust flightplan

  loopCount++;
  if (loopCount == 10) {
    loopCount = 0;
    updateSensorData();
  }

  updateFlightPlan();
}



void updateSensorData() { //update sensor information
  heading = getCompassReading();
}


void updateFlightPlan() { //calculate adjustments according to sensor data
  
}

float getCompassReading() {
  //create new event
  sensors_event_t event; 
  compass.getEvent(&event);

  //calculate heading (Z should be level, TODO: take Z offset into consideration)
  float newHeading;

  //correct compass
  float xOffset = event.magnetic.x - 10; //move x axis to the left to adjust for compass centre
  float yOffset = event.magnetic.y + 16; //move y axis to to the right to adjust for compass centre

  //Serial.print("Y:"); Serial.print(yOffset); Serial.print(". X:"); Serial.print(xOffset); Serial.print(". Z:");Serial.print(event.magnetic.z); Serial.print(".");
  
  //split data into 4 squadrons
  if (yOffset >= 0 && xOffset >= 0) { //NE
	float angleRadians = atan2(yOffset, xOffset); //calculate angle
	float angleDegrees = angleRadians * (180/M_PI); // Convert radians to degrees
	newHeading = 90 - angleDegrees;
	//Serial.print("   NE: heading="); Serial.println(newHeading);
  }
  else if (yOffset >= 0 && xOffset <= 0) { //NW
	float angleRadians = atan2(yOffset, xOffset); //calculate angle
	float angleDegrees = angleRadians * (180/M_PI); // Convert radians to degrees
	newHeading = (180 - angleDegrees) + 270;
	//Serial.print("   NW: heading="); Serial.println(newHeading);
  }
  else if (yOffset <= 0 && xOffset >= 0) { //SE
	float angleRadians = atan2(yOffset, xOffset); //calculate angle
	float angleDegrees = angleRadians * (180/M_PI); // Convert radians to degrees
	newHeading = abs(angleDegrees) + 90;
	//Serial.print("   SE: heading="); Serial.println(newHeading);
  }
  else if (yOffset <= 0 && xOffset <= 0) { //SW
	float angleRadians = atan2(yOffset, xOffset); //calculate angle
	float angleDegrees = angleRadians * (180/M_PI); // Convert radians to degrees
	newHeading = abs(angleDegrees) + 90;
	//Serial.print("   SW: heading="); Serial.println(newHeading);
  }
  else {
	  newHeading = heading;
	  Serial.println(">>>uncaught heading<<<");
  }
  
  //Serial.print(" Heading: "); Serial.print(newHeading); Serial.println(".");
  
  newHeading = 360 - newHeading;//invert heading;
  return newHeading;
}

//send log message over serial
void sendLogMsg() {
  String logMsg = String("|h" + String(heading) + "|e" + String(elevation) + "|");
  Serial.println(logMsg);
}








