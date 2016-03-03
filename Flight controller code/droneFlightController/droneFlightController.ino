/****************************************************************************************
 *  
 *  Version: 0.2
 *  Date: 03-3-2016
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
#include "I2Cdev.h" //used for communication with MPU6050
#include "MPU6050_6Axis_MotionApps20.h" //used for calculating MPU6050 values

//defines
#define trigPin 13
#define echoPin 12
bool blinkState = false;

//assign unique ID to compass sensor
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

//initiate mpu at default adress 0x68 (use 'MPU6050 mpu(0x69);' for alternative adress)
MPU6050 mpu;

//timing variables
unsigned long lastRunTime = micros();
int logicUpdatesPerSecond = 60;
int updateInterval = 1000000 / logicUpdatesPerSecond;
int loopCount = 0;

//sensor variables
float heading; //heading in degrees from magnetic north
float elevation = 0; //height
float yaw;
float pitch;
float roll;

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//interrupt detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



void setup() {
    // initialize serial communication
    Serial.begin(115200); Serial.println(""); Serial.println("Flight Controller entered setup.");


    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing mpu device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    //initialize compass sensor
    Serial.println(F("Initializing compass..."));
    if(!compass.begin()) {
      Serial.println(">>>ERROR<<< unable to connect with HMC5883");
      while(1);
    }
    sensor_t sensor;
    compass.getSensor(&sensor);

    //set custom mpu6050 offsets
    mpu.setXGyroOffset(-18);
    mpu.setYGyroOffset(-40);
    mpu.setZGyroOffset(16);
    mpu.setXAccelOffset(-2880);
    mpu.setYAccelOffset(-110);
    mpu.setZAccelOffset(1565);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


    //configure in and outputs
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}



void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  Serial.println("Flight Controller is executing.");

  //timing loop
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

  //loopCount++;
  //if (loopCount == 10) {
  //  loopCount = 0;
    updateSensorData();
  //}

  updateFlightPlan();
}



void updateSensorData() { //update sensor information
  heading = getCompassReading();
  elevation = getSonarReading();
  getMpuReading(); //get gyro reading
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


//get data from sonar
float getSonarReading() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2; //(duration/2) / 29.1;

  if (distance >= 200) { //return 0 when out of range
    distance = 0;
  }
  return distance;
}


void getMpuReading() {
    //get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = ypr[0] * 180/M_PI;
        pitch = ypr[1] * 180/M_PI;
        roll = ypr[2] * 180/M_PI;
    }
}



//send log message over serial
void sendLogMsg() {
  String logMsg = String("|h" + String(heading) + "|e" + String(elevation) + "|y" + String(yaw) + "|p" + String(pitch) + "|r" + String(roll) + "|");
  Serial.println(logMsg);
}








