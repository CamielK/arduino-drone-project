# arduino-drone-project
Arduino flight controller for a quadcopter drone

project consists of 2 arduino uno's. The first serves as flight controller on the drone and the second arduino serves as base station to cummunicate between pc and flight controller.
flight controller parts:  
  - Arduino Uno
  - 1 x Magnetic compass (HMC5883L), for calcualting heading
  - 1 x Gyroscope+accelerometer (GY-521 MPU-6050), for maintaining orientation
  - 1 x Ultrasone sensor (HC-SR04), for measuring distance to ground
  - 1 x wireless module (Nrf2401+), for communication with base station arduino
  - 4 x 800Kv motors
  - 4 x 30A ECS
base station parts:
  - Arduino Uno
  - 1 x wireless module (Nrf2401+), for communication with flight controller arduino
