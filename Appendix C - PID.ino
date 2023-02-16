#include <Wire.h>

// I2C address of the Arduino Nano
#define NANO_ADDRESS 0x04

// ADC Pin assignments for sensors
#define SENSOR1 26 //Left sensor
#define SENSOR2 25 //Left sensor
#define SENSOR3 27 //Middle left
#define SENSOR4 14 //Middle right
#define SENSOR5 13 //Right
#define SENSOR6 12 //Right

int baseSpeed = 150; //EEEBot motor stable speed

int leftSpeed = 0;
int rightSpeed = 0;
int servoPos = 89.92;


int midServo = 89.92; //EEEBot center steering

int setpoint = 0; //Setpoint located on the center of the car

float K = 0.171;  //Motor speed scaling factor

// Define the weights for each sensor in mm

float weight1 = -38;
float weight2 = -22;
float weight3 = -8;
float weight4 = 8;
float weight5 = 22;
float weight6 = 38;

// Calibration variables
int min_1, min_2, min_3, min_4, min_5, min_6;
int max_1, max_2, max_3, max_4, max_5, max_6;

// PID variables
double Setpoint;//, Input, Output;
double Kp= 152.5, Ki=0, Kd=0;


//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Start the I2C communication
  Wire.begin();
  Serial.begin(9600);

  // Set the pin modes for the sensor pairs
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);
  pinMode(SENSOR6, INPUT);

  //Calibrated max and min values
  min_1 = 1367;
  min_2 = 923;
  min_3 = 1263;
  min_4 = 1127;
  min_5 = 983;
  min_6 = 1038;

  max_1 = 4095;
  max_2 = 4095;
  max_3 = 4095;
  max_4 = 4095;
  max_5 = 4095;
  max_6 = 4095;

  // Initialize PID
  //myPID.SetMode(AUTOMATIC);
  //myPID.SetSampleTime(1000);
  Setpoint = 0;
}

void loop() {


  // Read the values from all 6 sensors
  int sensor1Value = analogRead(SENSOR1);
  int sensor2Value = analogRead(SENSOR2);
  int sensor3Value = analogRead(SENSOR3);
  int sensor4Value = analogRead(SENSOR4);
  int sensor5Value = analogRead(SENSOR5);
  int sensor6Value = analogRead(SENSOR6);

    //Constrains the values to a specified range, to disallow extremes
  sensor1Value = constrain(sensor1Value, min_1, max_1);
  sensor2Value = constrain(sensor2Value, min_2, max_2);
  sensor3Value = constrain(sensor3Value, min_3, max_3);
  sensor4Value = constrain(sensor4Value, min_4, max_4);
  sensor5Value = constrain(sensor5Value, min_5, max_5);
  sensor6Value = constrain(sensor6Value, min_6, max_6);

  //Maps the values into the 0 to 255 range
  sensor1Value = map(sensor1Value, min_1, max_1, 0, 255);
  sensor2Value = map(sensor2Value, min_2, max_2, 0, 255);
  sensor3Value = map(sensor3Value, min_3, max_3, 0, 255);
  sensor4Value = map(sensor4Value, min_4, max_4, 0, 255);
  sensor5Value = map(sensor5Value, min_5, max_5, 0, 255);
  sensor6Value = map(sensor6Value, min_6, max_6, 0, 255);

  // Calculate the weighted average of the sensor values
  float num = (sensor1Value * weight1 + sensor2Value * weight2 + sensor3Value * weight3 + sensor4Value * weight4 + sensor5Value * weight5 + sensor6Value * weight6);
  float denum = (sensor1Value + sensor2Value + sensor3Value + sensor4Value + sensor5Value + sensor6Value);

  float weightedAvg = num/denum;

  float error = Setpoint - weightedAvg;
  float sum = 0;
  sum = sum + error;
  float PrevError = 0;
  float dErr = error - PrevError
  float output = Kp*error + Ki*sum + Kd*dErr;
  PrevError = error;
  
  leftSpeed = baseSpeed + (K * Output);
  rightSpeed = baseSpeed - (K * Output);
  servoPos = midServo + Output;

  // Constrain the values to the appropriate range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  servoPos = constrain(servoPos, 25, 85);

  transmitArduino(leftSpeed, rightSpeed, servoPos);
}

void transmitArduino(int leftSpeed, int rightSpeed, int servoPos)

{

  Wire.beginTransmission(NANO_ADDRESS);
  Wire.write((byte)((leftSpeed & 0x0000FF00) >> 8)); 
  Wire.write((byte)(leftSpeed & 0x000000FF)); 
  Wire.write((byte)((rightSpeed & 0x0000FF00) >> 8));
  Wire.write((byte)(rightSpeed & 0x000000FF));
  Wire.write((byte)((servoPos & 0x0000FF00) >> 8));
  Wire.write((byte)(servoPos & 0x000000FF));  
  Wire.endTransmission();
}
