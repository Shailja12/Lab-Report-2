int S1 = 12;
int S2 = 13;
int S3 = 14 ;
int S4 = 25;
int S5 = 26;
int S6 = 27;
 
int value1 = 0;
int value2 = 0;
int value3 = 0;
int value4 = 0;
int value5 = 0;
int value6 = 0;

int leftMotor_speed = 0;
int rightMotor_speed = 0;
int servoAngle = 90;
 
#define I2C_SLAVE_ADDR 0x04  
#include <Wire.h>
 
void setup() {
    
  Serial.begin(9600);
  Wire.begin();  // Initialize the I2C bus
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  // Center servo
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 88;
 
}
 
void loop() {
  value1 = analogRead(S1);
  value2 = analogRead(S2);
  value3 = analogRead(S3);
  value4 = analogRead(S4);
  value5 = analogRead(S5);
  value6 = analogRead(S6);
 
  if (value3 < 4095 && value4 < 4095) {
    //Robot is on the line, move forward
   leftMotor_speed = 150;
   rightMotor_speed = 150;
   servoAngle = 88;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); 
  }
  else if (value3 == 4095 && value4 == 4095) {
    // Turn left
    leftMotor_speed = 0;
    rightMotor_speed = 0;
    servoAngle = 88;
    Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); 
  }
 
}
void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle){
  Wire.beginTransmission(I2C_SLAVE_ADDR); 
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8)); 
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));         
  Wire.write((byte)(servoAngle & 0x000000FF)); 
  Wire.endTransmission();   
}
