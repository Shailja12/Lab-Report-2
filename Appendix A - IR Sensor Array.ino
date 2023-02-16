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
 
#include <Wire.h>
 
 
void setup() {
  Serial.begin(9600);  // put your setup code here, to run once:
}
 
void loop() {
  value1 = analogRead(S1);
  value2 = analogRead(S2);
  value3 = analogRead(S3);
  value4 = analogRead(S4);
  value5 = analogRead(S5);
  value6 = analogRead(S6);
 
  
  // Print sensor values
  Serial.print(value1);
  Serial.print(value2);
  Serial.print(value3);
  Serial.print(value4);
  Serial.print(value5);
  Serial.print(value6);
  
}
