//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

#include <WiFi.h>
#include <PubSubClient.h>
#include <MPU6050_tockn.h>

//HCSR04
#define echoPin 5//
#define trigPin 18///
long duration;
int distance;

//MPU6050
MPU6050 mpu6050(Wire);
int mpux = 0;
int mpuy = 0;
int mpuz = 0;

//Infrared Sensor
int Sensor1 = 34;
int val1 = 0;

// Replace the next variables with your SSID/Password combination
const char* ssid = "c15Ison";
const char* password = "c15Isonn";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.137.242";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
int value = 0;
char msg[50];

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
 
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
}
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void setup_wifi(){
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");

      // Add your subscribe topics here
      // --
      client.subscribe("esp32/output");
      client.subscribe("esp32/distance");
      client.subscribe("esp32/X", mpu_xString);
      client.subscribe("esp32/Y", mpu_yString);
      client.subscribe("esp32/Z", mpu_zString);
      client.subscribe("esp32/S1", S1_String);
      // --

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void loop() {
    if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;


    //HC-SR04
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);

    distance = duration * 0.034 / 2;
    Serial.println(distance);
    char cm_String[8];
    dtostrf(distance, 1, 2, cm_String);
    client.publish("esp32/distance", cm_String);

    //MPU6050
    mpu6050.update();

    mpu_x = mpu6050.getAngleX();
    Serial.println(mpu_x);
    char mpu_xString[8];
    dtostrf(mpu_x, 1, 2, mpu_xString);
    client.publish("esp32/X", mpu_xString);

    mpu_y = mpu6050.getAngleY();
    Serial.println(mpu_y);
    char mpu_yString[8];
    dtostrf(mpu_y, 1, 2, mpu_yString);
    client.publish("esp32/Y", mpu_yString);

     mpu_z = mpu6050.getAngleZ();
    Serial.println(mpu_z);
    char mpu_zString[8];
    dtostrf(mpu_z, 1, 2, mpu_zString);
    client.publish("esp32/Z", mpu_zString);
 
    //IR S1
    value1 = analogRead(1);
    Serial.println(value1);
    char S1_String[8];
    dtostrf(value1, 1, 2, S1String);
    client.publish("esp32/S1", S1_String);
   
  }
}