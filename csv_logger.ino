/* Smart Laptop Bag Code */
//External Library Selection
#include "dht.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <MPU6050.h>
//////////////////////////////////////////////////////////////////////////////
//Connection Section
const int  MQ2 = A0; //Connection for the MQ2 Sensor
const int  MQ135 = A1; //Connection for MQ135 Sensor
int knockSensor = A7;  //Connection for Piezo  1
int knockSensor2 = A6;  //Connection for Piezo  2
byte val = 0;
#define dht_apin A2  //Connection for DHT11
dht DHT;
MPU6050 mpu;
unsigned long startTime = 0;
float timeStep = 0.01;
float timeThresh = 0.5;
float pitch = 0;
float roll = 0;
float yaw = 0;
int led = 39;
bool ledstatus;
//////////////////////////////////////////////////////////////////////////////

//Declarations for the Ublox GPS Sensor
/*********************
  10 to GPS Module TX
  09 to GPS Module RX
 *********************/


SoftwareSerial mySerial(4, 3);
TinyGPSPlus gps;


//Setup Begins Here
/////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial.flush();
  mySerial.begin(115200);
  Serial.println("Initializing......");
  pinMode(knockSensor, INPUT);
  pinMode(knockSensor2, INPUT);
  pinMode(led, OUTPUT);
  pinMode(A3, INPUT);
  pinMode(MQ2, INPUT);
  pinMode(MQ135, INPUT);
  ledstatus = 0;
  //Setup for the MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Idhar Hai");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  Serial.println("Initialized!!");
}
/////////////////////////////////////////////////////////////////
void loop() {
  // put your main code here, to run repeatedly
  /*Serial.println("Piezo Value1:");
    Serial.println(piezo());
    Serial.println("Piezo Value2:");
    Serial.println(piezo2());
    Serial.println("MQ2:");
    Serial.println(MQ2_GAS());
    Serial.println("MQ135:");
    Serial.println(MQ135_GAS());
    Serial.println("MPU6050:");
    Serial.println(mpu_vals());
    Serial.println("Water Level");
    Serial.println(water());
    Serial.println("DHT:");
    DHT11_TEMP();*/
  led_Toggle();
  led_con();
  GPS();
  String s = String()+piezo() + "," + piezo2() + "," + MQ2_GAS() + "," + MQ135_GAS() + "," + mpu_vals() + "," + water() + "," + DHT11_TEMP();
  Serial.println(s);
  delay(1000);
}
///////////////////////////////////////////////////////////////////
int piezo() {
  val = analogRead(knockSensor);
  return val;
}
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
void led_con() {
  if (ledstatus) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}
///////////////////////////////////////////////////////////////////
int water() {
  int val = analogRead(A3);
  return val;
}
///////////////////////////////////////////////////////////////////
int piezo2() {
  val = analogRead(knockSensor2);
  return val;
}
///////////////////////////////////////////////////////////////////
int MQ2_GAS() {
  val = analogRead(MQ2);
  return val;
}
////////////////////////////////////////////////////////////////////
int MQ135_GAS() {
  val = analogRead(MQ2);
  return val;
}
////////////////////////////////////////////////////////////////////
String DHT11_TEMP() {
  DHT.read11(dht_apin);
  /*Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature);
    Serial.println("C  ");*/
  String s = String(DHT.humidity) + "," + DHT.temperature;
  return s;
}
/////////////////////////////////////////////////////////////////////
String mpu_vals() {
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
  String s = String(normAccel.XAxis) + "," + normAccel.YAxis + "," + normAccel.ZAxis + "," + pitch + "," + roll + "," + yaw;
  
  return (s);
}
////////////////////////////////////////////////////////////////////

//Code for the GPS and the Functions Related to GPS Are all here
void GPS() {
  while (mySerial.available() > 0) {
    gps.encode(mySerial.read());
    if (gps.location.isUpdated()) {
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
    }
  }
}


void led_Toggle() {
  char input = 0;
  if (Serial.available())
  {
    input = Serial.read();
  }
  switch (input)
  {
    case 'L'://On Sending L via the Serial Monitor the LED Will Toggle Its State
      ledstatus = !ledstatus;
  }
}
