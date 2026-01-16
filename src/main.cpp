#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "FastIMU.h"
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

#define ADDR_FOREARM 0x68
#define ADDR_HAND    0x69

#define NRF24CE   4
#define NRF24CSN  5 //PLACEHOLDER

#define basParmak       36
#define basParmakUst    4096
#define basParmakAlt    0

#define isaretParmak    39
#define isaretParmakUst 4096
#define isaretParmakAlt 0

#define ortaParmak      34
#define ortaParmakUst   4096 //ÜST VE ALT DEĞERLER ÖLÇÜLMEDİĞİ İÇİN ŞİMDİLİK BÖYLE
#define ortaParmakAlt   0

#define yuzukParmak     35
#define yuzukParmakUst  4096
#define yuzukParmakAlt  0

#define serceParmak     32
#define serceParmakUst  4096
#define serceParmakAlt  0

MPU6500 IMU_F;
MPU6500 IMU_H;

calData calF, calH;
AccelData accF, accH; 
GyroData gyroF, gyroH;

struct AngleData {
  float pitch;
  float roll;
  float yaw;
};
AngleData anglesF = {0,0,0};
AngleData anglesH = {0,0,0};

// Variables for Angle Tracking
float yawH = 0;
float yawF = 0; 
unsigned long lastMicros;

int servoHesap(int, int, int);
void angleCalc(MPU6500 &imu, AccelData &acc, GyroData &gyro, AngleData &out, float);
Servo parmak;

short kanal[8];

const byte nrf24kod[5] = {'r','o','b','o','t'}; 
RF24 radio(NRF24CE, NRF24CSN);

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);


  // Calibration (Keep sensors still!)
  Serial.println("Calibrating... DO NOT MOVE.");
  delay(2000);
  IMU_F.calibrateAccelGyro(&calF);
  IMU_H.calibrateAccelGyro(&calH);

  // Init Sensors
  IMU_F.init(calF, ADDR_FOREARM);
  IMU_H.init(calH, ADDR_HAND);

  lastMicros = micros();
  Serial.println("System Ready!");
  parmak.attach(26);

  radio.begin();
  radio.openWritingPipe(nrf24kod);
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX); // güç çıkışı şu an yüksek durumda, mesafeyi azaltmak için ...(RF24_PA_LOW/MIN) yapılmalı.
  radio.stopListening();
}

void loop() {
  // 1. Get Time Delta
  unsigned long currentMicros = micros();
  float deltaT = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;

  angleCalc(IMU_F, accF, gyroF, anglesF, deltaT);
  angleCalc(IMU_H, accH, gyroH, anglesH, deltaT);

  /* 5. Output for the Plotter
  Serial.print("Pitch:"); Serial.print(pitchF);
  Serial.print(",");
  Serial.print("Roll:");  Serial.print(rollF);
  Serial.print(",");
  Serial.print("Yaw:");   Serial.println(yawF);
  */



  int rawdeger = (analogRead(34));
  int deger = constrain(map(rawdeger, 2720, 4095, 1000, 2000), 1000, 2000);
    Serial.println(anglesF.roll);
  

  
  kanal[3] = servoHesap(basParmak, basParmakAlt, basParmakUst);
  kanal[4] = servoHesap(isaretParmak, isaretParmakAlt, isaretParmakUst);
  kanal[5] = servoHesap(ortaParmak, ortaParmakAlt, ortaParmakUst);
  kanal[6] = servoHesap(yuzukParmak, yuzukParmakAlt, yuzukParmakUst);
  kanal[7] = servoHesap(serceParmak, serceParmakAlt, serceParmakUst);
  
  //Serial.println(deger);
  //parmak.writeMicroseconds(deger);
  //delay(10);
}

int servoHesap(int pin, int altDeger, int ustDeger) {
  int rawDeger = analogRead(pin);
  int deger = map(rawDeger, altDeger, ustDeger, 1000, 2000);
  return constrain(deger, 1000, 2000);
}

void angleCalc(MPU6500 &imu, AccelData &acc, GyroData &gyro, AngleData &out, float dt) {
  imu.update();
  imu.getAccel(&acc);
  imu.getGyro(&gyro);

 float pitchRad = atan2(acc.accelX, sqrt(acc.accelY * acc.accelY + acc.accelZ * acc.accelZ));
 float rollRad = atan2(acc.accelY, acc.accelZ);
  
  // Math: 1500 (center) + (radians * (500 / (PI/2)))
  // 500 / 1.5707 = 318.3
  out.pitch = 1700 + (pitchRad * 636.6);
  out.roll = 1550 + (rollRad * 636.6);

  if (abs(gyro.gyroZ) > 0.8) {
    out.yaw += (gyro.gyroZ) * dt;
  }
}
