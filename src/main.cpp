#include <Arduino.h>
#include <Wire.h>
#include "FastIMU.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
//ALT VE ÜST DEĞERLER TEKER TEKER ÖLÇÜLMELİ
#define basParmak         36
#define basParmakAlt      550
#define basParmakUst      4095

#define isaretParmak      39
#define isaretParmakAlt   0
#define isaretParmakUst   4096

#define ortaParmak        34
#define ortaParmakAlt     0
#define ortaParmakUst     4096

#define yuzukParmak       35
#define yuzukParmakAlt    0
#define yuzukParmakUst    4096

#define serceParmak       32
#define serceParmakAlt    0
#define serceParmakUst    4096

#define ADDR_FOREARM  0x68
#define ADDR_HAND     0x69

#define NRF24_CE          4
#define NRF24_CSN         5
#define DEBUG_LED         16 //PLACEHOLDER

MPU6500 IMU_F;
MPU6500 IMU_H;

calData calF, calH;
AccelData accF, accH; 
GyroData gyroF, gyroH;

struct AngleData {
  float pitch;
  float roll;
  float yaw;
  short pitchPWM;
  short rollPWM;
  short yawPWM;
};

AngleData anglesF = {0,0,0};
AngleData anglesH = {0,0,0};

// Variables for Angle Tracking
float yawH = 0;
float yawF = 0; 
unsigned long lastMicros;

unsigned long sonVeri = 0;
unsigned int failsafeAralik = 700; // fail-safe devreye girmesi için gereken süre.
bool failsafeDurum = false;

short kanal[8];

const byte nrf24kod[5] = {'r','o','b','o','t'}; 
RF24 radio(NRF24_CE, NRF24_CSN);
bool ilkVeriGitti = false;

int parmakHesap(int, int, int);
void angleCalc(MPU6500 &imu, AccelData &acc, GyroData &gyro, AngleData &out, float);

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  //Serial.begin(115200);

  pinMode(basParmak, INPUT);
  pinMode(isaretParmak, INPUT);
  pinMode(ortaParmak, INPUT);
  pinMode(yuzukParmak, INPUT);
  pinMode(serceParmak, INPUT);
  pinMode(DEBUG_LED, OUTPUT);

  radio.begin();
  radio.openWritingPipe(nrf24kod);
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX); // güç çıkışı yüksekten düşüğe: MAX | HIGH | LOW | MIN
  radio.stopListening();

  // Calibration (Keep sensors still!)
  //Serial.println("Calibrating... DO NOT MOVE.");
  delay(2000);
  
  IMU_F.init(calF, ADDR_FOREARM);
  IMU_H.init(calH, ADDR_HAND);
  
  IMU_F.calibrateAccelGyro(&calF);
  IMU_H.calibrateAccelGyro(&calH);

  // Init Sensors
  IMU_F.init(calF, ADDR_FOREARM);
  IMU_H.init(calH, ADDR_HAND);

  lastMicros = micros();
  //Serial.println("System Ready!");
}

void loop() {
  // 1. Get Time Delta
  unsigned long currentMicros = micros();
  float deltaT = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;

  angleCalc(IMU_F, accF, gyroF, anglesF, deltaT);
  angleCalc(IMU_H, accH, gyroH, anglesH, deltaT);

  kanal[0] = anglesF.pitch;
  kanal[1] = anglesF.yaw;
  kanal[2] = anglesH.pitch;
  kanal[3] = parmakHesap(basParmak, basParmakAlt, basParmakUst); 
  kanal[4] = parmakHesap(isaretParmak, isaretParmakAlt, isaretParmakUst);
  kanal[5] = parmakHesap(ortaParmak, ortaParmakAlt, ortaParmakUst);
  kanal[6] = parmakHesap(yuzukParmak, yuzukParmakAlt, yuzukParmakUst);
  kanal[7] = parmakHesap(serceParmak, serceParmakAlt, serceParmakUst);

  bool iletisim = radio.write(&kanal,sizeof(kanal));

  if (iletisim) {
    sonVeri = millis();
    if (failsafeDurum) {
      failsafeDurum = false;
      digitalWrite(DEBUG_LED, HIGH);
    }
    ilkVeriGitti = true;
  }
  if (ilkVeriGitti && !iletisim && millis() - sonVeri >= failsafeAralik && !failsafeDurum) {
    failsafeDurum = true;
    digitalWrite(DEBUG_LED, LOW);
    }

  //Serial.println(kanal[3]);
  //Serial.println(analogRead(basParmak));
 //parmak.writeMicroseconds(deger);
  //delay(10);
}

int parmakHesap(int pin, int altDeger, int ustDeger) {
  int rawDeger = analogRead(pin);
  short deger = map(rawDeger, altDeger, ustDeger, 1000, 2000);
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
  out.pitch = constrain(1500 + (pitchRad * (500 / (PI / 2))), 1000, 2000);
  out.roll = constrain(1500 + (rollRad * (500 / (PI / 2))), 1000, 2000);

  if (abs(gyro.gyroZ) > 0.8) {
   out.yaw += (gyro.gyroZ) * dt;
  }

  out.pitchPWM = short(out.pitch);
  out.rollPWM = short(out.roll);
  out.yawPWM = constrain((out.yaw + 90) * (1000 / 180) + 1050, 1000, 2000);
}