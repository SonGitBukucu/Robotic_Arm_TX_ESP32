// MUSTAFA ALPER KAYA
// NRF24L01+ modüllerini kullanan robot kol projesinin "eldiven" (verici) kodu.

/*#####################################################         KANAL SIRALAMASI         #####################################################
  Keyfinize göre açın, kapatın, değiştirin.
  1 = Pan
  2 = Tilt
  3 = Bilek
  4 = Baş Parmak
  5 = İşaret Parmak
  6 = Orta Parmak
  7 = Yüzük Parmak
  8 = Serçe Parmak
######################         DOĞRU ÇALIŞMASI İÇİN LÜTFEN HEM VERİCİDE HEM DE ALICIDA AYNI SIRALAMAYI YAPIN         #######################*/

#include <Arduino.h>
#include <Wire.h>
#include "FastIMU.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>

//######################################                PARMAKLAR                ######################################
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
//######################################                PARMAKLAR                ######################################

//######################################                NRF24                ######################################
#define NRF24_CE          4
#define NRF24_CSN         5
#define DEBUG_LED         16

unsigned long sonVeri = 0;
unsigned int failsafeAralik = 700; // fail-safe devreye girmesi için gereken süre.
bool iletisimVar = false;

short kanal[8];

const byte nrf24kod[5] = {'r','o','b','o','t'}; 
RF24 radio(NRF24_CE, NRF24_CSN);
bool ilkVeriGitti = false;
//######################################                NRF24                ######################################

//######################################                GYRO & ACCEL                ######################################
#define ADDR_FOREARM  0x68
#define ADDR_HAND     0x69

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

AngleData anglesF = {};
AngleData anglesH = {};

// Yaw için ekstra değişkenler
float yawH = 0;
float yawF = 0; 
unsigned long lastMicros;
//######################################                GYRO & ACCEL                ######################################

//######################################                FONKSİYON BİLDİRİMLERİ                ######################################
int parmakHesap(int, int, int); //Flex sensöründen gelen veriden servo için hareket değerine dönüştüren fonksiyon.
void angleCalc(MPU6500 &imu, AccelData &acc, GyroData &gyro, AngleData &out, float);  //Bir gyro-accel modülünden gelen veriler doğrultusunda 3 eksende açı hesaplaması yapan fonksiyon.
//######################################                FONKSİYON BİLDİRİMLERİ                ######################################

//######################################                FONKSİYON TANIMLARI               ######################################

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

  kanal[0] = anglesF.pitchPWM;
  kanal[1] = anglesF.yawPWM;
  kanal[2] = anglesH.pitchPWM;
  kanal[3] = parmakHesap(basParmak, basParmakAlt, basParmakUst); 
  kanal[4] = parmakHesap(isaretParmak, isaretParmakAlt, isaretParmakUst);
  kanal[5] = parmakHesap(ortaParmak, ortaParmakAlt, ortaParmakUst);
  kanal[6] = parmakHesap(yuzukParmak, yuzukParmakAlt, yuzukParmakUst);
  kanal[7] = parmakHesap(serceParmak, serceParmakAlt, serceParmakUst);

  bool iletisim = radio.write(&kanal,sizeof(kanal));

  if (iletisim) {
    sonVeri = millis();
    if (!iletisimVar) {
      iletisimVar = true;
      ilkVeriGitti = true;
      digitalWrite(DEBUG_LED, HIGH);
    }
    
  }
  if (ilkVeriGitti && !iletisim && millis() - sonVeri >= failsafeAralik && iletisimVar) {
    iletisimVar = false;
    digitalWrite(DEBUG_LED, LOW);
    }

  //Serial.println(anglesF.pitch);
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

  // 1. Calculate Angles from Accelerometer (Gravity)
  float pitchRad = atan2(acc.accelX, sqrt(acc.accelY * acc.accelY + acc.accelZ * acc.accelZ));
  float rollRad = atan2(acc.accelY, acc.accelZ);
  
  // 2. Map to 544-2400 range
  // Center is 1472. The max swing from center is 928.
  // 928 / (PI/2) = 590.78
  out.pitch = 1472 + (pitchRad * (928.0 / (PI / 2.0)));
  out.roll = 1472 + (rollRad * (928.0 / (PI / 2.0)));

  // 3. Constrain to your new physical limits
  out.pitch = constrain(out.pitch, 544, 2400);
  out.roll = constrain(out.roll, 544, 2400);

  // 4. Calculate Yaw (Integrated from Gyro)
  if (abs(gyro.gyroZ) > 0.8) {
    out.yaw += (gyro.gyroZ) * dt;
  }

  // 5. Convert to PWM Output
  out.pitchPWM = (short)out.pitch;
  out.rollPWM = (short)out.roll;

  // For Yaw: Assuming out.yaw is degrees. 
  // To map -90 to +90 degrees into 544 to 2400:
  // (yaw + 90) * (TotalSpan / TotalDegrees) + Offset
  out.yawPWM = constrain((out.yaw + 90.0) * (1856.0 / 180.0) + 544, 544, 2400);
}

//######################################                FONKSİYON TANIMLARI               ######################################