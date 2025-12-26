#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "FastIMU.h"

#define ADDR_FOREARM 0x68
#define ADDR_HAND    0x69

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
void AngleCalc(MPU6500 &imu, AccelData &acc, GyroData &gyro, AngleData &out, float);
Servo parmak;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Init Sensors
  IMU_F.init(calF, ADDR_FOREARM);
  IMU_H.init(calH, ADDR_HAND);

  // Calibration (Keep sensors still!)
  Serial.println("Calibrating... DO NOT MOVE.");
  delay(2000);
  IMU_F.calibrateAccelGyro(&calF);
  IMU_H.calibrateAccelGyro(&calH);

  lastMicros = micros();
  Serial.println("System Ready!");
  parmak.attach(26);
}

void loop() {
  // 1. Get Time Delta
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;

  AngleCalc(IMU_F, accF, gyroF, anglesF, dt);
  AngleCalc(IMU_H, accH, gyroH, anglesH, dt);

  /* 5. Output for the Plotter
  Serial.print("Pitch:"); Serial.print(pitchF);
  Serial.print(",");
  Serial.print("Roll:");  Serial.print(rollF);
  Serial.print(",");
  Serial.print("Yaw:");   Serial.println(yawF);
  */

  //delay(10);

  int rawdeger = (analogRead(34));
  int deger = constrain(map(rawdeger, 2720, 4095, 1000, 2000), 1000, 2000);
    Serial.println(rawdeger);
  //Serial.println(deger);
  //parmak.writeMicroseconds(deger);

}

int servoHesap(int pin, int altDeger, int ustDeger) {
  int rawDeger = analogRead(pin);
  int deger = map(rawDeger, altDeger, ustDeger, 1000, 2000);
  return constrain(deger, 1000, 2000);
}

void AngleCalc(MPU6500 &imu, AccelData &acc, GyroData &gyro, AngleData &out, float dt) {
  imu.update();
  imu.getAccel(&acc);
  imu.getGyro(&gyro);

 float pitchRad = atan2(acc.accelX, sqrt(acc.accelY * acc.accelY + acc.accelZ * acc.accelZ)) * 180.0 / PI;
 float rollRad = atan2(acc.accelY, acc.accelZ) * 180.0 / PI;
  
  // Math: 1500 (center) + (radians * (500 / (PI/2)))
  // 500 / 1.5707 = 318.3
  out.pitch = 1500 + (pitchRad * 318.3);
  out.roll = 1500 + (rollRad * 318.3);

  if (abs(gyro.gyroZ) > 0.8) {
    out.yaw += (gyro.gyroZ * 5.55) * dt;
  }
}
