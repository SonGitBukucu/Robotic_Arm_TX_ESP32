#include <Arduino.h>
#include <MadgwickAHRS.h>
#include "FastIMU.h"

Madgwick filter;
unsigned long lastMicros;

void setup() {
  Serial.begin(115200);
  // ... your IMU init code here ...
  
  filter.begin(100); // Start with 100Hz (100 times per second)
  lastMicros = micros();
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  // Update the filter with current data
  // FastIMU gives us Accel in Gs and Gyro in deg/sec
  filter.updateIMU(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, 
                   accelData.accelX, accelData.accelY, accelData.accelZ);

  // Print results for the Serial Plotter
  Serial.print("Roll:");
  Serial.print(filter.getRoll());
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.print(filter.getPitch());
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.println(filter.getYaw());

  // Wait to maintain 100Hz frequency
  while (micros() - lastMicros < 10000); 
  lastMicros = micros();
}