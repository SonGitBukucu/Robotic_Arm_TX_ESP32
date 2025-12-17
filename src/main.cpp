#include <Arduino.h>
#include <ESP32Servo.h>
// put function declarations here:
int servoHesap(int, int, int);
Servo parmak;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  parmak.attach(26);
}

void loop() {
  // put your main code here, to run repeatedly:
  int rawdeger = (analogRead(34));
  int deger = constrain(map(rawdeger, 2720, 4095, 1000, 2000), 1000, 2000);
  //Serial.println(rawdeger);
  Serial.println(deger);
  //parmak.writeMicroseconds(deger);

}

// put function definitions here:
int servoHesap(int pin, int altDeger, int ustDeger) {
  int rawDeger = analogRead(pin);
  int deger = map(rawDeger, altDeger, ustDeger, 1000, 2000);
  return constrain(deger, 1000, 2000);
}