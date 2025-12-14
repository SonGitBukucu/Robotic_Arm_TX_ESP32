#include <Arduino.h>
#include <ESP32Servo.h>
// put function declarations here:
int servoHesap(int, int, int);
Servo parmak;
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  parmak.attach(26);
}

void loop() {
  // put your main code here, to run repeatedly:
  int rawdeger = (constrain(analogRead(34), 0, 4096));
  int deger = constrain(map(rawdeger, 0, 3300, 1000, 2000), 1000, 2000);
  Serial.println(deger);
  parmak.writeMicroseconds(deger);

}

// put function definitions here:
int servoHesap(int pin, int altDeger, int ustDeger) {
  int rawDeger = constrain(analogRead(pin), altDeger, ustDeger);
  int deger = map(rawDeger, 0, 4096, 1000, 2000);
  return constrain(deger, 1000, 2000);
}