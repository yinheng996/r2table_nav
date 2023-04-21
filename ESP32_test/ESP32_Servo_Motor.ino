#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>

#define SERVO_PIN 4 // ESP32 pin GIOP4 connected to servo motor

Servo servoMotor;

void setup() {
  Serial.begin(9600);
  servoMotor.attach(SERVO_PIN, 771, 2740);  // attaches the servo on ESP32 pin
  servoMotor.write(0);
}

void loop() {
  // rotates from 0 degrees to 180 degrees
  Serial.println("started");
  servoMotor.write(120);
  delay(2000);
  servoMotor.write(0);
  delay(2000);
  servoMotor.write(120);
  delay(2000);
  servoMotor.write(0);
  delay(2000);
  // rotates from 180 degrees to 0 degrees
  // for (int pos = 120; pos >= 0; pos -= 1) {
  //   Serial.println("start");
  //   servoMotor.write(pos);
  //   delay(15); // waits 15ms to reach the position
  //   Serial.println("stopped");
  //   delay(1000);
  // }
}