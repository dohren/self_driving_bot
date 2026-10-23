#include "MotorControl.h"
#include <Arduino.h>

// Motor-Pins
static const int r1 = 14;
static const int r2 = 12;
static const int l1 = 26;
static const int l2 = 27;
static const int speedr = 13;
static const int speedl = 25;


MotorControl::MotorControl() {
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(speedr, OUTPUT);
  pinMode(speedl, OUTPUT);
}

void MotorControl::setSpeed(float left, float right) {
  Serial.printf("[MotorControl] setSpeed L: %.1f, R: %.1f\n", left, right);

  applyPWM(l1, l2, speedl, left);
  applyPWM(r1, r2, speedr, right);
}

void MotorControl::applyPWM(int pin1, int pin2, int pwmPin, float speed) {
  bool forward = speed >= 0;
  int pwm = abs((int)speed);
  pwm = constrain(pwm, 0, 255);

  digitalWrite(pin1, forward ? HIGH : LOW);
  digitalWrite(pin2, forward ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void MotorControl::stop() {
  analogWrite(speedl, 0);
  analogWrite(speedr, 0);
  digitalWrite(l1, LOW);
  digitalWrite(l2, LOW);
  digitalWrite(r1, LOW);
  digitalWrite(r2, LOW);
  Serial.println("[MotorControl] Motors stopped");
}