#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

class MotorControl {
public:
  MotorControl();
  void setSpeed(float left, float right);  // Hauptmethode für ROS
  void stop(); // optional für Notstopp

private:
  void applyPWM(int pin1, int pin2, int pwmPin, float speed);
};

#endif

