#pragma once
#include <Arduino.h>

class MotorController {
private:
  int m1_pin_fwd, m1_pin_rev;
  int m2_pin_fwd, m2_pin_rev;
  int current_left_speed;
  int current_right_speed;
  bool initialized;
  
  void driveMotor(int pin_fwd, int pin_rev, int speed, bool forward);
  void stopMotor(int pin_fwd, int pin_rev);

public:
  MotorController();
  void init();
  
  // main control methods
  void setMotors(int left_speed, int right_speed);
  void stop();
  
  // status queries
  bool isMoving() const;
  int getLeftSpeed() const { return current_left_speed; }
  int getRightSpeed() const { return current_right_speed; }
  
  // debug
  void printStatus();
};