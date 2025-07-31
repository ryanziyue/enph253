#pragma once
#include <Arduino.h>

class MotorController {
private:
  bool initialized;
  int current_left_speed;
  int current_right_speed;
  
  int minSpeed;
  int maxSpeed; 

public:
  MotorController();
  void init();
  
  // basic motor control
  void stopMotor(uint8_t chanFwd, uint8_t chanRev);
  void driveMotor(uint8_t chanFwd, uint8_t chanRev, int speed);
  void setMotors(int left_speed, int right_speed);
  void stop();
  
  // speed constraint methods
  void setMinSpeed(int min);
  void setMaxSpeed(int max);
  int getMinSpeed() const { return minSpeed; }
  int getMaxSpeed() const { return maxSpeed; }
  
  // status
  bool isMoving() const;
  int getLeftSpeed() const { return current_left_speed; }
  int getRightSpeed() const { return current_right_speed; }
  
  // debug
  void printStatus();
};