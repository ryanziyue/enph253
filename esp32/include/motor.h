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
  
  // Basic motor control
  void stopMotor(uint8_t chanFwd, uint8_t chanRev);
  void driveMotor(uint8_t chanFwd, uint8_t chanRev, int speed);
  void setMotors(int left_speed, int right_speed);  // Now enforces min/max speeds
  void stop();
  
  // Speed constraint methods - new functionality
  void setMinSpeed(int min);
  void setMaxSpeed(int max);
  int getMinSpeed() const { return minSpeed; }
  int getMaxSpeed() const { return maxSpeed; }
  
  // Status
  bool isMoving() const;
  int getLeftSpeed() const { return current_left_speed; }
  int getRightSpeed() const { return current_right_speed; }
  
  // Debug
  void printStatus();
  
  // Note: All sensor methods removed - now handled by LineFollower
};