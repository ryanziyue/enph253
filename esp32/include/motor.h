#pragma once
#include <Arduino.h>

class MotorController {
private:
  bool initialized;
  int current_left_speed;
  int current_right_speed;

public:
  // sensor constants
  static const int R1 = 0, L1 = 1, R2 = 2, L2 = 3;
  float sensorVoltages[4] = {0.0, 0.0, 0.0, 0.0};
  float sensorThresholds[4] = {0.3, 0.3, 0.3, 0.3};

  MotorController();
  void init();
  
  // basic motor control
  void stopMotor(uint8_t chanFwd, uint8_t chanRev);
  void driveMotor(uint8_t chanFwd, uint8_t chanRev, int speed);
  void setMotors(int left_speed, int right_speed);
  void stop();
  
  // sensor methods
  void updateSensors();
  bool offLine(int sensor);
  float getSensorVoltage(int sensor) const;
  void setSensorThreshold(int sensor, float threshold);
  
  // status
  bool isMoving() const;
  int getLeftSpeed() const { return current_left_speed; }
  int getRightSpeed() const { return current_right_speed; }
  
  // debug
  void printSensorValues();
  void printStatus();
};