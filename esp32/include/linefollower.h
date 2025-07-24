// linefollower.h - Fixed version
#pragma once
#include <Arduino.h>
#include "motor.h"

class LineFollower {
private:
  MotorController* motors;
  TaskHandle_t lineFollowTaskHandle = nullptr;
  bool running = false;
  
  // PID variables - MATCH working code exactly
  float Kp = 45.0, Ki = 0.0, Kd = 0.0;  // Kp = 45, not 1.0!
  float Ko = 2.0;  // ADD Ko parameter like working code
  float targetPosition = 220.0;  // 220, not 0!
  float currentPosition = 0.0;
  float previousError = 0.0;
  float integral = 0.0;
  
  // Speed settings - MATCH working code exactly
  int baseSpeed = 190;  // 190, not 150!
  int searchSpeed = 120;
  
  // Sensor variables - MATCH working code thresholds
  float sensorVoltages[4] = {0.0, 0.0, 0.0, 0.0};
  float sensorThresholds[4] = {1.7, 1.7, 1.8, 1.8};  // NOT 0.3!
  
  // Control logic
  static void lineFollowTaskWrapper(void* parameter);
  void lineFollowLoop();
  float calculatePIDOutput(float error, float deltaTime);

public:
  // Sensor constants
  static const int R1 = 0, L1 = 1, R2 = 2, L2 = 3;
  
  LineFollower(MotorController* motorController);
  ~LineFollower();
  
  // Control methods
  bool start();
  void stop();
  bool isRunning() const { return running; }
  
  // Configuration
  void setPID(float kp, float ki = 0.0, float kd = 0.0);
  void setKo(float ko);  // ADD Ko setter method
  void setBaseSpeed(int base);
  void setSearchSpeed(int speed);
  void setTarget(float target);
  void resetPID();
  
  // Sensor methods
  void updateSensors();
  bool offLine(int sensor);
  float getSensorVoltage(int sensor) const;
  void setSensorThreshold(int sensor, float threshold);
  void printSensorValues();
  
  // Status and debug
  float getCurrentPosition();
  void printStatus();
};