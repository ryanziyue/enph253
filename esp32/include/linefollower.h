// linefollower.h - Fixed version
#pragma once
#include <Arduino.h>
#include "motor.h"

class LineFollower {
private:
  MotorController* motors;
  TaskHandle_t lineFollowTaskHandle = nullptr;
  bool running = false;
  
  // PID variables
  float Kp = 1.0, Ki = 0.0, Kd = 0.0;
  float targetPosition = 0.0;
  float previousError = 0.0;
  float integral = 0.0;
  
  // Speed settings
  int baseSpeed = 150;
  int searchSpeed = 120;  // Increased from 100 for better off-line recovery
  
  // Sensor variables
  float sensorVoltages[4] = {0.0, 0.0, 0.0, 0.0};
  float sensorThresholds[4] = {0.3, 0.3, 0.3, 0.3};
  
  // Control logic
  static void lineFollowTaskWrapper(void* parameter);
  void lineFollowLoop();
  float calculatePIDOutput(float error, float deltaTime);
  // REMOVED: handleOffLine() - now handled in main loop

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