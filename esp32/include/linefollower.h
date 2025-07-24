#pragma once
#include <Arduino.h>
#include "motor.h"

class LineFollower {
private:
  MotorController* motors;
  TaskHandle_t lineFollowTaskHandle = nullptr;
  bool running = false;
  
  // PID variables
  float Kp, Ki, Kd;
  float Ko; 
  float targetPosition;
  float currentPosition;
  float previousError;
  float integral;
  
  // speed settings
  int baseSpeed;
  int searchSpeed;
  
  // sensor variables
  float sensorVoltages[4];
  float sensorThresholds[4];
  
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
  void setPID(float kp, float ki, float kd);
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