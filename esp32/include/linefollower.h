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
  int minSpeed = 108;
  int searchSpeed = 100;  // Speed when searching for line
  
  // Control logic
  static void lineFollowTaskWrapper(void* parameter);
  void lineFollowLoop();
  float calculatePIDOutput(float currentPosition, float deltaTime);
  void handleOffLine();

public:
  LineFollower(MotorController* motorController);
  ~LineFollower();
  
  // Control methods
  bool start();
  void stop();
  bool isRunning() const { return running; }
  
  // Configuration
  void setPID(float kp, float ki = 0.0, float kd = 0.0);
  void setBaseSpeed(int base, int min = -1);
  void setSearchSpeed(int speed);
  void setTarget(float target);
  void resetPID();
  
  // Status and debug
  float getCurrentPosition();
  void printStatus();
};