#pragma once
#include <Arduino.h>
#include "motor.h"
#include "arm.h"

// response structure for pi commands
struct PiResponse {
  bool success;
  String message;
  String data;  // For responses like position data
  
  PiResponse(bool success = true, String message = "", String data = "") 
    : success(success), message(message), data(data) {}
};

// pi communication handler
class PiComm {
private:
  MotorController* motors;
  ServoController* servos;
  
  // command parsing helpers
  PiResponse handleMotorCommand(const String& cmd);
  PiResponse handleServoPositionCommand(const String& cmd);
  PiResponse handleServoSpeedCommand(const String& cmd);
  PiResponse handleGlobalPositionCommand(const String& cmd);
  PiResponse handleGlobalVelocityCommand(const String& cmd);
  PiResponse handleClawCommand(const String& cmd);
  PiResponse handleWristLockCommand(const String& cmd);
  PiResponse handleStatusRequest(const String& cmd);

public:
  PiComm(MotorController* motor_ctrl, ServoController* servo_ctrl);
  
  // main command processing
  PiResponse processCommand(const String& cmd);
  
  // utility functions
  void sendResponse(const PiResponse& response);
  void sendPositionUpdate();
  bool isValidCommand(const String& cmd);
};