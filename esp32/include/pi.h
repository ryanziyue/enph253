#pragma once
#include <Arduino.h>
#include "motor.h"
#include "arm.h"
#include "linefollower.h"

#define WRIST_DISABLE_TIME 5000

// response structure for pi commands
struct PiResponse {
  bool success;
  String message;
  String data;  // for responses like position data
  
  PiResponse(bool success = true, String message = "", String data = "") 
    : success(success), message(message), data(data) {}
};

// pi communication handler
class PiComm {
private:
  MotorController* motors;
  ServoController* servos;
  LineFollower* lineFollower;
  
  // command parsing helpers - MOTOR CONTROL
  PiResponse handleMotorCommand(const String& cmd);          // PI:MC,x,y
  PiResponse handleLineFollowToggle(const String& cmd);      // PI:LF,x
  PiResponse handleLineFollowSpeed(const String& cmd);       // PI:LFS,x
  
  // command parsing helpers - ARM CONTROL
  PiResponse handleServoPositionCommand(const String& cmd);  // PI:SP,a,b,c
  PiResponse handleWristPositionCommand(const String& cmd);  // PI:WP,a
  PiResponse handleClawPositionCommand(const String& cmd);   // PI:CP,a
  PiResponse handleGlobalPositionCommand(const String& cmd); // PI:GP,x,y
  PiResponse handleGlobalVelocityCommand(const String& cmd); // PI:GV,x,y
  PiResponse handleWristLockToggle(const String& cmd);       // PI:WLT,1/0
  PiResponse handleWristLockAngle(const String& cmd);        // PI:WLA,angle
  PiResponse handleWristLockTempDisable(const String& cmd);  // PI:WLTD,duration_ms
  
  // status and utility
  PiResponse handleStatusRequest(const String& cmd);

public:
  PiComm(MotorController* motor_ctrl, ServoController* servo_ctrl, LineFollower* line_follower);
  
  // main command processing
  PiResponse processCommand(const String& cmd);
  
  // utility functions
  void sendResponse(const PiResponse& response);
  void sendPositionUpdate();
  void sendLimitSwitchPressed();  // ESP:LS
  bool isValidCommand(const String& cmd);
};