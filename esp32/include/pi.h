#pragma once
#include <Arduino.h>
#include "motor.h"
#include "arm.h"
#include "linefollower.h"

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
  
  // command parsing helpers - LINE FOLLOWING CONTROL
  PiResponse handlePIDSettingCommand(const String& cmd);            // PI:PID,kp,ki,kd,ko
  PiResponse handleTargetPositionCommand(const String& cmd);        // PI:TP,x
  PiResponse handleSensorThresholdCommand(const String& cmd);       // PI:ST,r1,l1,r2,l2
  PiResponse handleLineFollowToggle(const String& cmd);             // PI:LF,x
  PiResponse handleBaseSpeed(const String& cmd);                    // PI:LBS,x
  PiResponse handleMinSpeed(const String& cmd);                     // PI:LMS,x
  PiResponse handleReflectanceDataCommand(const String& cmd);       // PI:REF

  // command parsing helpers - MOTOR CONTROL
  PiResponse handleMotorCommand(const String& cmd);                 // PI:MC,x,y

  
  // command parsing helpers - ARM CONTROL
  PiResponse handleServoPositionCommand(const String& cmd);         // PI:SP,a,b,c
  PiResponse handleWristPositionCommand(const String& cmd);         // PI:WP,a
  PiResponse handleClawPositionCommand(const String& cmd);          // PI:CP,a
  PiResponse handleGlobalPositionCommand(const String& cmd);        // PI:GP,x,y
  PiResponse handleGlobalVelocityCommand(const String& cmd);        // PI:GV,x,y
  PiResponse handleWristLockToggle(const String& cmd);              // PI:WLT,1/0
  PiResponse handleWristLockAngle(const String& cmd);               // PI:WLA,angle
  PiResponse handleAllServoSpeedsCommand(const String& cmd);        // PI:SS,base,shoulder,elbow,wrist
  PiResponse handleAllServoMaxSpeedsCommand(const String& cmd);     // PI:SMS,base,shoulder,elbow,wrist
  
  // status and utility
  PiResponse handleStatusRequest(const String& cmd);

public:
  PiComm(MotorController* motor_ctrl, ServoController* servo_ctrl, LineFollower* line_follower);
  
  // main command processing
  PiResponse processCommand(const String& cmd);
  
  // utility functions
  void sendResponse(const PiResponse& response);            // ESP:OK/ERROR:msg
  void sendPositionUpdate();                                // ESP:x,y
  bool isValidCommand(const String& cmd);
};