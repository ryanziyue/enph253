#include "pi.h"
#include "main.h"

PiComm::PiComm(MotorController* motor_ctrl, ServoController* servo_ctrl, LineFollower* line_follower) 
  : motors(motor_ctrl), servos(servo_ctrl), lineFollower(line_follower) {}

PiResponse PiComm::processCommand(const String& cmd) {
  Serial.println(cmd);
  
  if (!isValidCommand(cmd)) {
    return PiResponse(false, "Invalid command format");
  }
  
  // MOTOR CONTROL COMMANDS
  if (cmd.startsWith("PI:MC,")) {
    return handleMotorCommand(cmd);
  }
  else if (cmd.startsWith("PI:LF,")) {
    return handleLineFollowToggle(cmd);
  }
  else if (cmd.startsWith("PI:LFS,")) {
    return handleLineFollowSpeed(cmd);
  }
  
  // ARM CONTROL COMMANDS
  else if (cmd.startsWith("PI:SP,")) {
    return handleServoPositionCommand(cmd);
  }
  else if (cmd.startsWith("PI:WP,")) {
    return handleWristPositionCommand(cmd);
  }
  else if (cmd.startsWith("PI:CP,")) {
    return handleClawPositionCommand(cmd);
  }
  else if (cmd.startsWith("PI:GP,")) {
    return handleGlobalPositionCommand(cmd);
  }
  else if (cmd.startsWith("PI:GV,")) {
    return handleGlobalVelocityCommand(cmd);
  }
  else if (cmd.startsWith("PI:WLT,")) {
    return handleWristLockToggle(cmd);
  }
  else if (cmd.startsWith("PI:WLA,")) {
    return handleWristLockAngle(cmd);
  }
  else if (cmd.startsWith("PI:WLTD,")) {
    return handleWristLockTempDisable(cmd);
  }
  
  // STATUS REQUESTS
  else if (cmd.equals("PI:STATUS")) {
    return handleStatusRequest(cmd);
  }
  else {
    return PiResponse(false, "Unknown command: " + cmd);
  }
}

// ------- MOTOR CONTROL COMMANDS ------- 

PiResponse PiComm::handleMotorCommand(const String& cmd) {
  // parse: PI:MC,x,y
  int x = 0, y = 0;
  if (sscanf(cmd.c_str(), "PI:MC,%d,%d", &x, &y) != 2) {
    return PiResponse(false, "Invalid motor command format. Use PI:MC,left,right");
  }
  
  // stop line following manual motor control takes priority
  if (lineFollower && lineFollower->isRunning()) {
    lineFollower->stop();
  }
  
  // Execute command
  motors->setMotors(x, y);
  
  return PiResponse(true, "Motors set to L=" + String(x) + " R=" + String(y));
}

PiResponse PiComm::handleLineFollowToggle(const String& cmd) {
  // cmd = 1 for on, 0 for off
  int toggle = 0;
  if (sscanf(cmd.c_str(), "PI:LF,%d", &toggle) != 1) {
    return PiResponse(false, "Invalid line follow toggle format. Use PI:LF,1 or PI:LF,0");
  }
  
  if (!lineFollower) {
    return PiResponse(false, "Line follower not available");
  }
  
  if (toggle == 1) {
    if (lineFollower->start()) {
      return PiResponse(true, "Line following started");
    } else {
      return PiResponse(false, "Failed to start line following");
    }
  } else if (toggle == 0) {
    lineFollower->stop();
    return PiResponse(true, "Line following stopped");
  } else {
    return PiResponse(false, "Invalid toggle value. Use 1 (on) or 0 (off)");
  }
}

PiResponse PiComm::handleLineFollowSpeed(const String& cmd) {
  // parse: PI:LFS,x
  int speed = 0;
  if (sscanf(cmd.c_str(), "PI:LFS,%d", &speed) != 1) {
    return PiResponse(false, "Invalid line follow speed format. Use PI:LFS,speed");
  }
  
  if (!lineFollower) {
    return PiResponse(false, "Line follower not available");
  }
  
  // Set base speed (assuming min speed is 80% of base speed)
  int minSpeed = speed * 0.8;
  lineFollower->setBaseSpeed(speed, minSpeed);
  
  return PiResponse(true, "Line follow speed set to " + String(speed));
}

// ------- ARM CONTROL COMMANDS ------- 

PiResponse PiComm::handleServoPositionCommand(const String& cmd) {
  // parse: PI:SP,a,b,c (base, shoulder, elbow)
  String parts[3];
  int idx = 0, start = 6; // after "PI:SP,"
  
  // parse comma-separated values
  for (int i = 6; i <= cmd.length() && idx < 3; i++) {
    if (i == cmd.length() || cmd.charAt(i) == ',') {
      parts[idx++] = cmd.substring(start, i);
      start = i + 1;
    }
  }
  
  if (idx < 3) {
    return PiResponse(false, "Invalid servo position format. Use PI:SP,base,shoulder,elbow");
  }
  
  // execute commands (skip if "-")
  if (parts[0] != "-") {
    int basePos = parts[0].toInt();
    servos->setTarget(IDX_BASE, basePos);
  }
  if (parts[1] != "-") {
    int shoulderPos = parts[1].toInt();
    servos->setTarget(IDX_SHOULDER_L, shoulderPos);  // This sets both shoulders
  }
  if (parts[2] != "-") {
    int elbowPos = parts[2].toInt();
    servos->setTarget(IDX_ELBOW, elbowPos);
  }
  
  return PiResponse(true, "Servo positions set: base=" + parts[0] + 
                         " shoulder=" + parts[1] + " elbow=" + parts[2]);
}

PiResponse PiComm::handleWristPositionCommand(const String& cmd) {
  // parse: PI:WP,a
  int wristPos = 0;
  if (sscanf(cmd.c_str(), "PI:WP,%d", &wristPos) != 1) {
    return PiResponse(false, "Invalid wrist position format. Use PI:WP,position");
  }
  
  // Temporarily disable wrist lock for manual control (will re-engage after 5 seconds)
  servos->temporarilyDisableWristLock(WRIST_DISABLE_TIME);
  servos->setTarget(IDX_WRIST, wristPos);
  
  return PiResponse(true, "Wrist position set to " + String(wristPos) + "° (wrist lock will re-engage in 5s)");
}

PiResponse PiComm::handleClawPositionCommand(const String& cmd) {
  // parse: PI:CP,a
  int clawPos = 0;
  if (sscanf(cmd.c_str(), "PI:CP,%d", &clawPos) != 1) {
    return PiResponse(false, "Invalid claw position format. Use PI:CP,position");
  }
  
  // execute command
  servos->setClaw(clawPos);
  
  return PiResponse(true, "Claw position set to " + String(clawPos) + "°");
}

PiResponse PiComm::handleGlobalPositionCommand(const String& cmd) {
  // parse: PI:GP,x,y
  float x = 0, y = 0;
  if (sscanf(cmd.c_str(), "PI:GP,%f,%f", &x, &y) != 2) {
    return PiResponse(false, "Invalid global position format. Use PI:GP,x,y");
  }
  
  servos->setGlobalPosition(x, y);
  
  return PiResponse(true, "Moving to position (" + String(x, 2) + "," + String(y, 2) + ")");
}

PiResponse PiComm::handleGlobalVelocityCommand(const String& cmd) {
  // parse: PI:GV,x,y
  float vx = 0, vy = 0;
  if (sscanf(cmd.c_str(), "PI:GV,%f,%f", &vx, &vy) != 2) {
    return PiResponse(false, "Invalid global velocity format. Use PI:GV,vx,vy");
  }
  
  servos->setGlobalVelocity(vx, vy);
  
  return PiResponse(true, "Global velocity set to (" + String(vx, 2) + "," + String(vy, 2) + ")");
}

PiResponse PiComm::handleWristLockToggle(const String& cmd) {
  // parse: PI:WLT,x (1=enable, 0=disable)
  int toggle = 0;
  if (sscanf(cmd.c_str(), "PI:WLT,%d", &toggle) != 1) {
    return PiResponse(false, "Invalid wrist lock toggle format. Use PI:WLT,1 or PI:WLT,0");
  }
  
  if (toggle == 1) {
    servos->setWristLock(true, servos->getWristLockAngle());
    return PiResponse(true, "Wrist lock enabled at " + String(servos->getWristLockAngle()) + "°");
  } else if (toggle == 0) {
    servos->setWristLock(false);
    return PiResponse(true, "Wrist lock disabled");
  } else {
    return PiResponse(false, "Invalid toggle value. Use 1 (enable) or 0 (disable)");
  }
}

PiResponse PiComm::handleWristLockAngle(const String& cmd) {
  // parse: PI:WLA,angle
  float angle = 0;
  if (sscanf(cmd.c_str(), "PI:WLA,%f", &angle) != 1) {
    return PiResponse(false, "Invalid wrist lock angle format. Use PI:WLA,angle");
  }
  
  servos->setWristLockAngle(angle);
  
  String status = servos->getWristLockAngle() == angle ? "" : " (clamped)";
  return PiResponse(true, "Wrist lock angle set to " + String(servos->getWristLockAngle()) + "°" + status);
}

PiResponse PiComm::handleWristLockTempDisable(const String& cmd) {
  // parse: PI:WLTD,duration_ms
  int duration = 5000;  // default 5 seconds
  if (sscanf(cmd.c_str(), "PI:WLTD,%d", &duration) != 1) {
    return PiResponse(false, "Invalid temp disable format. Use PI:WLTD,duration_ms");
  }
  
  // Clamp duration to reasonable limits (100ms to 30 seconds)
  duration = constrain(duration, 100, 30000);
  
  servos->temporarilyDisableWristLock(duration);
  
  return PiResponse(true, "Wrist lock temporarily disabled for " + String(duration) + "ms");
}

// ------- STATUS AND UTILITY COMMANDS ------- 

PiResponse PiComm::handleStatusRequest(const String& cmd) {
  // get arm, line following, motor status
  Point pos = servos->getCurrentPosition();
  String lfStatus = lineFollower ? (lineFollower->isRunning() ? "ON" : "OFF") : "N/A";
  String motorStatus = "L=" + String(motors->getLeftSpeed()) + " R=" + String(motors->getRightSpeed());
  
  // format response data
  String statusData = "Pos:(" + String(pos.x, 2) + "," + String(pos.y, 2) + ") " +
                     "LF:" + lfStatus + " " +
                     "Motors:" + motorStatus;
  
  return PiResponse(true, "System status", statusData);
}

// replies to PI command
void PiComm::sendResponse(const PiResponse& response) {
  if (response.success) {
    if (response.data.length() > 0) {
      // send data response (like status)
      Serial.println("ESP:" + response.data);
    } else {
      // send acknowledgment
      Serial.println("OK: " + response.message);
    }
  } else {
    // error
    Serial.println("ERROR: " + response.message);
  }
}

void PiComm::sendPositionUpdate() {
  Point pos = servos->getCurrentPosition();
  String posData = "ESP:" + String(pos.x, 2) + "," + String(pos.y, 2);
  Serial.println(posData);
}

void PiComm::sendLimitSwitchPressed() {
  Serial.println("ESP:LS");
}

bool PiComm::isValidCommand(const String& cmd) {
  return cmd.startsWith("PI:") && cmd.length() > 3;
}