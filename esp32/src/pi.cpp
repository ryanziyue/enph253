#include "pi.h"
#include "main.h"

PiComm::PiComm(MotorController* motor_ctrl, ServoController* servo_ctrl) 
  : motors(motor_ctrl), servos(servo_ctrl) {}

PiResponse PiComm::processCommand(const String& cmd) {
  Serial.print("Processing Pi command: ");
  Serial.println(cmd);
  
  if (!isValidCommand(cmd)) {
    return PiResponse(false, "Invalid command format");
  }
  
  // route to appropriate handler based on command type
  if (cmd.startsWith("PI:MC,")) {
    return handleMotorCommand(cmd);
  }
  else if (cmd.startsWith("PI:p,")) {
    return handleServoPositionCommand(cmd);
  }
  else if (cmd.startsWith("PI:s,")) {
    return handleServoSpeedCommand(cmd);
  }
  else if (cmd.startsWith("PI:GP,")) {
    return handleGlobalPositionCommand(cmd);
  }
  else if (cmd.startsWith("PI:GV,")) {
    return handleGlobalVelocityCommand(cmd);
  }
  else if (cmd.startsWith("PI:C,")) {
    return handleClawCommand(cmd);
  }
  else if (cmd.startsWith("PI:WL,")) {
    return handleWristLockCommand(cmd);
  }
  else if (cmd.equals("PI:CGP")) {
    return handleStatusRequest(cmd);
  }
  else {
    return PiResponse(false, "Unknown command: " + cmd);
  }
}

PiResponse PiComm::handleMotorCommand(const String& cmd) {
  // parse: PI:MC,x,y
  int x = 0, y = 0;
  if (sscanf(cmd.c_str(), "PI:MC,%d,%d", &x, &y) != 2) {
    return PiResponse(false, "Invalid motor command format");
  }
  
  // execute command
  motors->setMotors(x, y);
  
  return PiResponse(true, "Motors set to L=" + String(x) + " R=" + String(y));
}

PiResponse PiComm::handleServoPositionCommand(const String& cmd) {
  // parse: PI:p,base,shoulder,elbow,wrist
  String parts[4];
  int idx = 0, start = 5;
  
  for (int i = 5; i <= cmd.length() && idx < 4; i++) {
    if (i == cmd.length() || cmd.charAt(i) == ',') {
      parts[idx++] = cmd.substring(start, i);
      start = i + 1;
    }
  }
  
  if (idx < 4) {
    return PiResponse(false, "Invalid servo position format");
  }
  
  // execute commands (skip if "-")
  if (parts[0] != "-") servos->setTarget(IDX_BASE, parts[0].toFloat());
  if (parts[1] != "-") servos->setTarget(IDX_SHOULDER_L, parts[1].toFloat());
  if (parts[2] != "-") servos->setTarget(IDX_ELBOW, parts[2].toFloat());
  if (parts[3] != "-") servos->setTarget(IDX_WRIST, parts[3].toFloat());
  
  return PiResponse(true, "Servo positions set");
}

PiResponse PiComm::handleServoSpeedCommand(const String& cmd) {
  // parse: PI:s,base,shoulder,elbow,wrist
  String parts[4];
  int idx = 0, start = 5;
  
  for (int i = 5; i <= cmd.length() && idx < 4; i++) {
    if (i == cmd.length() || cmd.charAt(i) == ',') {
      parts[idx++] = cmd.substring(start, i);
      start = i + 1;
    }
  }
  
  if (idx < 4) {
    return PiResponse(false, "Invalid servo speed format");
  }
  
  // execute commands (skip if "-")
  if (parts[0] != "-") servos->setSpeed(IDX_BASE, parts[0].toFloat());
  if (parts[1] != "-") servos->setSpeed(IDX_SHOULDER_L, parts[1].toFloat());
  if (parts[2] != "-") servos->setSpeed(IDX_ELBOW, parts[2].toFloat());
  if (parts[3] != "-") servos->setSpeed(IDX_WRIST, parts[3].toFloat());
  
  return PiResponse(true, "Servo speeds set");
}

PiResponse PiComm::handleGlobalPositionCommand(const String& cmd) {
  // parse: PI:GP,x,y
  int comma = cmd.indexOf(',', 6);
  if (comma <= 0) {
    return PiResponse(false, "Invalid global position format");
  }
  
  float x = cmd.substring(6, comma).toFloat();
  float y = cmd.substring(comma + 1).toFloat();
  
  // execute command
  servos->setGlobalPosition(x, y);
  
  return PiResponse(true, "Moving to position (" + String(x, 2) + "," + String(y, 2) + ")");
}

PiResponse PiComm::handleGlobalVelocityCommand(const String& cmd) {
  // parse: PI:GV,vx,vy
  int comma = cmd.indexOf(',', 6);
  if (comma <= 0) {
    return PiResponse(false, "Invalid global velocity format");
  }
  
  float vx = cmd.substring(6, comma).toFloat();
  float vy = cmd.substring(comma + 1).toFloat();
  
  // execute command
  servos->setGlobalVelocity(vx, vy);
  
  return PiResponse(true, "Global velocity set to (" + String(vx, 2) + "," + String(vy, 2) + ")");
}

PiResponse PiComm::handleClawCommand(const String& cmd) {
  // parse: PI:C,angle
  float angle = cmd.substring(5).toFloat();
  
  // execute command
  servos->setClaw(angle);
  
  return PiResponse(true, "Claw set to " + String(angle, 1) + "°");
}

PiResponse PiComm::handleWristLockCommand(const String& cmd) {
  // parse: PI:WL,T,offset or PI:WL,F
  if (cmd.startsWith("PI:WL,T,")) {
    float offset = cmd.substring(9).toFloat();
    servos->setWristLock(true, offset);
    return PiResponse(true, "Wrist lock enabled with offset " + String(offset, 1) + "°");
  }
  else if (cmd.equals("PI:WL,F")) {
    servos->setWristLock(false);
    return PiResponse(true, "Wrist lock disabled");
  }
  else {
    return PiResponse(false, "Invalid wrist lock command");
  }
}

PiResponse PiComm::handleStatusRequest(const String& cmd) {
  // get current position from servo controller
  Point pos = servos->getCurrentPosition();
  
  // format response data
  String posData = String(pos.x, 2) + "," + String(pos.y, 2);
  
  return PiResponse(true, "Current position", posData);
}

void PiComm::sendResponse(const PiResponse& response) {
  if (response.success) {
    if (response.data.length() > 0) {
      // send data response (like position)
      Serial1.println("ESP:" + response.data);
    } else {
      // send acknowledgment
      Serial1.println("OK: " + response.message);
    }
  } else {
    // send error
    Serial1.println("ERROR: " + response.message);
  }
  
  // also log to serial for debugging
  Serial.print("Response: ");
  Serial.println(response.success ? "SUCCESS" : "FAILED");
  Serial.println("  Message: " + response.message);
  if (response.data.length() > 0) {
    Serial.println("  Data: " + response.data);
  }
}

void PiComm::sendPositionUpdate() {
  Point pos = servos->getCurrentPosition();
  String posData = "ESP:" + String(pos.x, 2) + "," + String(pos.y, 2);
  Serial1.println(posData);
}

bool PiComm::isValidCommand(const String& cmd) {
  return cmd.startsWith("PI:") && cmd.length() > 3;
}