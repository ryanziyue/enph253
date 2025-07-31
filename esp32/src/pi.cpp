#include "pi.h"
#include "main.h"

PiComm::PiComm(MotorController* motor_ctrl, ServoController* servo_ctrl, LineFollower* line_follower) 
  : motors(motor_ctrl), servos(servo_ctrl), lineFollower(line_follower) {}

PiResponse PiComm::processCommand(const String& cmd) {
  Serial.println(cmd);
  
  if (!isValidCommand(cmd)) {
    return PiResponse(false, "Invalid command format");
  }
  
  if (cmd.startsWith("PI:PID,")) {
    return handlePIDSettingCommand(cmd);
  }
  else if (cmd.startsWith("PI:TP,")) {
    return handleTargetPositionCommand(cmd);
  }
  else if (cmd.startsWith("PI:ST,")) {
    return handleSensorThresholdCommand(cmd);
  }
  else if (cmd.startsWith("PI:MC,")) {
    return handleMotorCommand(cmd);
  }
  else if (cmd.startsWith("PI:LF,")) {
    return handleLineFollowToggle(cmd);
  }
  else if (cmd.startsWith("PI:LBS,")) {
    return handleBaseSpeed(cmd);
  }
  else if (cmd.startsWith("PI:LMS,")) {
    return handleMinSpeed(cmd);
  }
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
  else if (cmd.startsWith("PI:SS,")) {
    return handleAllServoSpeedsCommand(cmd);
  }
  else if (cmd.startsWith("PI:SMS,")) {
    return handleAllServoMaxSpeedsCommand(cmd);
  }
  else if (cmd.startsWith("PI:REF")) {
    return handleReflectanceDataCommand(cmd);
  }
  else if (cmd.equals("PI:STATUS")) {
    return handleStatusRequest(cmd);
  }
  else {
    return PiResponse(false, "Unknown command: " + cmd);
  }
}

// ------- LINEFOLLOWING COMMANDS -------
PiResponse PiComm::handlePIDSettingCommand(const String& cmd) {
  float kp, ki, kd;
  if (sscanf(cmd.c_str(), "PI:PID,%f,%f,%f", &kp, &ki, &kd) != 3) {
    return PiResponse(false, "Invalid PID command format. Use PI:PID,kp,ki,kd");
  }

  lineFollower->setPID(kp, ki, kd);

  return PiResponse(true, "PID values set to Kp = " + String(kp) + " , Ki = " + String(ki) + ", Kd = " + String(kd));
}

PiResponse PiComm::handleTargetPositionCommand(const String& cmd) {
  float tp;
  if (sscanf(cmd.c_str(), "PI:TP,%f", &tp) != 1) {
    return PiResponse(false, "Invalid target position command format. Use PI:TP,x");
  }

  lineFollower->setTargetPosition(tp);

  return PiResponse(true, "Target position set to " + String(tp));
}

PiResponse PiComm::handleSensorThresholdCommand(const String& cmd) {
  float r1, l1, r2, l2;
  if (sscanf(cmd.c_str(), "PI:ST,%f,%f,%f,%f", &r1, &l1, &r2, &l2) != 4) {
    return PiResponse(false, "Invalid sensor threshold command. Use PI:ST,r1,l1,r2,l2");
  }

  float thresholds[4] = {r1, l1, r2, l2};

  for (int i = 0; i < 4; i++) {
    lineFollower->setSensorThreshold(i, thresholds[i]);
  }

  return PiResponse(true, "Sensor thresholds set to r1 = " + String(r1) + " , l1 = " + String(l1) + ", r2 = " + String(r2) + ", l2 = " + String(l2));
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

PiResponse PiComm::handleBaseSpeed(const String& cmd) {
  // parse: PI:LBS,x
  int speed = 0;
  if (sscanf(cmd.c_str(), "PI:LBS,%d", &speed) != 1) {
    return PiResponse(false, "Invalid line follow speed format. Use PI:LBS,speed");
  }
  
  if (!lineFollower) {
    return PiResponse(false, "Line follower not available");
  }
  
  // set base speed in LineFollower
  lineFollower->setBaseSpeed(speed);
  
  return PiResponse(true, "Base speed set to " + String(speed));
}

PiResponse PiComm::handleMinSpeed(const String& cmd) {
  // parse: PI:LMS,x
  int speed = 0;
  if (sscanf(cmd.c_str(), "PI:LMS,%d", &speed) != 1) {
    return PiResponse(false, "Invalid line follow speed format. Use PI:LMS,speed");
  }
  
  if (!lineFollower) {
    return PiResponse(false, "Line follower not available");
  }

  motors->setMinSpeed(speed);

  return PiResponse(true, "Min speed set to " + String(speed));
}

PiResponse PiComm::handleReflectanceDataCommand(const String& cmd) {

  lineFollower->updateSensors();

  float voltageR1 = lineFollower->getSensorVoltage(0);
  float voltageL1 = lineFollower->getSensorVoltage(1);
  float voltageR2 = lineFollower->getSensorVoltage(2);
  float voltageL2 = lineFollower->getSensorVoltage(3);

  String s;
  s.reserve(32);

  s  = "ESP:RF,"
    + String(voltageR1, 3) + ','
    + String(voltageL1, 3) + ','
    + String(voltageR2, 3) + ','
    + String(voltageL2, 3);

  return PiResponse(true, s);
  
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
    servos->setShoulderTarget(shoulderPos);  // Use centralized shoulder method directly
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
  
  // Simply disable wrist lock for manual control
  servos->setWristLock(false);
  servos->setTarget(IDX_WRIST, wristPos);
  
  return PiResponse(true, "Wrist position set to " + String(wristPos) + "° (wrist lock disabled)");
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

PiResponse PiComm::handleAllServoSpeedsCommand(const String& cmd) {
  // parse: PI:SS,base,shoulder,elbow,wrist
  String parts[4];
  int idx = 0, start = 6;

  // parse comma-separated values
  for (int i = 6; i <= cmd.length() && idx < 4; i++) {
    if (i == cmd.length() || cmd.charAt(i) == ',') {
      parts[idx++] = cmd.substring(start, i);
      start = i + 1;
    }
  }
  
  if (idx < 4) {
    return PiResponse(false, "Invalid servo speeds format. Use PI:SS,base,shoulder,elbow,wrist");
  }

  // execute commands (skip if "-")
  if (parts[0] != "-") {
    parts[0].trim();
    float baseSpeed = parts[0].toFloat();
    servos->setSpeed(IDX_BASE, baseSpeed);
  }
  if (parts[1] != "-") {
    parts[1].trim();
    float shoulderSpeed = parts[1].toFloat();
    servos->setShoulderSpeed(shoulderSpeed);
  }
  if (parts[2] != "-") {
    parts[2].trim();
    float elbowSpeed = parts[2].toFloat();
    servos->setSpeed(IDX_ELBOW, elbowSpeed);
  }
  if (parts[3] != "-") {
    parts[3].trim();
    float wristSpeed = parts[3].toFloat();
    servos->setSpeed(IDX_WRIST, wristSpeed);
  }
  
  return PiResponse(true, "Servo speeds set: base=" + parts[0] + 
                         " shoulder=" + parts[1] + " elbow=" + parts[2] + " wrist=" + parts[3]);
}

PiResponse PiComm::handleAllServoMaxSpeedsCommand(const String& cmd) {
  // parse: PI:SMS,base,shoulder,elbow,wrist
  String parts[4];
  int idx = 0, start = 7; // after "PI:SMS,"
  
  // parse comma-separated values
  for (int i = 7; i <= cmd.length() && idx < 4; i++) {
    if (i == cmd.length() || cmd.charAt(i) == ',') {
      parts[idx++] = cmd.substring(start, i);
      start = i + 1;
    }
  }
  
  if (idx < 4) {
    return PiResponse(false, "Invalid servo max speeds format. Use PI:SMS,base,shoulder,elbow,wrist");
  }
  
  // execute commands (skip if "-")
  if (parts[0] != "-") {
    parts[0].trim();
    float baseMaxSpeed = parts[0].toFloat();
    servos->setMaxSpeed(IDX_BASE, baseMaxSpeed);
  }
  if (parts[1] != "-") {
    parts[1].trim();
    float shoulderMaxSpeed = parts[1].toFloat();
    servos->setMaxSpeed(IDX_SHOULDER_L, shoulderMaxSpeed);
    servos->setMaxSpeed(IDX_SHOULDER_R, shoulderMaxSpeed);
  }
  if (parts[2] != "-") {
    parts[2].trim();
    float elbowMaxSpeed = parts[2].toFloat();
    servos->setMaxSpeed(IDX_ELBOW, elbowMaxSpeed);
  }
  if (parts[3] != "-") {
    parts[3].trim();
    float wristMaxSpeed = parts[3].toFloat();
    servos->setMaxSpeed(IDX_WRIST, wristMaxSpeed);
  }
  
  return PiResponse(true, "Servo max speeds set: base=" + parts[0] + 
                         " shoulder=" + parts[1] + " elbow=" + parts[2] + " wrist=" + parts[3]);
}

// ------- STATUS AND UTILITY COMMANDS ------- 

PiResponse PiComm::handleStatusRequest(const String& cmd) {
  // get arm, line following, motor status
  Point pos = servos->getCurrentPosition();
  String lfStatus = lineFollower ? (lineFollower->isRunning() ? "ON" : "OFF") : "N/A";
  String motorStatus = "L=" + String(motors->getLeftSpeed()) + " R=" + String(motors->getRightSpeed());
  
  // Add speed constraint info
  String speedLimits = " Min=" + String(motors->getMinSpeed()) + " Max=" + String(motors->getMaxSpeed());
  
  // format response data
  String statusData = "Pos:(" + String(pos.x, 2) + "," + String(pos.y, 2) + ") " +
                     "LF:" + lfStatus + " " +
                     "Motors:" + motorStatus + speedLimits;
  
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
      Serial.println(response.message);
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