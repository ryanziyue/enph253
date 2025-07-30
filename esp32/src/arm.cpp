#include "arm.h"
#include "main.h"

ServoController::ServoController() : last_millis(0), last_ik_millis(0),
    ik_vel_enabled(false), ik_target_x(0), ik_target_y(0), ik_vx(0), ik_vy(0),
    wrist_lock_enabled(true), wrist_lock_angle(0), wrist_lock_disable_until(0), 
    initialized(false), shoulder_r_offset(SHOULDER_R_OFFSET) {
  
  // set max speeds
  max_speed[IDX_BASE] = IDX_BASE_SPEED;
  max_speed[IDX_SHOULDER_L] = IDX_SHOULDER_SPEED;
  max_speed[IDX_SHOULDER_R] = IDX_SHOULDER_SPEED;
  max_speed[IDX_ELBOW] = IDX_ELBOW_SPEED;
  max_speed[IDX_WRIST] = IDX_WRIST_SPEED; 
  
  // set home positions - account for shoulder offset and constraints
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i == IDX_SHOULDER_L) {
      // Validate that default home position (90°) is within constraint
      float desired_home = 90.0;
      if (desired_home > shoulder_r_offset) {
        Serial.print("Warning: Default home position (90°) exceeds constraint (");
        Serial.print(shoulder_r_offset); Serial.println("°) - using max allowed");
        desired_home = shoulder_r_offset;
      }
      home_pos[i] = desired_home;
      current_pos[i] = desired_home;
      target_pos[i] = desired_home;
    } else if (i == IDX_SHOULDER_R) {
      // Calculate right shoulder home position based on offset
      float right_home = constrain(shoulder_r_offset - home_pos[IDX_SHOULDER_L], 0, 180);
      home_pos[i] = right_home;
      current_pos[i] = right_home;
      target_pos[i] = right_home;
    } else {
      home_pos[i] = 90.0;
      current_pos[i] = 90.0;
      target_pos[i] = 90.0;
    }
    speed_cmd[i] = 0.0;
  }
}

void ServoController::init() {
  // initialize servo manager
  ServoManager::init();
  
  // servo pin assignments
  int servo_pins[NUM_SERVOS] = {
    SERVO_BASE_PIN, SERVO_SHOULDER_L, SERVO_SHOULDER_R, 
    SERVO_ELBOW_PIN, SERVO_WRIST_PIN
  };
  
  // attach servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servos[i].attach(servo_pins[i], i + 4)) {  // pin and channel
      Serial.print("Failed to attach servo ");
      Serial.println(i);
      return;
    }
    // Write the correct position accounting for shoulder offset
    servos[i].write(current_pos[i]);
    delay(100);  // small delay between servo initializations
  }
  
  // attach claw
  if (!claw.attach(SERVO_CLAW_PIN, 9)) {  // pin and channel
    Serial.println("Failed to attach claw servo");
    return;
  }
  claw.write(90);
  
  last_millis = millis();
  last_ik_millis = millis();
  
  // seed IK target
  Point W = forwardKinematics(current_pos[IDX_SHOULDER_L], current_pos[IDX_ELBOW]);
  ik_target_x = W.x;
  ik_target_y = W.y;
  
  initialized = true;
  Serial.println("Servo controller initialized with opposite-facing shoulder servos");
  Serial.print("Right shoulder offset: "); Serial.println(shoulder_r_offset);
  Serial.println("Wrist lock enabled by default at 0° (level with ground)");
}

void ServoController::update() {
  if (!initialized) return;
  
  // ik velocity integration
  if (ik_vel_enabled) {
    unsigned long now = millis();
    float dt = (now - last_ik_millis) / 1000.0;
    last_ik_millis = now;
    
    if (dt > 0) {
      // advance wrist target
      ik_target_x += ik_vx * dt;
      ik_target_y += ik_vy * dt;
      
      // solve IK
      float th1, th2;
      if (inverseKinematics(ik_target_x, ik_target_y, th1, th2)) {
        // Check shoulder constraint for IK velocity mode
        if (th1 > shoulder_r_offset) {
          // Hit constraint - stop velocity mode
          ik_vel_enabled = false;
          stopAll();
          Serial.print("IK velocity stopped - shoulder constraint reached (");
          Serial.print(th1); Serial.print("° > "); Serial.print(shoulder_r_offset); Serial.println("°)");
          return;
        }
        
        float servo2 = ELBOW_OFFSET - th2;
        setShoulderTarget(th1);  // Use centralized method
        setTarget(IDX_ELBOW, servo2);
      } else {
        // hit boundary - stop
        ik_vel_enabled = false;
        stopAll();
        Serial.println("IK velocity stopped - out of reach");
      }
    }
  }
  
  // apply wrist lock
  applyWristLock();
  
  // update motion
  updateMotion();
}

void ServoController::updateMotion() {
  unsigned long now = millis();
  float dt = (now - last_millis) / 1000.0;
  if (dt > 0.05) dt = 0.05;
  last_millis = now;
  
  // Handle shoulder coordination first (speed-based movement)
  bool left_shoulder_moved = false;
  if (fabs(speed_cmd[IDX_SHOULDER_L]) > 1e-3) {
    target_pos[IDX_SHOULDER_L] += speed_cmd[IDX_SHOULDER_L] * dt;
    
    // Apply left shoulder constraint
    if (target_pos[IDX_SHOULDER_L] < 0) { 
      target_pos[IDX_SHOULDER_L] = 0; 
      speed_cmd[IDX_SHOULDER_L] = 0; 
      speed_cmd[IDX_SHOULDER_R] = 0; // Stop both shoulders
    }
    else if (target_pos[IDX_SHOULDER_L] > shoulder_r_offset) { 
      target_pos[IDX_SHOULDER_L] = shoulder_r_offset; 
      speed_cmd[IDX_SHOULDER_L] = 0;
      speed_cmd[IDX_SHOULDER_R] = 0; // Stop both shoulders
      Serial.println("Left shoulder hit constraint - both shoulders stopped");
    }
    
    // Update right shoulder to maintain coordination
    target_pos[IDX_SHOULDER_R] = convertToRightShoulderAngle(target_pos[IDX_SHOULDER_L]);
    left_shoulder_moved = true;
  }
  
  // Update all joints (skip shoulders if we handled them above, skip wrist if locked)
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (wrist_lock_enabled && i == IDX_WRIST && millis() >= wrist_lock_disable_until) continue;
    if (left_shoulder_moved && (i == IDX_SHOULDER_L || i == IDX_SHOULDER_R)) continue;
    
    // drive target by speed command for non-shoulder servos
    if (fabs(speed_cmd[i]) > 1e-3) {
      target_pos[i] += speed_cmd[i] * dt;
      
      // Standard constraint for non-shoulder servos
      if (target_pos[i] < 0) { target_pos[i] = 0; speed_cmd[i] = 0; }
      else if (target_pos[i] > 180) { target_pos[i] = 180; speed_cmd[i] = 0; }
    }
    
    // slew toward target for all servos
    float delta = target_pos[i] - current_pos[i];
    if (fabs(delta) > 0.5) {
      float step = max_speed[i] * dt;
      if (fabs(delta) < step) step = fabs(delta);
      current_pos[i] += (delta > 0 ? 1 : -1) * step;
      servos[i].write(current_pos[i]);
    }
  }
}

float ServoController::convertToRightShoulderAngle(float left_angle) {
  float right_angle = shoulder_r_offset - left_angle;
  return constrain(right_angle, 0, 180);
}

void ServoController::setShoulderTarget(float angle) {
  if (!initialized) return;
  
  // Constrain left shoulder to prevent right shoulder from going out of bounds
  // Right shoulder = offset - left, so left must be <= offset to keep right >= 0
  angle = constrain(angle, 0, shoulder_r_offset);
  
  // Set left shoulder
  target_pos[IDX_SHOULDER_L] = angle;
  
  // Set right shoulder to opposite angle with offset
  target_pos[IDX_SHOULDER_R] = convertToRightShoulderAngle(angle);
  
  Serial.print("Shoulder targets: L="); Serial.print(angle);
  Serial.print("° R="); Serial.print(target_pos[IDX_SHOULDER_R]); 
  Serial.print("° (L constrained to 0-"); Serial.print(shoulder_r_offset); Serial.println("°)");
}

void ServoController::setShoulderSpeed(float speed) {
  if (!initialized) return;
  
  speed_cmd[IDX_SHOULDER_L] = speed;
  speed_cmd[IDX_SHOULDER_R] = -speed;
}

void ServoController::setShoulderOffset(float offset) {
  shoulder_r_offset = constrain(offset, 0, 180);
  Serial.print("Right shoulder offset set to: "); Serial.println(shoulder_r_offset);
  
  // Recalculate home positions
  recalculateHomePositions();
  
  // If we're initialized, update the right shoulder position immediately
  if (initialized) {
    target_pos[IDX_SHOULDER_R] = convertToRightShoulderAngle(target_pos[IDX_SHOULDER_L]);
  }
}

void ServoController::recalculateHomePositions() {
  // Validate that left shoulder home position is within new constraint
  float desired_left_home = 90.0;
  if (desired_left_home > shoulder_r_offset) {
    Serial.print("Warning: Left shoulder home (90°) exceeds new constraint (");
    Serial.print(shoulder_r_offset); Serial.println("°) - adjusting to constraint limit");
    desired_left_home = shoulder_r_offset;
  }
  
  home_pos[IDX_SHOULDER_L] = desired_left_home;
  
  // Recalculate right shoulder home position based on new offset
  home_pos[IDX_SHOULDER_R] = constrain(shoulder_r_offset - desired_left_home, 0, 180);
  
  Serial.print("Home positions updated: L="); Serial.print(home_pos[IDX_SHOULDER_L]);
  Serial.print("° R="); Serial.print(home_pos[IDX_SHOULDER_R]);
  Serial.print("° (L constraint: 0-"); Serial.print(shoulder_r_offset); Serial.println("°)");
}

void ServoController::setTarget(int idx, float angle) {
  if (!initialized) return;
  
  if (idx == IDX_SHOULDER_L || idx == IDX_SHOULDER_R) {
    // For shoulder servos, always use the centralized method
    // This ensures both servos move together properly and respects constraints
    if (idx == IDX_SHOULDER_L) {
      setShoulderTarget(angle);  // This will apply the proper constraint
    } else {
      // If setting right shoulder directly, convert back to left shoulder angle
      float left_angle = shoulder_r_offset - angle;
      left_angle = constrain(left_angle, 0, shoulder_r_offset);  // Apply constraint
      setShoulderTarget(left_angle);
    }
  } else if (idx >= 0 && idx < NUM_SERVOS) {
    angle = constrain(angle, 0, 180);
    target_pos[idx] = angle;
  }
}

void ServoController::setSpeed(int idx, float speed) {
  if (!initialized) return;
  
  if (idx == IDX_SHOULDER_L || idx == IDX_SHOULDER_R) {
    // For shoulder servos, always use the centralized method
    setShoulderSpeed(speed);
  } else if (idx >= 0 && idx < NUM_SERVOS) {
    speed_cmd[idx] = speed;
  }
}

void ServoController::stopAll() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    speed_cmd[i] = 0;
  }
}

void ServoController::resetPosition() {
  // Use centralized shoulder method for reset to ensure proper coordination
  setShoulderTarget(home_pos[IDX_SHOULDER_L]);  // This will set both shoulders correctly
  
  // Set other servos directly
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i != IDX_SHOULDER_L && i != IDX_SHOULDER_R) {
      target_pos[i] = home_pos[i];
    }
    speed_cmd[i] = 0;
  }
}

void ServoController::zeroAllServos() {
  if (!initialized) return;
  
  Serial.println("Zeroing all servos...");
  
  // For shoulder servos, use centralized method
  setShoulderTarget(0.0);
  
  // For other servos, set directly
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i != IDX_SHOULDER_L && i != IDX_SHOULDER_R) {
      target_pos[i] = 0.0;
      current_pos[i] = 0.0;
      speed_cmd[i] = 0.0;
      servos[i].write(0);
      delay(100);
    }
  }
  
  // Set shoulder servos after calculation
  current_pos[IDX_SHOULDER_L] = 0.0;
  current_pos[IDX_SHOULDER_R] = convertToRightShoulderAngle(0.0);
  speed_cmd[IDX_SHOULDER_L] = 0.0;
  speed_cmd[IDX_SHOULDER_R] = 0.0;
  servos[IDX_SHOULDER_L].write(0);
  servos[IDX_SHOULDER_R].write(current_pos[IDX_SHOULDER_R]);
  delay(100);
  
  // zero the claw as well
  claw.write(0);
  
  // disable any velocity mode
  ik_vel_enabled = false;
  ik_vx = 0;
  ik_vy = 0;
  
  Serial.println("All servos zeroed");
}

void ServoController::setGlobalPosition(float x, float y) {
  if (!initialized) return;
  
  ik_vel_enabled = false;  // disable velocity mode
  
  float th1, th2;
  if (inverseKinematics(x, y, th1, th2)) {
    // Check if shoulder angle is within constraints
    if (th1 > shoulder_r_offset) {
      Serial.print("Warning: IK solution ("); Serial.print(th1);
      Serial.print("°) exceeds shoulder constraint ("); Serial.print(shoulder_r_offset);
      Serial.println("°) - target clamped");
      th1 = shoulder_r_offset;
    }
    
    float servo2 = ELBOW_OFFSET - th2;
    setShoulderTarget(th1);  // Use centralized method (will apply constraint)
    setTarget(IDX_ELBOW, servo2);
    Serial.print("Moving to: ("); Serial.print(x); Serial.print(","); Serial.print(y); 
    Serial.print(") - shoulder: "); Serial.print(th1); Serial.println("°");
  } else {
    Serial.println("Target out of reach!");
  }
}

void ServoController::setGlobalVelocity(float vx, float vy) {
  if (!initialized) return;
  
  ik_vx = vx;
  ik_vy = vy;
  
  // seed target from current position
  Point W = forwardKinematics(current_pos[IDX_SHOULDER_L], current_pos[IDX_ELBOW]);
  ik_target_x = W.x;
  ik_target_y = W.y;
  last_ik_millis = millis();
  ik_vel_enabled = true;
  
  Serial.print("GV enabled: vx="); Serial.print(vx); Serial.print(" vy="); Serial.println(vy);
}

void ServoController::setWristLock(bool enabled, float angle_degrees) {
  wrist_lock_enabled = enabled;
  wrist_lock_angle = constrain(angle_degrees, WRIST_LOWER_LIMIT, WRIST_UPPER_LIMIT);
  
  if (enabled) {
    Serial.print("Wrist lock enabled at "); 
    Serial.print(wrist_lock_angle); 
    Serial.println("° relative to horizontal");
  } else {
    Serial.println("Wrist lock disabled");
  }
}

void ServoController::setWristLockAngle(float angle_degrees) {
  wrist_lock_angle = constrain(angle_degrees, WRIST_LOWER_LIMIT, WRIST_UPPER_LIMIT);
  
  if (wrist_lock_enabled) {
    Serial.print("Wrist lock angle updated to "); 
    Serial.print(wrist_lock_angle); 
    Serial.println("° relative to horizontal");
  } else {
    Serial.print("Wrist lock angle set to "); 
    Serial.print(wrist_lock_angle); 
    Serial.println("° (will apply when lock is enabled)");
  }
}

void ServoController::temporarilyDisableWristLock(int duration_ms) {
  wrist_lock_disable_until = millis() + duration_ms;
  Serial.print("Wrist lock temporarily disabled for ");
  Serial.print(duration_ms);
  Serial.println("ms");
}

void ServoController::setClaw(float angle) {
  if (!initialized) return;
  angle = constrain(angle, 0, 180);
  claw.write(angle);
  Serial.print("Claw: "); Serial.println(angle);
}

void ServoController::setMaxSpeed(int idx, float maxSpeed) {
  if (!initialized) return;
  if (idx >= 0 && idx < NUM_SERVOS) {
    max_speed[idx] = maxSpeed;
  }
}

void ServoController::applyWristLock() {
  // Check if temporarily disabled
  if (millis() < wrist_lock_disable_until) {
    return;  // Skip wrist lock while temporarily disabled
  }
  
  if (!wrist_lock_enabled) return;
  
  float angle_1 = current_pos[IDX_SHOULDER_L];  // Use left shoulder angle for calculation
  float angle_2 = current_pos[IDX_ELBOW];       // Elbow angle
  
  // formula: 50.0 - angle_1 + angle_2 gives horizontal orientation (0°)
  // adding wrist_lock_angle tilts the wrist by that amount from horizontal
  float lockAng = 90.0 - angle_1 + angle_2 + wrist_lock_angle;
  lockAng = constrain(lockAng, 0.0, 180.0);
  
  servos[IDX_WRIST].write(lockAng);
  current_pos[IDX_WRIST] = lockAng;
  target_pos[IDX_WRIST] = lockAng;
}

// kinematics
Point ServoController::forwardKinematics(float theta1, float theta2) {
  float t1 = theta1 * DEG2RAD;
  float phi = (ELBOW_OFFSET - theta2) * DEG2RAD;
  
  Point W;
  W.x = ARM_L1 * cos(t1) + ARM_L2 * cos(t1 + phi);
  W.y = ARM_L1 * sin(t1) + ARM_L2 * sin(t1 + phi);

  return W;
}

bool ServoController::inverseKinematics(float x, float y, float &theta1, float &theta2) {
  float r2 = x*x + y*y;
  float cosPsi = (r2 - ARM_L1*ARM_L1 - ARM_L2*ARM_L2) / (2 * ARM_L1 * ARM_L2);
  if (cosPsi < -1.0 || cosPsi > 1.0) return false;
  
  float psi = acos(cosPsi);
  float alpha = atan2(y, x);
  float beta = atan2(ARM_L2 * sin(psi), ARM_L1 + ARM_L2 * cos(psi));
  
  theta1 = (alpha + beta) * RAD2DEG;
  theta2 = -psi * RAD2DEG;
  return true;
}

Point ServoController::getCurrentPosition() {
  return forwardKinematics(current_pos[IDX_SHOULDER_L], current_pos[IDX_ELBOW]);
}

bool ServoController::isMoving() const {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (fabs(target_pos[i] - current_pos[i]) > 0.5) return true;
    if (fabs(speed_cmd[i]) > 1e-3) return true;
  }
  return ik_vel_enabled;
}

void ServoController::printStatus() {
  Serial.println("=== Servo Status ===");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print("Joint "); Serial.print(i);
    Serial.print(": "); Serial.print(current_pos[i]);
    Serial.print("° → "); Serial.print(target_pos[i]);
    Serial.print("° (home: "); Serial.print(home_pos[i]);
    Serial.print("°, speed: "); Serial.print(speed_cmd[i]); Serial.println(")");
  }
  
  Serial.print("Right shoulder offset: "); Serial.print(shoulder_r_offset);
  Serial.print("° | Left shoulder constraint: 0-"); Serial.print(shoulder_r_offset); Serial.println("°");
  Serial.print("Shoulder mapping: L=0°→R="); Serial.print(shoulder_r_offset);
  Serial.print("°, L="); Serial.print(shoulder_r_offset); Serial.println("°→R=0°");
  
  Point pos = getCurrentPosition();
  Serial.print("Wrist: ("); Serial.print(pos.x); Serial.print(", "); Serial.print(pos.y); Serial.println(")");
  Serial.print("Moving: "); Serial.println(isMoving() ? "YES" : "NO");
  Serial.print("IK velocity: "); Serial.println(ik_vel_enabled ? "ENABLED" : "DISABLED");
  
  if (millis() < wrist_lock_disable_until) {
    unsigned long remaining = wrist_lock_disable_until - millis();
    Serial.print("Wrist lock: TEMPORARILY DISABLED (");
    Serial.print(remaining);
    Serial.println("ms remaining)");
  } else if (wrist_lock_enabled) {
    Serial.print("Wrist lock: ENABLED at "); 
    Serial.print(wrist_lock_angle); 
    Serial.println("° relative to horizontal");
  } else {
    Serial.print("Wrist lock: DISABLED (angle set to "); 
    Serial.print(wrist_lock_angle); 
    Serial.println("°)");
  }
  Serial.println("==================");
}