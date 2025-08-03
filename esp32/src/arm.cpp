#include "arm.h"
#include "main.h"

ServoController::ServoController() : last_millis(0), last_ik_millis(0),
    ik_vel_enabled(false), ik_target_x(0), ik_target_y(0), ik_vx(0), ik_vy(0),
    wrist_lock_enabled(true), wrist_lock_angle(0), initialized(false) {
  
  // set max speeds
  max_speed[IDX_BASE] = IDX_BASE_SPEED;
  max_speed[IDX_SHOULDER_L] = IDX_SHOULDER_SPEED;
  max_speed[IDX_SHOULDER_R] = IDX_SHOULDER_SPEED;
  max_speed[IDX_ELBOW] = IDX_ELBOW_SPEED;
  max_speed[IDX_WRIST] = IDX_WRIST_SPEED; 
  
  // set home positions - account for fixed shoulder offset and base offset
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i == IDX_BASE) {
      // Base home position: user angle 90° maps to servo angle 100°
      float home_user_angle = 90.0;
      float home_servo_angle = map(home_user_angle, 0, 180, BASE_SERVO_OFFSET, 180 - BASE_SERVO_OFFSET);
      home_pos[i] = home_servo_angle;
      current_pos[i] = home_servo_angle;
      target_pos[i] = home_servo_angle;
    }
    else if (i == IDX_SHOULDER_L) {
      float desired_home = 90.0;
      if (desired_home > SHOULDER_R_OFFSET) {
        desired_home = SHOULDER_R_OFFSET;
      }
      home_pos[i] = desired_home;
      current_pos[i] = desired_home;
      target_pos[i] = desired_home;
    }
    else if (i == IDX_SHOULDER_R) {
      float right_home = constrain(SHOULDER_R_OFFSET - home_pos[IDX_SHOULDER_L], 0, 180);
      home_pos[i] = right_home;
      current_pos[i] = right_home;
      target_pos[i] = right_home;
    }
    else {
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
    // Write the correct position
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
  Serial.println("Servo controller initialized with EXACT working kinematics pattern");
  Serial.print("Arm geometry: L1="); Serial.print(ARM_L1); 
  Serial.print("cm L2="); Serial.print(ARM_L2); 
  Serial.print("cm ELBOW_OFFSET="); Serial.print(ELBOW_OFFSET); Serial.println("°");
  Serial.print("Max reach: "); Serial.print(ARM_L1 + ARM_L2);
  Serial.print("cm Min reach: "); Serial.println(fabs(ARM_L1 - ARM_L2));
}

// FIXED: Updated IK velocity handling in update() method
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
        // FIXED: Check constraints but don't immediately stop - just clamp
        bool constrained = false;
        
        if (th1 > SHOULDER_R_OFFSET || th1 < 0) {
          th1 = constrain(th1, 0, SHOULDER_R_OFFSET);
          constrained = true;
          Serial.print("IK velocity: shoulder clamped to "); Serial.println(th1);
        }
        
        // Convert IK result to servo angle
        float servo2 = ELBOW_OFFSET - th2;
        
        if (servo2 < 0 || servo2 > 180) {
          servo2 = constrain(servo2, 0, 180);
          constrained = true;
          Serial.print("IK velocity: elbow clamped to "); Serial.println(servo2);
        }
        
        // Only stop if we're consistently hitting constraints AND have zero velocity
        if (constrained && fabs(ik_vx) < 0.1 && fabs(ik_vy) < 0.1) {
          ik_vel_enabled = false;
          Serial.println("IK velocity stopped - low velocity at constraint boundary");
          return;
        }
        
        setShoulderTarget(th1);
        setTarget(IDX_ELBOW, servo2);
      } else {
        // FIXED: Don't stop immediately, try to back off
        Serial.println("IK velocity: out of reach, backing off...");
        ik_target_x -= ik_vx * dt * 0.5;  // back off by half step
        ik_target_y -= ik_vy * dt * 0.5;
        
        // If velocity is pushing us further out of reach, stop
        float current_dist = sqrt(ik_target_x*ik_target_x + ik_target_y*ik_target_y);
        float max_reach = ARM_L1 + ARM_L2;
        if (current_dist > max_reach && (ik_vx*ik_target_x + ik_vy*ik_target_y) > 0) {
          ik_vel_enabled = false;
          stopAll();
          Serial.println("IK velocity stopped - moving away from workspace");
        }
      }
    }
  }
  
  // apply wrist lock
  if (wrist_lock_enabled) {
    applyWristLock();
  }
  
  // update motion
  updateMotion();
}

void ServoController::updateMotion() {
  unsigned long now = millis();
  float dt = (now - last_millis) / 1000.0;
  if (dt > 0.05) dt = 0.05;
  last_millis = now;
  
  // Handle shoulder coordination (your existing code stays the same)
  bool left_shoulder_moved = false;
  if (fabs(speed_cmd[IDX_SHOULDER_L]) > 1e-3) {
    float new_target = target_pos[IDX_SHOULDER_L] + speed_cmd[IDX_SHOULDER_L] * dt;
    
    if (new_target < 0) { 
      target_pos[IDX_SHOULDER_L] = 0; 
    }
    else if (new_target > SHOULDER_R_OFFSET) { 
      target_pos[IDX_SHOULDER_L] = SHOULDER_R_OFFSET; 
    }
    else {
      target_pos[IDX_SHOULDER_L] = new_target;
    }
    
    target_pos[IDX_SHOULDER_R] = convertToRightShoulderAngle(target_pos[IDX_SHOULDER_L]);
    left_shoulder_moved = true;
  }
  
  // update all joints
  for (int i = 0; i < NUM_SERVOS; i++) {
    // CHANGE: Skip wrist if locked OR manually controlled
    if (i == IDX_WRIST && (wrist_lock_enabled || isWristManuallyControlled())) continue;
    if (left_shoulder_moved && (i == IDX_SHOULDER_L || i == IDX_SHOULDER_R)) continue;
    
    // drive target by speed command (your existing code stays the same)
    if (fabs(speed_cmd[i]) > 1e-3) {
      float new_target = target_pos[i] + speed_cmd[i] * dt;
      
      if (i == IDX_BASE) {
        if (new_target < BASE_SERVO_OFFSET) { 
          target_pos[i] = BASE_SERVO_OFFSET; 
        }
        else if (new_target > (180 - BASE_SERVO_OFFSET)) { 
          target_pos[i] = 180 - BASE_SERVO_OFFSET; 
        }
        else {
          target_pos[i] = new_target;
        }
      } else {
        if (new_target < 0) { 
          target_pos[i] = 0; 
        }
        else if (new_target > 180) { 
          target_pos[i] = 180; 
        }
        else {
          target_pos[i] = new_target;
        }
      }
    }
    
    // slew toward target (your existing code stays the same)
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
  float right_angle = SHOULDER_R_OFFSET - left_angle;
  return constrain(right_angle, 0, 180);
}

void ServoController::setShoulderTarget(float angle) {
  if (!initialized) return;
  
  // constrain left shoulder to prevent right shoulder from going out of bounds
  angle = constrain(angle, 0, SHOULDER_R_OFFSET);
  
  // set left shoulder
  target_pos[IDX_SHOULDER_L] = angle;
  
  // set right shoulder to opposite angle with fixed offset
  target_pos[IDX_SHOULDER_R] = convertToRightShoulderAngle(angle);
  
  Serial.print("Shoulder targets: L="); Serial.print(angle);
  Serial.print("° R="); Serial.print(target_pos[IDX_SHOULDER_R]); 
  Serial.print("° (L constrained to 0-"); Serial.print(SHOULDER_R_OFFSET); Serial.println("°)");
}

void ServoController::setShoulderSpeed(float speed) {
  if (!initialized) return;
  
  speed_cmd[IDX_SHOULDER_L] = speed;
  speed_cmd[IDX_SHOULDER_R] = -speed;
}

// Base target with offset compensation
void ServoController::setBaseTarget(float user_angle) {
  if (!initialized) return;
  
  // Map user angle (0-180, perpendicular at 90°) to servo angle (10-170, perpendicular at 100°)
  float servo_angle = map(user_angle, 0, 180, BASE_SERVO_OFFSET, 180 - BASE_SERVO_OFFSET);
  servo_angle = constrain(servo_angle, BASE_SERVO_OFFSET, 180 - BASE_SERVO_OFFSET);
  
  target_pos[IDX_BASE] = servo_angle;
  
  Serial.print("Base target: user="); Serial.print(user_angle);
  Serial.print("° → servo="); Serial.print(servo_angle); 
  if (user_angle == 90.0) Serial.print("° [PERPENDICULAR]");
  Serial.println("°");
}

void ServoController::setTarget(int idx, float angle) {
  if (!initialized) return;
  
  if (idx == IDX_BASE) {
    Serial.println("Note: Use setBaseTarget() for proper base offset compensation");
  }
  else if (idx == IDX_SHOULDER_L || idx == IDX_SHOULDER_R) {
    if (idx == IDX_SHOULDER_L) {
      setShoulderTarget(angle);
    } else {
      float left_angle = SHOULDER_R_OFFSET - angle;
      left_angle = constrain(left_angle, 0, SHOULDER_R_OFFSET);
      setShoulderTarget(left_angle);
    }
    return;
  }
  else if (idx == IDX_WRIST) {
    // CHANGE: Track when wrist is manually positioned
    wrist_manual_control_time = millis();
    Serial.print("Wrist manually positioned to "); Serial.print(angle); 
    Serial.println("° - manual control active for 2 seconds");
  }
  
  if (idx >= 0 && idx < NUM_SERVOS) {
    if (idx == IDX_BASE) {
      angle = constrain(angle, BASE_SERVO_OFFSET, 180 - BASE_SERVO_OFFSET);
    } else {
      angle = constrain(angle, 0, 180);
    }
    target_pos[idx] = angle;
  }
}

void ServoController::setSpeed(int idx, float speed) {
  if (!initialized) return;
  
  if (idx == IDX_SHOULDER_L || idx == IDX_SHOULDER_R) {
    setShoulderSpeed(speed);
  } else if (idx >= 0 && idx < NUM_SERVOS) {
    speed_cmd[idx] = speed;
  }
}

void ServoController::clearSpeedCommands() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    speed_cmd[i] = 0;
  }
  Serial.println("All speed commands cleared");
}

void ServoController::stopAll() {
  clearSpeedCommands();
  ik_vel_enabled = false;
  ik_vx = 0;
  ik_vy = 0;
  Serial.println("All motion stopped");
}

void ServoController::resetPosition() {
  // Use centralized shoulder method for reset to ensure proper coordination
  setShoulderTarget(home_pos[IDX_SHOULDER_L]);  // This will set both shoulders correctly
  
  // Set base using offset compensation
  float base_user_angle = map(home_pos[IDX_BASE], BASE_SERVO_OFFSET, 180 - BASE_SERVO_OFFSET, 0, 180);
  setBaseTarget(base_user_angle);
  
  // Set other servos directly
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i != IDX_SHOULDER_L && i != IDX_SHOULDER_R && i != IDX_BASE) {
      target_pos[i] = home_pos[i];
    }
    speed_cmd[i] = 0;
  }
}

void ServoController::zeroAllServos() {
  if (!initialized) return;
  
  Serial.println("Zeroing all servos...");
  
  // For base servo, use minimum position (which is BASE_SERVO_OFFSET)
  target_pos[IDX_BASE] = BASE_SERVO_OFFSET;
  current_pos[IDX_BASE] = BASE_SERVO_OFFSET;
  speed_cmd[IDX_BASE] = 0.0;
  servos[IDX_BASE].write(BASE_SERVO_OFFSET);
  delay(100);
  
  // For shoulder servos, use centralized method
  setShoulderTarget(0.0);
  
  // For other servos, set directly
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i != IDX_SHOULDER_L && i != IDX_SHOULDER_R && i != IDX_BASE) {
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
    if (th1 > SHOULDER_R_OFFSET || th1 < 0) {
      Serial.print("Warning: IK solution shoulder angle ("); Serial.print(th1);
      Serial.print("°) outside constraint range (0-"); Serial.print(SHOULDER_R_OFFSET);
      Serial.println("°) - target clamped");
      th1 = constrain(th1, 0, SHOULDER_R_OFFSET);
    }
    
    // Convert IK result to servo angle - EXACT working pattern
    float servo2 = ELBOW_OFFSET - th2;
    
    // Check elbow servo angle constraints
    if (servo2 < 0 || servo2 > 180) {
      Serial.print("Warning: IK solution elbow servo ("); Serial.print(servo2);
      Serial.println("°) outside servo range (0-180°) - target clamped");
      servo2 = constrain(servo2, 0, 180);
    }
    
    setShoulderTarget(th1);  // Use centralized method
    setTarget(IDX_ELBOW, servo2);
    
    Serial.print("Moving to: ("); Serial.print(x); Serial.print(","); Serial.print(y); 
    Serial.print(") - shoulder: "); Serial.print(th1); 
    Serial.print("° IK_theta2: "); Serial.print(th2);
    Serial.print("° servo: "); Serial.print(servo2); Serial.println("°");
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
  // CHANGE: Clear manual control state when toggling lock
  if (wrist_lock_enabled != enabled) {
    speed_cmd[IDX_WRIST] = 0;
    wrist_manual_control_time = 0;  // Clear manual control timer
    Serial.println("Wrist state cleared - switching lock mode");
  }
  
  wrist_lock_enabled = enabled;
  wrist_lock_angle = constrain(angle_degrees, WRIST_LOWER_LIMIT, WRIST_UPPER_LIMIT);
  
  if (enabled) {
    Serial.print("Wrist lock enabled at "); 
    Serial.print(wrist_lock_angle); 
    Serial.print("° relative to horizontal (range: ");
    Serial.print(WRIST_LOWER_LIMIT); Serial.print("° to "); Serial.print(WRIST_UPPER_LIMIT); Serial.println("°)");
    
    // CHANGE: Immediately apply the lock
    applyWristLock();
  } else {
    Serial.println("Wrist lock disabled - wrist moves freely");
  }
}

void ServoController::setWristLockAngle(float angle_degrees) {
  wrist_lock_angle = constrain(angle_degrees, WRIST_LOWER_LIMIT, WRIST_UPPER_LIMIT);
  
  if (wrist_lock_enabled) {
    Serial.print("Wrist lock angle updated to "); 
    Serial.print(wrist_lock_angle); 
    Serial.print("° relative to horizontal");
    
    // Show approximate servo position for this angle (assumes 0° shoulder/elbow)
    float approx_servo = 75.0 + wrist_lock_angle;  // Corrected formula
    Serial.print(" [~"); Serial.print(approx_servo); Serial.println("° servo position]");
  } else {
    Serial.print("Wrist lock angle set to "); 
    Serial.print(wrist_lock_angle); 
    Serial.println("° (will apply when lock is enabled)");
  }
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
  if (!wrist_lock_enabled) return;
  
  // CHANGE: Don't override manual control during timeout period
  if (isWristManuallyControlled()) {
    unsigned long time_remaining = 2000 - (millis() - wrist_manual_control_time);
    Serial.print("Wrist lock waiting - manual control active (");
    Serial.print(time_remaining); Serial.println("ms remaining)");
    return;
  }
  
  // Your existing applyWristLock calculation stays the same
  float shoulder_angle = current_pos[IDX_SHOULDER_L];
  float elbow_servo_angle = current_pos[IDX_ELBOW];
  
  float elbow_joint_angle = ELBOW_OFFSET - elbow_servo_angle;
  float second_arm_angle = shoulder_angle + elbow_joint_angle;
  float desired_wrist_from_extension = wrist_lock_angle - second_arm_angle;
  float lockAng = desired_wrist_from_extension + WRIST_SERVO_ALIGNED;
  
  lockAng = constrain(lockAng, 0, 180);
  
  servos[IDX_WRIST].write(lockAng);
  current_pos[IDX_WRIST] = lockAng;
  target_pos[IDX_WRIST] = lockAng;
}

// Enhanced wrist diagnostic function
void ServoController::diagnoseWristIssue() {
  Serial.println("=== KINEMATICS DIAGNOSTIC ===");
  Serial.print("Arm geometry: L1="); Serial.print(ARM_L1);
  Serial.print("cm L2="); Serial.print(ARM_L2);
  Serial.print("cm ELBOW_OFFSET="); Serial.print(ELBOW_OFFSET); Serial.println("°");
  Serial.print("Max reach: "); Serial.print(ARM_L1 + ARM_L2);
  Serial.print("cm Min reach: "); Serial.println(fabs(ARM_L1 - ARM_L2));
  Serial.println();
  
  Serial.println("Forward/Inverse Kinematics Test:");
  float test_points[][2] = {{15, 15}, {20, 10}, {10, 20}, {25, 5}, {5, 25}, {30, 0}};
  
  for (int i = 0; i < 6; i++) {
    float target_x = test_points[i][0];
    float target_y = test_points[i][1];
    float distance = sqrt(target_x*target_x + target_y*target_y);
    
    Serial.print("Target: ("); Serial.print(target_x); Serial.print(","); Serial.print(target_y);
    Serial.print(") dist="); Serial.print(distance, 1); Serial.print(" → ");
    
    float th1, th2;
    if (inverseKinematics(target_x, target_y, th1, th2)) {
      float servo2 = ELBOW_OFFSET - th2;
      Serial.print("IK: S="); Serial.print(th1, 1); Serial.print("° th2="); Serial.print(th2, 1);
      Serial.print("° servo="); Serial.print(servo2, 1); Serial.print("° → ");
      
      // Test forward kinematics round-trip
      Point result = forwardKinematics(th1, servo2);
      
      Serial.print("FK: ("); Serial.print(result.x, 2); Serial.print(","); Serial.print(result.y, 2); Serial.print(") ");
      
      float error = sqrt((result.x - target_x)*(result.x - target_x) + (result.y - target_y)*(result.y - target_y));
      Serial.print("Error: "); Serial.print(error, 3);
      
      if (error < 0.01) Serial.println(" ✓EXCELLENT");
      else if (error < 0.1) Serial.println(" ✓GOOD");
      else Serial.println(" ✗ERROR!");
    } else {
      Serial.println("OUT OF REACH");
    }
  }
  
  Serial.println("========================");
}

// EXACT working pattern: Forward kinematics
Point ServoController::forwardKinematics(float theta1, float theta2) {
  // theta1: shoulder joint angle (0-180°)
  // theta2: elbow servo angle (0-180°)
  
  // EXACT same pattern as working code
  float t1 = theta1 * DEG2RAD;
  float phi = (ELBOW_OFFSET - theta2) * DEG2RAD;
  
  // Standard 2-DOF arm forward kinematics
  Point W;
  W.x = ARM_L1 * cos(t1) + ARM_L2 * cos(t1 + phi);
  W.y = ARM_L1 * sin(t1) + ARM_L2 * sin(t1 + phi);

  return W;
}

// EXACT working pattern: Inverse kinematics  
bool ServoController::inverseKinematics(float x, float y, float &theta1, float &theta2) {
  // EXACT same pattern as working code
  float r2 = x*x + y*y;
  float cosPsi = (r2 - ARM_L1*ARM_L1 - ARM_L2*ARM_L2) / (2 * ARM_L1 * ARM_L2);
  if (cosPsi < -1.0 || cosPsi > 1.0) return false;
  
  float psi = acos(cosPsi);
  float alpha = atan2(y, x);
  float beta = atan2(ARM_L2 * sin(psi), ARM_L1 + ARM_L2 * cos(psi));
  
  // EXACT same sign convention as working code
  theta1 = (alpha + beta) * RAD2DEG;  // PLUS beta
  theta2 = -psi * RAD2DEG;            // NEGATIVE psi
  
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
    if (i == IDX_BASE) Serial.print(" (Base)");
    else if (i == IDX_SHOULDER_L) Serial.print(" (Shoulder L)");
    else if (i == IDX_SHOULDER_R) Serial.print(" (Shoulder R)");
    else if (i == IDX_ELBOW) Serial.print(" (Elbow)");
    else if (i == IDX_WRIST) Serial.print(" (Wrist)");
    
    Serial.print(": "); Serial.print(current_pos[i]);
    Serial.print("° → "); Serial.print(target_pos[i]);
    Serial.print("° (home: "); Serial.print(home_pos[i]);
    Serial.print("°, speed: "); Serial.print(speed_cmd[i]); Serial.println(")");
  }
  
  Point pos = getCurrentPosition();
  Serial.print("Wrist: ("); Serial.print(pos.x); Serial.print(", "); Serial.print(pos.y); Serial.println(")");
  Serial.print("Moving: "); Serial.println(isMoving() ? "YES" : "NO");
  Serial.print("IK velocity: "); Serial.println(ik_vel_enabled ? "ENABLED" : "DISABLED");
  
  if (wrist_lock_enabled) {
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