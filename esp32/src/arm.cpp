#include "arm.h"
#include "main.h"

ServoController::ServoController() : last_millis(0), last_ik_millis(0),
    ik_vel_enabled(false), ik_target_x(0), ik_target_y(0), ik_vx(0), ik_vy(0),
    wrist_lock_enabled(true), wrist_lock_angle(0), wrist_lock_disable_until(0), initialized(false) {
  
  // set max speeds
  max_speed[IDX_BASE] = IDX_BASE_SPEED;
  max_speed[IDX_SHOULDER_L] = IDX_SHOULDER_SPEED;
  max_speed[IDX_SHOULDER_R] = IDX_SHOULDER_SPEED;
  max_speed[IDX_ELBOW] = IDX_ELBOW_SPEED;
  max_speed[IDX_WRIST] = IDX_WRIST_SPEED; 
  
  // set home positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    home_pos[i] = 90.0;
    current_pos[i] = 90.0;
    target_pos[i] = 90.0;
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
  Serial.println("Servo controller initialized with custom servo library");
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
        float servo2 = ELBOW_OFFSET - th2;
        setShoulderTarget(th1);
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
  
  // update all joints except wrist if lock is on
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (wrist_lock_enabled && i == IDX_WRIST && millis() >= wrist_lock_disable_until) continue;
    
    // drive target by speed command
    if (fabs(speed_cmd[i]) > 1e-3) {
      target_pos[i] += speed_cmd[i] * dt;
      if (target_pos[i] < 0) { target_pos[i] = 0; speed_cmd[i] = 0; }
      else if (target_pos[i] > 180) { target_pos[i] = 180; speed_cmd[i] = 0; }
    }
    
    // slew toward target
    float delta = target_pos[i] - current_pos[i];
    if (fabs(delta) > 0.5) {
      float step = max_speed[i] * dt;
      if (fabs(delta) < step) step = fabs(delta);
      current_pos[i] += (delta > 0 ? 1 : -1) * step;
      servos[i].write(current_pos[i]);
    }
  }
}

void ServoController::setShoulderTarget(float angle) {
  angle = constrain(angle, 0, 180);
  target_pos[IDX_SHOULDER_L] = angle;
  target_pos[IDX_SHOULDER_R] = angle;
}

void ServoController::setTarget(int idx, float angle) {
  if (!initialized) return;
  angle = constrain(angle, 0, 180);
  
  if (idx == IDX_SHOULDER_L || idx == IDX_SHOULDER_R) {
    setShoulderTarget(angle);
  } else if (idx >= 0 && idx < NUM_SERVOS) {
    target_pos[idx] = angle;
  }
}

void ServoController::setSpeed(int idx, float speed) {
  if (!initialized) return;
  if (idx >= 0 && idx < NUM_SERVOS) {
    speed_cmd[idx] = speed;
  }
}

void ServoController::setShoulderSpeed(float speed) {
  if (!initialized) return;
  speed_cmd[IDX_SHOULDER_L] = speed;
  speed_cmd[IDX_SHOULDER_R] = speed;
}

void ServoController::stopAll() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    speed_cmd[i] = 0;
  }
}

void ServoController::resetPosition() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    target_pos[i] = home_pos[i];
    speed_cmd[i] = 0;
  }
}

void ServoController::zeroAllServos() {
  if (!initialized) return;
  
  Serial.println("Zeroing all servos...");
  
  // set all servo targets to 0 degrees
  for (int i = 0; i < NUM_SERVOS; i++) {
    target_pos[i] = 0.0;
    current_pos[i] = 0.0;
    speed_cmd[i] = 0.0;
    servos[i].write(0); // immediately write 0 to servo
    delay(100); // small delay between servo movements
  }
  
  // zero the claw as well
  claw.write(0);
  
  // disable any velocity mode
  ik_vel_enabled = false;
  ik_vx = 0;
  ik_vy = 0;
  
  Serial.println("All servos zeroed to 0 degrees");
}

void ServoController::setGlobalPosition(float x, float y) {
  if (!initialized) return;
  
  ik_vel_enabled = false;  // disable velocity mode
  
  float th1, th2;
  if (inverseKinematics(x, y, th1, th2)) {
    float servo2 = ELBOW_OFFSET - th2;
    setShoulderTarget(th1);
    setTarget(IDX_ELBOW, servo2);
    Serial.print("Moving to: ("); Serial.print(x); Serial.print(","); Serial.print(y); Serial.println(")");
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
  wrist_lock_angle = constrain(angle_degrees, WRIST_LOWER_LIMIT, WRIST_UPPER_LIMIT);  // Reasonable limits
  
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

void ServoController::applyWristLock() {
  // Check if temporarily disabled
  if (millis() < wrist_lock_disable_until) {
    return;  // Skip wrist lock while temporarily disabled
  }
  
  if (!wrist_lock_enabled) return;
  
  float angle_1 = current_pos[IDX_SHOULDER_L];  // Shoulder angle
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
    Serial.print("° (speed: "); Serial.print(speed_cmd[i]); Serial.println(")");
  }
  Point pos = getCurrentPosition();
  Serial.print("Wrist: ("); Serial.print(pos.x); Serial.print(", "); Serial.print(pos.y); Serial.println(")");
  Serial.print("Moving: "); Serial.println(isMoving() ? "YES" : "NO");
  Serial.print("IK velocity: "); Serial.println(ik_vel_enabled ? "ENABLED" : "DISABLED");
  
  // Enhanced wrist lock status with temporary disable info
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