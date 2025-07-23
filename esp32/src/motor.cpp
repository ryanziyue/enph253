#include "motor.h"
#include "main.h"

MotorController::MotorController():
  initialized(false), current_left_speed(0), current_right_speed(0),
  minSpeed(175), maxSpeed(255) {}  // Set minSpeed to 175 like working code

void MotorController::init() {
  // PWM channel setup
  ledcSetup(0, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(3, PWM_FREQ, PWM_RES_BITS);
  
  // Attach pins to PWM
  ledcAttachPin(M1_PIN_FWD, 0);
  ledcAttachPin(M1_PIN_REV, 1);
  ledcAttachPin(M2_PIN_FWD, 2);
  ledcAttachPin(M2_PIN_REV, 3);
  
  initialized = true;
}

void MotorController::setMinSpeed(int min) {
  minSpeed = constrain(min, 0, maxSpeed);
}

void MotorController::setMaxSpeed(int max) {
  maxSpeed = constrain(max, minSpeed, 255);
}

void MotorController::stopMotor(uint8_t chanFwd, uint8_t chanRev) {
  ledcWrite(chanFwd, 0);
  ledcWrite(chanRev, 0);
}

// EXACTLY match working code's driveMotor logic including the "bug"
void MotorController::driveMotor(uint8_t chanFwd, uint8_t chanRev, int speed) {
  // Constrain speed to -255 to 255 - exactly like working code
  if (speed > 255) {
    speed = 255;
  } else if (speed < -255) {
    speed = -255;
  }

  if (speed > 0) {
    ledcWrite(chanRev, 0);
    if (speed < minSpeed) {
      speed = minSpeed;
    }
    ledcWrite(chanFwd, speed);
  } else if (speed < 0) {
    ledcWrite(chanFwd, 0);
    // CRITICAL: Keep the "bug" from working code that makes it work!
    if (speed > -minSpeed) {
      speed = minSpeed;  // This is the "bug" - should be -minSpeed but working code has minSpeed!
    }
    ledcWrite(chanRev, -speed);
  } else {
    stopMotor(chanFwd, chanRev);
  }
}

void MotorController::setMotors(int left_speed, int right_speed) {
  // Don't apply any additional constraints - let driveMotor handle it exactly like working code
  current_left_speed = left_speed;
  current_right_speed = right_speed;
  driveMotor(M1_CHAN_FWD, M1_CHAN_REV, left_speed);
  driveMotor(M2_CHAN_FWD, M2_CHAN_REV, right_speed);
}

void MotorController::stop() {
  current_left_speed = 0;
  current_right_speed = 0;
  stopMotor(M1_CHAN_FWD, M1_CHAN_REV);
  stopMotor(M2_CHAN_FWD, M2_CHAN_REV);
}

bool MotorController::isMoving() const {
  return (current_left_speed != 0 || current_right_speed != 0);
}

void MotorController::printStatus() {
  Serial.println("=== Motor Controller Status ===");
  Serial.print("Initialized: "); Serial.println(initialized ? "Yes" : "No");
  Serial.print("Left Speed: "); Serial.println(current_left_speed);
  Serial.print("Right Speed: "); Serial.println(current_right_speed);
  Serial.print("Speed Limits: Min="); Serial.print(minSpeed);
  Serial.print(" Max="); Serial.println(maxSpeed);
  Serial.print("Moving: "); Serial.println(isMoving() ? "Yes" : "No");
}