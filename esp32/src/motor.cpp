#include "motor.h"
#include "main.h"

MotorController::MotorController() 
  : m1_pin_fwd(M1_PIN_FWD), m1_pin_rev(M1_PIN_REV),
    m2_pin_fwd(M2_PIN_FWD), m2_pin_rev(M2_PIN_REV),
    current_left_speed(0), current_right_speed(0),
    initialized(false) {}

void MotorController::init() {
  ledcSetup(0, PWM_FREQ, PWM_RES_BITS);  // channel 0 for m1 forward
  ledcSetup(1, PWM_FREQ, PWM_RES_BITS);  // channel 1 for m1 reverse
  ledcSetup(2, PWM_FREQ, PWM_RES_BITS);  // channel 2 for m2 forward
  ledcSetup(3, PWM_FREQ, PWM_RES_BITS);  // channel 3 for m2 reverse
  
  // attach pins to pwm
  ledcAttachPin(m1_pin_fwd, 0);
  ledcAttachPin(m1_pin_rev, 1);
  ledcAttachPin(m2_pin_fwd, 2);
  ledcAttachPin(m2_pin_rev, 3);
  
  stop();
  initialized = true;
  
  Serial.println("Motor controller initialized");
}

void MotorController::driveMotor(int pin_fwd, int pin_rev, int speed, bool forward) {
  // map pins to pwm channels
  int channel_fwd, channel_rev;
  if (pin_fwd == m1_pin_fwd) {
    channel_fwd = 0;
    channel_rev = 1;
  } else {
    channel_fwd = 2;
    channel_rev = 3;
  }
  
  if (forward) {
    ledcWrite(channel_rev, 0);
    ledcWrite(channel_fwd, speed);
  } else {
    ledcWrite(channel_fwd, 0);
    ledcWrite(channel_rev, speed);
  }
}

void MotorController::stopMotor(int pin_fwd, int pin_rev) {
  // map pins to pwm channels
  int channel_fwd, channel_rev;
  if (pin_fwd == m1_pin_fwd) {
    channel_fwd = 0;
    channel_rev = 1;
  } else {
    channel_fwd = 2;
    channel_rev = 3;
  }
  
  ledcWrite(channel_fwd, 0);
  ledcWrite(channel_rev, 0);
}

void MotorController::setMotors(int left_speed, int right_speed) {
  if (!initialized) return;
  
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // left motor
  if (left_speed == 0) {
    stopMotor(m1_pin_fwd, m1_pin_rev);
  } else if (left_speed > 0) {
    driveMotor(m1_pin_fwd, m1_pin_rev, left_speed, true);
  } else {
    driveMotor(m1_pin_fwd, m1_pin_rev, -left_speed, false);
  }
  
  // right motor
  if (right_speed == 0) {
    stopMotor(m2_pin_fwd, m2_pin_rev);
  } else if (right_speed > 0) {
    driveMotor(m2_pin_fwd, m2_pin_rev, right_speed, true);
  } else {
    driveMotor(m2_pin_fwd, m2_pin_rev, -right_speed, false);
  }
  
  // update state
  current_left_speed = left_speed;
  current_right_speed = right_speed;
}

void MotorController::stop() {
  stopMotor(m1_pin_fwd, m1_pin_rev);
  stopMotor(m2_pin_fwd, m2_pin_rev);
  current_left_speed = 0;
  current_right_speed = 0;
}

bool MotorController::isMoving() const {
  return (current_left_speed != 0 || current_right_speed != 0);
}

void MotorController::printStatus() {
  Serial.println("=== Motor Status ===");
  Serial.print("Left: "); Serial.println(current_left_speed);
  Serial.print("Right: "); Serial.println(current_right_speed);
  Serial.print("Moving: "); Serial.println(isMoving() ? "YES" : "NO");
  Serial.println("==================");
}