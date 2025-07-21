#include "motor.h"
#include "main.h"

MotorController::MotorController():
  initialized(false), current_left_speed(0), current_right_speed(0) {}

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
  Serial.println("Motor controller initialized");
}

void MotorController::updateSensors() {
  int rawR1 = analogRead(ANALOG_PIN_R1);
  int rawL1 = analogRead(ANALOG_PIN_L1);
  int rawR2 = analogRead(ANALOG_PIN_R2);
  int rawL2 = analogRead(ANALOG_PIN_L2);

  // Convert to voltage
  sensorVoltages[R1] = rawR1 * 3.3 / 4095.0;
  sensorVoltages[L1] = rawL1 * 3.3 / 4095.0;
  sensorVoltages[R2] = rawR2 * 3.3 / 4095.0;
  sensorVoltages[L2] = rawL2 * 3.3 / 4095.0;
}

bool MotorController::offLine(int sensor) {
  if (sensor < 0 || sensor > 3) return false;
  return (sensorVoltages[sensor] < sensorThresholds[sensor]);
}

void MotorController::stopMotor(uint8_t chanFwd, uint8_t chanRev) {
  ledcWrite(chanFwd, 0);
  ledcWrite(chanRev, 0);
}

void MotorController::driveMotor(uint8_t chanFwd, uint8_t chanRev, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    ledcWrite(chanRev, 0);
    ledcWrite(chanFwd, speed);
  }
  else if (speed < 0) {
    ledcWrite(chanFwd, 0);
    ledcWrite(chanRev, -speed);
  }
  else {
    stopMotor(chanFwd, chanRev);
  }
}

void MotorController::setMotors(int left_speed, int right_speed) {
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

float MotorController::getSensorVoltage(int sensor) const {
  if (sensor >= 0 && sensor < 4) {
    return sensorVoltages[sensor];
  }
  return 0.0;
}

void MotorController::setSensorThreshold(int sensor, float threshold) {
  if (sensor >= 0 && sensor < 4) {
    sensorThresholds[sensor] = threshold;
  }
}

bool MotorController::isMoving() const {
  return (current_left_speed != 0 || current_right_speed != 0);
}

void MotorController::printSensorValues() {
  updateSensors();
  Serial.print("Sensors - R1: "); Serial.print(sensorVoltages[R1], 3);
  Serial.print(" | L1: "); Serial.print(sensorVoltages[L1], 3);
  Serial.print(" | R2: "); Serial.print(sensorVoltages[R2], 3);
  Serial.print(" | L2: "); Serial.print(sensorVoltages[L2], 3);
  Serial.print(" | Position: "); Serial.println(sensorVoltages[R1] - sensorVoltages[L1], 3);
}

void MotorController::printStatus() {
  Serial.println("=== Motor Controller Status ===");
  Serial.print("Initialized: "); Serial.println(initialized ? "Yes" : "No");
  Serial.print("Left Speed: "); Serial.println(current_left_speed);
  Serial.print("Right Speed: "); Serial.println(current_right_speed);
  Serial.print("Moving: "); Serial.println(isMoving() ? "Yes" : "No");
}