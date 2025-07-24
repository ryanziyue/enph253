#include "linefollower.h"
#include "main.h"

LineFollower::LineFollower(MotorController* motorController) : motors(motorController) {
  Kp = K_P;
  Ki = K_I;
  Kd = K_D;
  Ko = K_O;
  targetPosition = TARGET_POSITION; 
  currentPosition = CURRENT_POSITION;
  baseSpeed = BASE_SPEED; 
  
  sensorThresholds[R1] = SENSOR_THRESHOLD_R1;
  sensorThresholds[L1] = SENSOR_THRESHOLD_L1;
  sensorThresholds[R2] = SENSOR_THRESHOLD_R2;
  sensorThresholds[L2] = SENSOR_THRESHOLD_L2;
}

LineFollower::~LineFollower() {
  stop();
}

bool LineFollower::start() {
  if (running) {
    return true;
  }
  
  if (!motors) {
    return false;
  }
  
  // Reset PID state for clean start
  resetPID();
  
  // Create the line following task
  BaseType_t result = xTaskCreatePinnedToCore(
    lineFollowTaskWrapper,
    "LineFollowTask",
    4096,
    this,
    2,  // higher priority for real-time control
    &lineFollowTaskHandle,
    1   // core 1
  );
  
  if (result == pdPASS) {
    running = true;
    return true;
  } else {
    return false;
  }
}

void LineFollower::stop() {
  if (!running) return;
  
  // delete the task
  if (lineFollowTaskHandle != nullptr) {
    vTaskDelete(lineFollowTaskHandle);
    lineFollowTaskHandle = nullptr;
  }
  
  running = false;
  
  // stop motors
  if (motors) {
    motors->stop();
  }
}

void LineFollower::updateSensors() {
  int rawR1 = analogRead(ANALOG_PIN_R1);
  int rawL1 = analogRead(ANALOG_PIN_L1);
  int rawR2 = analogRead(ANALOG_PIN_R2);
  int rawL2 = analogRead(ANALOG_PIN_L2);

  // convert to voltage
  sensorVoltages[R1] = rawR1 * 3.3 / 4095.0;
  sensorVoltages[L1] = rawL1 * 3.3 / 4095.0;
  sensorVoltages[R2] = rawR2 * 3.3 / 4095.0;
  sensorVoltages[L2] = rawL2 * 3.3 / 4095.0;
}

bool LineFollower::offLine(int sensor) {
  if (sensor < 0 || sensor > 3) return false;
  return (sensorVoltages[sensor] > sensorThresholds[sensor]);
}

float LineFollower::getSensorVoltage(int sensor) const {
  if (sensor >= 0 && sensor < 4) {
    return sensorVoltages[sensor];
  }
  return 0.0;
}

void LineFollower::setSensorThreshold(int sensor, float threshold) {
  if (sensor >= 0 && sensor < 4) {
    sensorThresholds[sensor] = threshold;
  }
}

void LineFollower::lineFollowTaskWrapper(void* parameter) {
  LineFollower* follower = static_cast<LineFollower*>(parameter);
  follower->lineFollowLoop();
}

void LineFollower::lineFollowLoop() {
  const int loopDelay = 10; // milliseconds
  bool leftTurn = true;
  
  for (;;) {
    // Update sensors
    updateSensors();
    
    // Determine turn direction based on outer sensors - EXACTLY like working code
    if (!offLine(R2)) {
      leftTurn = false;
    } else if (!offLine(L2)) {
      leftTurn = true;
    }
    
    // Check if both inner sensors are off the line
    if (offLine(R1) && offLine(L1)) {
      // OFF-LINE HANDLING - EXACTLY like working code
      if (leftTurn) {
        // Left motor backward, right motor forward using targetPosition
        motors->setMotors(-targetPosition, targetPosition);
      } else {
        // Left motor forward, right motor backward using targetPosition  
        motors->setMotors(targetPosition, -targetPosition);
      }
    } else {
      // Normal PID line following - EXACTLY like working code
      currentPosition = sensorVoltages[L1] - sensorVoltages[R1];
      
      // CRITICAL: error = currentPosition (NOT currentPosition - targetPosition)
      // Working code does: error = currentPosition;
      float error = currentPosition;
      
      // PID calculation - exactly like working code with Ko
      integral += error * (loopDelay / 1000.0);
      integral = constrain(integral, -100, 100);
      float derivative = (error - previousError) / (loopDelay / 1000.0);
      float output = (Kp * error + Ki * integral + Kd * derivative) * Ko;  // Multiply by Ko!
      
      // Apply PID output to motors - exactly like working code
      int leftSpeed = baseSpeed + (int)output;
      int rightSpeed = baseSpeed - (int)output;
      
      motors->setMotors(leftSpeed, rightSpeed);
      
      previousError = error;
    }
    
    vTaskDelay(pdMS_TO_TICKS(loopDelay));
  }
}

float LineFollower::calculatePIDOutput(float error, float deltaTime) {
  // This method is not used in the fixed version - PID is calculated directly in lineFollowLoop
  // Keeping for compatibility but adding Ko like working code
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100);
  
  float derivative = (error - previousError) / deltaTime;
  float output = (Kp * error + Ki * integral + Kd * derivative) * Ko;  // Include Ko
  
  previousError = error;
  return output;
}

float LineFollower::getCurrentPosition() {
  return currentPosition;  // Return the class member variable
}

void LineFollower::setPID(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void LineFollower::setKo(float ko) {
  Ko = ko;
}

void LineFollower::setBaseSpeed(int base) {
  baseSpeed = constrain(base, 0, 255);
}

void LineFollower::setSearchSpeed(int speed) {
  searchSpeed = constrain(speed, 0, 255);
}

void LineFollower::setTargetPosition(float target) {
  targetPosition = target;
}

void LineFollower::resetPID() {
  integral = 0.0;
  previousError = 0.0;
  Serial.println("Line Follower PID reset");
}

void LineFollower::printSensorValues() {
  updateSensors();
  Serial.print("Sensors - R1: "); Serial.print(sensorVoltages[R1], 3);
  Serial.print(" | L1: "); Serial.print(sensorVoltages[L1], 3);
  Serial.print(" | R2: "); Serial.print(sensorVoltages[R2], 3);
  Serial.print(" | L2: "); Serial.print(sensorVoltages[L2], 3);
  Serial.print(" | Position: "); Serial.println(sensorVoltages[L1] - sensorVoltages[R1], 3);
}

void LineFollower::printStatus() {
  Serial.println("=== Line Follower Status ===");
  Serial.print("Running: "); Serial.println(running ? "Yes" : "No");
  Serial.print("Base Speed: "); Serial.println(baseSpeed);
  Serial.print("Search Speed: "); Serial.println(searchSpeed);
  Serial.print("PID - Kp: "); Serial.print(Kp);
  Serial.print(", Ki: "); Serial.print(Ki);
  Serial.print(", Kd: "); Serial.print(Kd);
  Serial.print(", Ko: "); Serial.println(Ko);  // ADD Ko to status
  Serial.print("Target Position: "); Serial.println(targetPosition);
  Serial.print("Current Position: "); Serial.println(getCurrentPosition());
  if (motors) {
    Serial.print("Motor Status - Left: "); Serial.print(motors->getLeftSpeed());
    Serial.print(", Right: "); Serial.println(motors->getRightSpeed());
  }
}