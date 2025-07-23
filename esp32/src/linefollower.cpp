#include "linefollower.h"

LineFollower::LineFollower(MotorController* motorController) : motors(motorController) {
  if (!motors) {
    // Serial.println("Error: MotorController pointer is null!");
  }
}

LineFollower::~LineFollower() {
  stop();
}

bool LineFollower::start() {
  if (running) {
    // Serial.println("Line follower already running");
    return true;
  }
  
  if (!motors) {
    // Serial.println("Error: No motor controller available");
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
    2,  // Higher priority for real-time control
    &lineFollowTaskHandle,
    1   // Core 1
  );
  
  if (result == pdPASS) {
    running = true;
    // Serial.println("Sensor-based line following started");
    return true;
  } else {
    // Serial.println("Failed to create line following task");
    return false;
  }
}

void LineFollower::stop() {
  if (!running) return;
  
  // Delete the task
  if (lineFollowTaskHandle != nullptr) {
    vTaskDelete(lineFollowTaskHandle);
    lineFollowTaskHandle = nullptr;
  }
  
  running = false;
  
  // Stop motors
  if (motors) {
    motors->stop();
  }
  
}

void LineFollower::lineFollowTaskWrapper(void* parameter) {
  LineFollower* follower = static_cast<LineFollower*>(parameter);
  follower->lineFollowLoop();
}

void LineFollower::lineFollowLoop() {
  const int loopDelay = 10; // milliseconds
  unsigned long lastTime = millis();
  bool leftTurn = true;
  
  for (;;) {
    // Update sensors
    motors->updateSensors();
    
    // Calculate delta time
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;
    
    // Determine turn direction based on outer sensors
    if (!motors->offLine(MotorController::R2)) {
      leftTurn = false;
    } else if (!motors->offLine(MotorController::L2)) {
      leftTurn = true;
    }
    
    // Check if both inner sensors are off the line
    if (motors->offLine(MotorController::R1) && motors->offLine(MotorController::L1)) {
      handleOffLine();
    } else {
      // Normal PID line following
      float currentPosition = getCurrentPosition();
      float pidOutput = calculatePIDOutput(currentPosition, deltaTime);
      
      // Apply PID output to motors
      int leftSpeed = baseSpeed + (int)pidOutput;
      int rightSpeed = baseSpeed - (int)pidOutput;
      
      motors->setMotors(leftSpeed, rightSpeed);
    }
    
    vTaskDelay(pdMS_TO_TICKS(loopDelay));
  }
}

float LineFollower::calculatePIDOutput(float currentPosition, float deltaTime) {
  float error = currentPosition - targetPosition;
  
  // Integral term with windup protection
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100);
  
  // Derivative term
  float derivative = (error - previousError) / deltaTime;
  
  // Calculate PID output
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Store for next iteration
  previousError = error;
  
  return output;
}

void LineFollower::handleOffLine() {
  // Robot is completely off the line - implement search pattern
  // This is a simple left/right search - you can make it more sophisticated
  
  static bool searchLeft = true;
  static unsigned long searchStartTime = millis();
  
  // Search for 500ms in each direction
  if (millis() - searchStartTime > 500) {
    searchLeft = !searchLeft;
    searchStartTime = millis();
  }
  
  if (searchLeft) {
    // Turn left
    motors->setMotors(-searchSpeed, searchSpeed);
  } else {
    // Turn right  
    motors->setMotors(searchSpeed, -searchSpeed);
  }
}

float LineFollower::getCurrentPosition() {
  return motors->getSensorVoltage(MotorController::L1) - 
         motors->getSensorVoltage(MotorController::R1);
}

void LineFollower::setPID(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void LineFollower::setBaseSpeed(int base, int min) {
  baseSpeed = constrain(base, 0, 255);
  if (min >= 0) {
    minSpeed = constrain(min, 0, baseSpeed);
  }
}

void LineFollower::setSearchSpeed(int speed) {
  searchSpeed = constrain(speed, 0, 255);
}

void LineFollower::setTarget(float target) {
  targetPosition = target;
}

void LineFollower::resetPID() {
  integral = 0.0;
  previousError = 0.0;
  Serial.println("Line Follower PID reset");
}

void LineFollower::printStatus() {
  Serial.println("=== Line Follower Status ===");
  Serial.print("Running: "); Serial.println(running ? "Yes" : "No");
  Serial.print("Base Speed: "); Serial.println(baseSpeed);
  Serial.print("Search Speed: "); Serial.println(searchSpeed);
  Serial.print("PID - Kp: "); Serial.print(Kp);
  Serial.print(", Ki: "); Serial.print(Ki);
  Serial.print(", Kd: "); Serial.println(Kd);
  Serial.print("Target Position: "); Serial.println(targetPosition);
  Serial.print("Current Position: "); Serial.println(getCurrentPosition());
  if (motors) {
    Serial.print("Motor Status - Left: "); Serial.print(motors->getLeftSpeed());
    Serial.print(", Right: "); Serial.println(motors->getRightSpeed());
  }
}