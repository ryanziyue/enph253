// linefollower.cpp - Fixed version
#include "linefollower.h"
#include "main.h"

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
  unsigned long lastTime = millis();
  bool leftTurn = true;
  
  for (;;) {
    // Update sensors
    updateSensors();
    
    // Calculate delta time
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    
    // Determine turn direction based on outer sensors (same as working code)
    if (!offLine(R2)) {
      leftTurn = false;
    } else if (!offLine(L2)) {
      leftTurn = true;
    }
    
    // Check if both inner sensors are off the line
    if (offLine(R1) && offLine(L1)) {
      // OFF-LINE HANDLING - Fixed to match working code behavior
      if (leftTurn) {
        // Turn left: left motor backward, right motor forward
        motors->setMotors(-searchSpeed, searchSpeed);
      } else {
        // Turn right: left motor forward, right motor backward  
        motors->setMotors(searchSpeed, -searchSpeed);
      }
    } else {
      // Normal PID line following
      float currentPosition = getCurrentPosition();
      
      // Fixed error calculation to match working code
      // Working code uses: error = currentPosition (target implicitly 0)
      // Our target is 0 by default, so: error = currentPosition - 0 = currentPosition
      float error = currentPosition - targetPosition;
      
      float pidOutput = calculatePIDOutput(error, deltaTime);
      
      // Apply PID output to motors (same as working code)
      int leftSpeed = baseSpeed + (int)pidOutput;
      int rightSpeed = baseSpeed - (int)pidOutput;
      
      motors->setMotors(leftSpeed, rightSpeed);
    }
    
    vTaskDelay(pdMS_TO_TICKS(loopDelay));
  }
}

float LineFollower::calculatePIDOutput(float error, float deltaTime) {
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

// REMOVED: handleOffLine() method - now handled directly in main loop

float LineFollower::getCurrentPosition() {
  return sensorVoltages[L1] - sensorVoltages[R1];  // Same as working code
}

void LineFollower::setPID(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void LineFollower::setBaseSpeed(int base) {
  baseSpeed = constrain(base, 0, 255);
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
  Serial.print(", Kd: "); Serial.println(Kd);
  Serial.print("Target Position: "); Serial.println(targetPosition);
  Serial.print("Current Position: "); Serial.println(getCurrentPosition());
  if (motors) {
    Serial.print("Motor Status - Left: "); Serial.print(motors->getLeftSpeed());
    Serial.print(", Right: "); Serial.println(motors->getRightSpeed());
  }
}