#include <Arduino.h>
#include "main.h"
#include "arm.h"
#include "motor.h"
#include "linefollower.h"
#include "pi.h"

// create controller instances
ServoController arm;
MotorController motors;
LineFollower sensorLineFollower(&motors);
PiComm piComm(&motors, &arm, &sensorLineFollower);

// limit switch variables
bool lastLimitSwitchState = false;
unsigned long lastLimitSwitchTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce

// system state
bool systemInitialized = false;

void checkLimitSwitch();
void handleLocalCommand(String);

void setup() {
  Serial.begin(115200);
  
  // initialize limit switch pin
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  // initialize controllers
  motors.init();
  arm.init();
  
  motors.setMinSpeed(MOTOR_MIN_SPEED);
  motors.setMaxSpeed(MOTOR_MAX_SPEED);  
  
  sensorLineFollower.setBaseSpeed(BASE_SPEED);
  sensorLineFollower.setTargetPosition(TARGET_POSITION);
  sensorLineFollower.setPID(K_P, K_I, K_D);
  sensorLineFollower.setKo(K_O);
  
  // configure sensor thresholds
  sensorLineFollower.setSensorThreshold(LineFollower::R1, SENSOR_THRESHOLD_R1);
  sensorLineFollower.setSensorThreshold(LineFollower::L1, SENSOR_THRESHOLD_L1);
  sensorLineFollower.setSensorThreshold(LineFollower::R2, SENSOR_THRESHOLD_R2);
  sensorLineFollower.setSensorThreshold(LineFollower::L2, SENSOR_THRESHOLD_L2);
  
  delay(1000);
  arm.resetPosition();
  
  systemInitialized = true;
}

void loop() {
  if (!systemInitialized) return;
  
  // always update arm controller
  arm.update();
  
  // Handle serial commands (both Pi and local debug)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      // Check if it's a Pi command or local debug command
      if (command.startsWith("PI:")) {
        // Handle Pi command
        PiResponse response = piComm.processCommand(command);
        piComm.sendResponse(response);
      } else {
        // Handle local debug command
        handleLocalCommand(command);
      }
    }
  }
  
  // Check limit switch
  checkLimitSwitch();
  
  delay(10);
}

void handleLocalCommand(String cmd) {
  cmd.toLowerCase();
  
  if (cmd == "status") {
    printSystemStatus();
  }
  else if (cmd == "sensors") {
    sensorLineFollower.printSensorValues();  // Now called on LineFollower
  }
  else if (cmd == "arm") {
    arm.printStatus();
  }
  else if (cmd == "lf") {
    sensorLineFollower.printStatus();
  }
  else if (cmd == "test") {
    // Test Pi communication locally
    Serial.println("Testing Pi command: PI:STATUS");
    PiResponse response = piComm.processCommand("PI:STATUS");
    piComm.sendResponse(response);
  }
  else {
    Serial.println("Local commands: status, sensors, arm, lf, test");
    Serial.println("Pi commands start with 'PI:'");
  }
}

void checkLimitSwitch() {
  bool currentState = !digitalRead(LIMIT_SWITCH_PIN); // Inverted because of pullup
  unsigned long currentTime = millis();
  
  // Debounce the switch
  if (currentState != lastLimitSwitchState && 
      (currentTime - lastLimitSwitchTime) > DEBOUNCE_DELAY) {
    
    lastLimitSwitchState = currentState;
    lastLimitSwitchTime = currentTime;
    
    // Send notification on rising edge (switch pressed)
    if (currentState) {
      piComm.sendLimitSwitchPressed();
      
      // Optional: Emergency stop on limit switch
      // emergencyStop();
    }
  }
}

void printSystemStatus() {
  Serial.println("\n=== SYSTEM STATUS ===");
  Serial.print("System Initialized: "); Serial.println(systemInitialized ? "Yes" : "No");
  Serial.print("Limit Switch: "); Serial.println(lastLimitSwitchState ? "PRESSED" : "Released");
  
  Serial.println("\n--- Motor Controller ---");
  motors.printStatus();
  
  Serial.println("\n--- Line Follower ---");
  sensorLineFollower.printStatus();
  
  Serial.println("\n--- Arm Controller ---");
  arm.printStatus();
  
  Serial.println("=====================\n");
}

void emergencyStop() {
  Serial.println("EMERGENCY STOP ACTIVATED!");
  
  // Stop all movement
  motors.stop();
  sensorLineFollower.stop();
  arm.stopAll();
  
  // Send emergency notification to Pi
  Serial.println("ESP:EMERGENCY_STOP");
}