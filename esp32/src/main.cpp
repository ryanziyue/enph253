#include <Arduino.h>
#include "main.h"
#include "arm.h"
#include "motor.h"
#include "linefollower.h"
#include "pi.h"
#include "input_display.h"

// Create controller instances
ServoController arm;
MotorController motors;
LineFollower sensorLineFollower(&motors);
PiComm piComm(&motors, &arm, &sensorLineFollower);
InputDisplay inputDisplay;

// System state
bool systemInitialized = false;

void setup() {
  Serial.begin(921600);
  
  motors.init();
  arm.init();
  
  motors.setMinSpeed(MOTOR_MIN_SPEED);
  motors.setMaxSpeed(MOTOR_MAX_SPEED);  
  
  sensorLineFollower.setBaseSpeed(BASE_SPEED);
  sensorLineFollower.setTargetPosition(TARGET_POSITION);
  sensorLineFollower.setPID(K_P, K_I, K_D);
  sensorLineFollower.setKo(K_O);
  sensorLineFollower.setSensorThreshold(LineFollower::R1, SENSOR_THRESHOLD_R1);
  sensorLineFollower.setSensorThreshold(LineFollower::L1, SENSOR_THRESHOLD_L1);
  sensorLineFollower.setSensorThreshold(LineFollower::R2, SENSOR_THRESHOLD_R2);
  sensorLineFollower.setSensorThreshold(LineFollower::L2, SENSOR_THRESHOLD_L2);

  if (!inputDisplay.init()) {
    return;
  }
  
  delay(1000);
  arm.resetPosition();
  
  systemInitialized = true;
  // inputDisplay.setReady(true); 
}

void loop() {
  if (!systemInitialized) return;
  
  arm.update();
  inputDisplay.update();
  
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      if (command.equals("PI:READY")) {
        inputDisplay.setReady(true);
        inputDisplay.setSystemState(STATE_READY);
        inputDisplay.forceDisplayUpdate();
        Serial.println("OKAY");
      } 
      else if (command.startsWith("PI:")) {
        PiResponse response = piComm.processCommand(command);
        piComm.sendResponse(response);
      } 
      else {
        handleLocalCommand(command);
      }
    }
  }
  
  delay(10);
}

void handleLocalCommand(String cmd) {
  cmd.toLowerCase();
  
  if (cmd == "status") {
    printSystemStatus();
  }
  else if (cmd == "sensors") {
    sensorLineFollower.printSensorValues();
  }
  else if (cmd == "arm") {
    arm.printStatus();
  }
  else if (cmd == "lf") {
    sensorLineFollower.printStatus();
  }
  else if (cmd == "test") {
    Serial.println("Testing Pi command: PI:STATUS");
    PiResponse response = piComm.processCommand("PI:STATUS");
    piComm.sendResponse(response);
  }
  
  else if (cmd == "input" || cmd == "display") {
    inputDisplay.printStatus();
  }
  else if (cmd == "mode") {
    Serial.print("Current mode: "); Serial.print(inputDisplay.getPetCount());
    Serial.print(" pets"); Serial.println(inputDisplay.getSystemState());
  }
  else if (cmd == "teststart") {
    Serial.println("Simulating start button press...");
    // Test ESP command through Pi system
    PiResponse response = piComm.processCommand("ESP:START," + String(inputDisplay.getPetCount()));
    piComm.sendResponse(response);
  }
  else if (cmd == "testreset") {
    Serial.println("Simulating reset command...");
    // Test ESP command through Pi system
    PiResponse response = piComm.processCommand("ESP:RESET");
    piComm.sendResponse(response);
  }
  else if (cmd == "health") {
    Serial.print("InputDisplay healthy: "); 
    Serial.println(inputDisplay.isSystemHealthy() ? "YES" : "NO");
  }
  
  else {
    Serial.println("=== Available Commands ===");
    Serial.println("System: status, sensors, arm, lf, test");
    Serial.println("Display: input, mode, teststart, testreset");
    Serial.println("Pi commands start with 'PI:'");
  }
}

void printSystemStatus() {
  Serial.println("\n=== COMPLETE SYSTEM STATUS ===");
  Serial.print("System Initialized: "); Serial.println(systemInitialized ? "Yes" : "No");
  
  Serial.println("\n--- Motor Controller ---");
  motors.printStatus();
  
  Serial.println("\n--- Line Follower ---");
  sensorLineFollower.printStatus();
  
  Serial.println("\n--- Arm Controller ---");
  arm.printStatus();
  
  Serial.println("===============================\n");
}

void emergencyStop() {
  Serial.println("EMERGENCY STOP ACTIVATED!");

  motors.stop();
  sensorLineFollower.stop();
  arm.stopAll();
  
  inputDisplay.showError("EMERGENCY STOP");
  inputDisplay.setSystemState(STATE_ERROR);

  Serial.println("ESP:EMERGENCY_STOP");
}