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

// system state
bool systemInitialized = false;

void handleLocalCommand(String);

void setup() {
  Serial.begin(115200);
  
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
  
  // handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      if (command.startsWith("PI:")) {
        // handle pi command
        PiResponse response = piComm.processCommand(command);
        piComm.sendResponse(response);
      } else {
        // handle local command
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

void printSystemStatus() {
  Serial.println("\n=== SYSTEM STATUS ===");
  Serial.print("System Initialized: "); Serial.println(systemInitialized ? "Yes" : "No");
  
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

  motors.stop();
  sensorLineFollower.stop();
  arm.stopAll();

  Serial.println("ESP:EMERGENCY_STOP");
}