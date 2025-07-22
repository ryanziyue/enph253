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
  
  Serial.println("ESP32 Robotic System with Serial Communication");
  Serial.println("==============================================");
  
  // initialize limit switch pin
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  // initialize controllers
  motors.init();
  arm.init();
  
  // configure sensor line follower default settings
  // PID values are already set in linefollower.h (Kp=1.0, Ki=0.0, Kd=0.0)
  sensorLineFollower.setBaseSpeed(150, 100);
  
  // configure sensor thresholds
  motors.setSensorThreshold(MotorController::R1, 0.3);
  motors.setSensorThreshold(MotorController::L1, 0.3);
  motors.setSensorThreshold(MotorController::R2, 0.3);
  motors.setSensorThreshold(MotorController::L2, 0.3);
  
  delay(1000);
  arm.resetPosition();
  
  systemInitialized = true;
  
  Serial.println("System ready! Listening for commands...");
  Serial.println("");
  Serial.println("Available Pi Commands:");
  Serial.println("MOTOR CONTROL:");
  Serial.println("  PI:MC,x,y      - Set motor speeds (left, right)");
  Serial.println("  PI:LF,x        - Toggle line following (1=on, 0=off)");
  Serial.println("  PI:LFS,x       - Set line following speed");
  Serial.println("");
  Serial.println("ARM CONTROL:");
  Serial.println("  PI:SP,a,b,c    - Set servo positions (base,shoulder,elbow)");
  Serial.println("  PI:WP,a        - Set wrist position");
  Serial.println("  PI:CP,a        - Set claw position");
  Serial.println("  PI:GP,x,y      - Set global position (IK)");
  Serial.println("  PI:GV,x,y      - Set global velocity (IK)");
  Serial.println("");
  Serial.println("Local Debug Commands:");
  Serial.println("  status         - Show system status");
  Serial.println("  sensors        - Show sensor readings");
  Serial.println("  arm            - Show arm status");
  Serial.println("  lf             - Show line follower status");
  Serial.println("  test           - Test PI:STATUS command");
  Serial.println("==============================================");
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
    motors.printSensorValues();
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