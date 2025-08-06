#include <Arduino.h>
#include "main.h"
#include "arm.h"
#include "motor.h"
#include "linefollower.h"
#include "pi.h"
#include "input_display.h"  // NEW: Add InputDisplay

// Create controller instances
ServoController arm;
MotorController motors;
LineFollower sensorLineFollower(&motors);
PiComm piComm(&motors, &arm, &sensorLineFollower);
InputDisplay inputDisplay;  // NEW: Create InputDisplay instance

// System state
bool systemInitialized = false;

// ============================================================================
// CALLBACK FUNCTIONS (NEW) - Connect InputDisplay to your systems
// ============================================================================

// Called when start button is pressed
void onStartPressed() {
  Serial.println("🚀 Start button callback - preparing systems");
  
  // Add any custom start preparation here
  // For example:
  // - Reset sensor calibration
  // - Clear error states  
  // - Initialize mission variables
  
  // The InputDisplay already sent ESP:START command to Pi
  // Pi will handle the actual mission logic
}

// Called when reset is completed (3-second hold)
void onResetCompleted() {
  Serial.println("🛑 Reset callback - cleaning up systems");
  
  // Stop everything locally (Pi also stops via ESP:RESET command)
  if (sensorLineFollower.isRunning()) {
    sensorLineFollower.stop();
  }
  motors.stop();
  arm.stopAll();
  
  // Return to safe position
  delay(500);
  arm.resetPosition();
  
  Serial.println("✓ Local systems reset completed");
}

// Called when mode switches change
void onModeChanged(PetMode newMode, int petCount) {
  Serial.print("🔄 Mode changed to: ");
  Serial.print(petCount);
  Serial.print(" pets");
  
  // Adjust system parameters based on mode
  switch (newMode) {
    case MODE_ONE:
      // Basic mode - conservative settings
      sensorLineFollower.setBaseSpeed(150);
      arm.setMaxSpeed(IDX_SHOULDER_L, 25.0);
      break;
      
    case MODE_TWO:
      // Standard mode
      sensorLineFollower.setBaseSpeed(175);
      arm.setMaxSpeed(IDX_SHOULDER_L, 30.0);
      break;
      
    case MODE_THREE:
      // Advanced mode - faster
      sensorLineFollower.setBaseSpeed(200);
      arm.setMaxSpeed(IDX_SHOULDER_L, 35.0);
      break;
      
    case MODE_FOUR:
      // Expert mode - maximum speed
      sensorLineFollower.setBaseSpeed(225);
      arm.setMaxSpeed(IDX_SHOULDER_L, 40.0);
      break;
  }
}

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("🚀 Team 12 Robot Starting Up...");
  
  // Initialize core controllers (existing code)
  motors.init();
  arm.init();
  
  // Configure motor parameters (existing code)
  motors.setMinSpeed(MOTOR_MIN_SPEED);
  motors.setMaxSpeed(MOTOR_MAX_SPEED);  
  
  // Configure line follower (existing code)  
  sensorLineFollower.setBaseSpeed(BASE_SPEED);
  sensorLineFollower.setTargetPosition(TARGET_POSITION);
  sensorLineFollower.setPID(K_P, K_I, K_D);
  sensorLineFollower.setKo(K_O);
  sensorLineFollower.setSensorThreshold(LineFollower::R1, SENSOR_THRESHOLD_R1);
  sensorLineFollower.setSensorThreshold(LineFollower::L1, SENSOR_THRESHOLD_L1);
  sensorLineFollower.setSensorThreshold(LineFollower::R2, SENSOR_THRESHOLD_R2);
  sensorLineFollower.setSensorThreshold(LineFollower::L2, SENSOR_THRESHOLD_L2);

  // NEW: Initialize InputDisplay system
  if (!inputDisplay.init()) {
    Serial.println("❌ CRITICAL: InputDisplay initialization failed!");
    Serial.println("Check OLED and button connections");
    // Could add emergency fallback mode here
    return;
  }
  
  // NEW: Register callback functions to connect InputDisplay with your systems
  inputDisplay.setStartCallback(onStartPressed);
  inputDisplay.setResetCallback(onResetCompleted);
  inputDisplay.setModeChangeCallback(onModeChanged);
  
  delay(1000);
  arm.resetPosition();
  
  systemInitialized = true;
  inputDisplay.setReady(true);  // NEW: Signal system ready to InputDisplay
  
  Serial.println("✅ All systems initialized successfully!");
  Serial.println("Ready for mission - use switches to select mode, press START to begin");
}

// ============================================================================
// MAIN LOOP (Updated to use InputDisplay)
// ============================================================================
void loop() {
  if (!systemInitialized) return;
  
  // Core system updates
  arm.update();                // Existing: Update arm controller
  inputDisplay.update();       // NEW: Update input/display system
  
  // Handle serial commands (existing code - works with ESP commands now!)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      if (command.equals("PI:READY")) {
        inputDisplay.setReady(true);  // NEW: Update InputDisplay
      } 
      else if (command.startsWith("PI:") || command.startsWith("ESP:")) {
        // Existing Pi command handling - now also handles ESP commands!
        PiResponse response = piComm.processCommand(command);
        piComm.sendResponse(response);
      } 
      else {
        // Local command handling
        handleLocalCommand(command);
      }
    }
  }
  
  delay(10);
}

// ============================================================================
// UPDATED LOCAL COMMANDS (Enhanced for InputDisplay)
// ============================================================================
void handleLocalCommand(String cmd) {
  cmd.toLowerCase();
  
  // Existing commands (unchanged)
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
  
  // NEW: InputDisplay commands
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

// ============================================================================
// UPDATED SYSTEM STATUS (Enhanced)
// ============================================================================
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

// ============================================================================
// UPDATED EMERGENCY STOP (Enhanced)
// ============================================================================
void emergencyStop() {
  Serial.println("🚨 EMERGENCY STOP ACTIVATED! 🚨");

  // Stop all motion
  motors.stop();
  sensorLineFollower.stop();
  arm.stopAll();
  
  // NEW: Update InputDisplay to show error
  inputDisplay.showError("EMERGENCY STOP");
  inputDisplay.setSystemState(STATE_ERROR);

  // Send emergency command
  Serial.println("ESP:EMERGENCY_STOP");
  
  Serial.println("All systems halted - reset required");
}