#include <Arduino.h>
#include "main.h"
#include "arm.h"

// create servo controller instance
ServoController arm;

void handleSerialCommand(String);

int numTestPositions = 3;
int currentTestPosition = 0;
bool autoMode = false;

// =====================
// Original setup() commented out for IK velocity test
/*
void setup() {
  // initialize serial communication
  Serial.begin(115200);
  Serial.println("ESP32 Robotic Arm - IK Position Control");
  Serial.println("========================================");
  
  // initialize servo controller
  arm.init();
  
  Serial.println("Setup complete! Ready for commands.");
  Serial.println("");
  Serial.println("Available Commands:");
  Serial.println("  'x,y' - Move to position (e.g., '20,15' or '15.5,25.3')");
  Serial.println("  'p' - Print current position");
  Serial.println("  'h' - Move to home position (90° all joints)");
  Serial.println("  's' - Print servo status");
  Serial.println("  'c' - Test claw (open/close)");
  Serial.println("  'auto' - Toggle automatic cycling between test positions");
  Serial.println("  'stop' - Stop all movement");
  Serial.println("");
  Serial.println("Example: Type '20,15' to move wrist to (20cm, 15cm)");
  Serial.println("Type 'p' to see current position");
  Serial.println("");
  
  // start at home position
  delay(1000);
  arm.resetPosition();
  Serial.println("Ready for commands!");
}
*/

// IK velocity test setup
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Robotic Arm - IK Velocity Test Mode");
  Serial.println("=========================================");
  
  arm.init();
  delay(1000);
  arm.resetPosition();
  
  Serial.println("Ready! Available commands:");
  Serial.println("  h# - Move horizontally RIGHT for # seconds (e.g., h4)");
  Serial.println("  h-# - Move horizontally LEFT for # seconds (e.g., h-4)");
  Serial.println("  v# - Move vertically UP for # seconds (e.g., v3)");
  Serial.println("  v-# - Move vertically DOWN for # seconds (e.g., v-3)");
  Serial.println("  p - Print current position");
  Serial.println("  zero - Zero all servos to 0 degrees");
  Serial.println("  stop - Stop all movement");
  Serial.println("");
}

// =====================
// Original loop() commented out for IK velocity test
/*
void loop() {
  // always update the arm controller
  arm.update();
  
  // check for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    handleSerialCommand(input);
  }
  
  // automatic cycling if enabled
  if (autoMode) {
    static unsigned long lastMove = 0;
    if (millis() - lastMove > 4000) {  // move every 4 seconds
      float x = testPositions[currentTestPosition][0];
      float y = testPositions[currentTestPosition][1];
      
      Serial.print("Auto mode - moving to position ");
      Serial.print(currentTestPosition + 1);
      Serial.print(": (");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.println(")");
      
      arm.setGlobalPosition(x, y);
      
      currentTestPosition = (currentTestPosition + 1) % numTestPositions;
      lastMove = millis();
    }
  }
  
  delay(10);  // small delay for stability
}
*/

// IK velocity test loop
void loop() {
  arm.update();
  
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() < 2) {
      Serial.println("Invalid command. Use h#, h-#, v#, or v-# (e.g., h4, h-2, v3, v-1)");
      return;
    }
    
    char dir = tolower(input.charAt(0));
    String durationStr = input.substring(1);
    int duration = durationStr.toInt();
    bool isNegative = durationStr.startsWith("-");
    
    if (isNegative) {
      duration = abs(duration); // make duration positive for timing
    }
    
    if (input == "p") {
      // print current position
      Point pos = arm.getCurrentPosition();
      Serial.print("Current wrist position: (");
      Serial.print(pos.x, 2);
      Serial.print(", ");
      Serial.print(pos.y, 2);
      Serial.println(") cm");
      
    } else if (input == "stop") {
      // stop all movement
      arm.setGlobalVelocity(0, 0);
      Serial.println("All movement stopped");
      
    } else if (input == "zero") {
      // zero all servos
      arm.zeroAllServos();
      
    } else if (duration > 0 && (dir == 'h' || dir == 'v')) {
      float vx = 0, vy = 0;
      
      if (dir == 'h') {
        vx = isNegative ? -2.0 : 2.0; // cm/s, left if negative, right if positive
        vy = 0;
        Serial.print("Moving horizontally ");
        Serial.print(isNegative ? "LEFT" : "RIGHT");
        Serial.print(" for ");
      } else if (dir == 'v') {
        vx = 0;
        vy = isNegative ? -2.0 : 2.0; // cm/s, down if negative, up if positive
        Serial.print("Moving vertically ");
        Serial.print(isNegative ? "DOWN" : "UP");
        Serial.print(" for ");
      }
      
      Serial.print(duration);
      Serial.println(" seconds");
      
      // start velocity movement
      arm.setGlobalVelocity(vx, vy);
      
      // move for specified duration
      unsigned long startTime = millis();
      while (millis() - startTime < duration * 1000) {
        arm.update();
        delay(10);
      }
      
      // stop movement
      arm.setGlobalVelocity(0, 0);
      Serial.println("Movement completed. Velocity stopped.");
      
      // show final position
      Point pos = arm.getCurrentPosition();
      Serial.print("Final position: (");
      Serial.print(pos.x, 2);
      Serial.print(", ");
      Serial.print(pos.y, 2);
      Serial.println(") cm");
      
    } else {
      Serial.println("Unknown command! Use:");
      Serial.println("  h# - horizontal RIGHT movement (e.g., h4)");
      Serial.println("  h-# - horizontal LEFT movement (e.g., h-4)");
      Serial.println("  v# - vertical UP movement (e.g., v3)");
      Serial.println("  v-# - vertical DOWN movement (e.g., v-3)");
      Serial.println("  p - print position");
      Serial.println("  zero - zero all servos");
      Serial.println("  stop - stop movement");
    }
  }
  
  delay(10);
}

// =====================
// Original handleSerialCommand() commented out for IK velocity test
/*
void handleSerialCommand(String cmd) {
  // ...existing code...
}
*/