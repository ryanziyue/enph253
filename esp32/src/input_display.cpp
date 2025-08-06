#include "input_display.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================
InputDisplay::InputDisplay() : 
  display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
  currentState(STATE_INIT),
  currentMode(MODE_ONE),
  systemReady(false),
  missionActive(false),
  initialized(false),
  lastError(""),
  hasError(false),
  onStartCallback(nullptr),
  onResetCallback(nullptr),
  onModeChangeCallback(nullptr) {
  
  // Initialize input structure
  inputs.switch1_current = false;
  inputs.switch1_last = false;
  inputs.switch2_current = false;
  inputs.switch2_last = false;
  
  inputs.start_last_state = HIGH;
  inputs.start_current_state = HIGH;
  inputs.start_last_debounce_time = 0;
  
  inputs.reset_last_state = HIGH;
  inputs.reset_current_state = HIGH;
  inputs.reset_last_debounce_time = 0;
  
  inputs.reset_hold_active = false;
  inputs.reset_progress_percent = 0;
  inputs.reset_needs_release = false;
  inputs.last_display_update = 0;
  inputs.display_needs_update = true;
}

// ============================================================================
// INITIALIZATION
// ============================================================================
bool InputDisplay::init() {
  Serial.println("=== Initializing InputDisplay System ===");
  
  // Configure GPIO pins
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);  
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RESET_PIN, INPUT_PULLUP);
  
  Serial.println("✓ GPIO pins configured");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCK_PIN);
  Serial.print("✓ I2C initialized: SDA="); Serial.print(SDA_PIN);
  Serial.print(" SCK="); Serial.println(SCK_PIN);
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    handleDisplayError("OLED initialization failed");
    return false;
  }
  
  // Configure display settings
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setRotation(0);
  
  display.setCursor(0, 5);
  display.setTextSize(2);
  display.println("Robot");
  display.println("Starting");
  display.setTextSize(1);
  display.println();
  display.println("Please wait...");
  display.display();
  
  Serial.println("✓ OLED display initialized");
  
  // Read initial pin states
  inputs.switch1_current = digitalRead(SWITCH_1_PIN);
  inputs.switch1_last = inputs.switch1_current;
  inputs.switch2_current = digitalRead(SWITCH_2_PIN);
  inputs.switch2_last = inputs.switch2_current;
  
  inputs.start_last_state = digitalRead(BUTTON_START_PIN);
  inputs.start_current_state = inputs.start_last_state;
  inputs.start_last_debounce_time = millis();
  
  inputs.reset_last_state = digitalRead(BUTTON_RESET_PIN);
  inputs.reset_current_state = inputs.reset_last_state;
  inputs.reset_last_debounce_time = millis();
  
  // Set initial mode
  currentMode = getCurrentMode();
  
  delay(2000);  // Show startup screen
  
  // Mark as initialized and ready
  initialized = true;
  currentState = STATE_READY;
  systemReady = true;
  inputs.display_needs_update = true;
  
  Serial.print("✓ Initial mode: "); Serial.print(getPetCount()); Serial.println(" pets");
  Serial.println("=== InputDisplay System Ready ===");
  
  return true;
}

// ============================================================================
// MAIN UPDATE LOOP
// ============================================================================
void InputDisplay::update() {
  if (!initialized) return;
  
  unsigned long now = millis();
  
  updateSwitches();
  updateButtons();
  
  // Update display if needed
  if (inputs.display_needs_update && (now - inputs.last_display_update) > DISPLAY_UPDATE_MS) {
    updateDisplayContent();
    inputs.last_display_update = now;
    inputs.display_needs_update = false;
  }
}

// ============================================================================
// INPUT HANDLING
// ============================================================================
void InputDisplay::updateSwitches() {
  // LOCK: Completely ignore switch changes during mission execution
  if (currentState == STATE_RUNNING) {
    // Still read the switches to keep internal state updated,
    // but don't process any changes or call callbacks
    inputs.switch1_current = digitalRead(SWITCH_1_PIN);
    inputs.switch2_current = digitalRead(SWITCH_2_PIN);
    
    // Update last states to prevent false triggers when mission ends
    inputs.switch1_last = inputs.switch1_current;
    inputs.switch2_last = inputs.switch2_current;
    
    // Exit early - no mode processing during mission
    return;
  }
  
  // Normal switch processing (only when NOT running)
  inputs.switch1_current = digitalRead(SWITCH_1_PIN);
  inputs.switch2_current = digitalRead(SWITCH_2_PIN);
  
  // Check for mode changes
  if (inputs.switch1_current != inputs.switch1_last || 
      inputs.switch2_current != inputs.switch2_last) {
    
    inputs.switch1_last = inputs.switch1_current;
    inputs.switch2_last = inputs.switch2_current;
    
    PetMode newMode = getCurrentMode();
    if (newMode != currentMode) {
      currentMode = newMode;
      inputs.display_needs_update = true;
      
      Serial.print("Mode switched to: "); 
      Serial.print(getPetCount()); 
      Serial.println(" pets");
      
      // Call mode change callback if registered
      if (onModeChangeCallback) {
        onModeChangeCallback(currentMode, getPetCount());
      }
    }
  }
}

void InputDisplay::updateButtons() {
  unsigned long now = millis();
  
  // Handle START button - EVENT-based (triggers once per press)
  if (debounceButtonPress(BUTTON_START_PIN, inputs.start_last_state, inputs.start_current_state, inputs.start_last_debounce_time)) {
    handleStartPress();
  }
  
  // Handle RESET button - STATE-based (detects hold duration)
  bool resetCurrentlyPressed = debounceButtonState(BUTTON_RESET_PIN, inputs.reset_last_state, inputs.reset_current_state, inputs.reset_last_debounce_time);
  
  // Check if button was released (clear the needs_release flag)
  if (!resetCurrentlyPressed && inputs.reset_needs_release) {
    inputs.reset_needs_release = false;
    Serial.println("Reset button released - ready for new reset");
  }
  
  // Reset button just pressed (but only if we don't need a release first)
  if (resetCurrentlyPressed && !inputs.reset_hold_active && !inputs.reset_needs_release) {
    inputs.reset_press_start = now;
    inputs.reset_hold_active = true;
    inputs.reset_progress_percent = 0;
    inputs.display_needs_update = true;
    Serial.println("Reset button pressed - hold for 3 seconds");
  }
  
  // Reset button just released (cancel ongoing reset)
  if (!resetCurrentlyPressed && inputs.reset_hold_active) {
    inputs.reset_hold_active = false;
    inputs.reset_progress_percent = 0;
    inputs.display_needs_update = true;
    Serial.println("Reset cancelled");
  }
  
  // Handle reset hold progress
  if (inputs.reset_hold_active && resetCurrentlyPressed) {
    unsigned long holdTime = now - inputs.reset_press_start;
    int newProgress = map(holdTime, 0, RESET_HOLD_TIME, 0, 100);
    newProgress = constrain(newProgress, 0, 100);
    
    if (newProgress != inputs.reset_progress_percent) {
      inputs.reset_progress_percent = newProgress;
      inputs.display_needs_update = true;
    }
    
    // Reset hold complete
    if (holdTime >= RESET_HOLD_TIME) {
      inputs.reset_hold_active = false;
      inputs.reset_progress_percent = 0;
      inputs.reset_needs_release = true;  // ← REQUIRE RELEASE BEFORE NEW RESET
      handleResetComplete();
    }
  }
}

// ============================================================================
// BUTTON DEBOUNCING (Your preferred method)
// ============================================================================
bool InputDisplay::debounceButtonPress(int pin, bool &lastState, bool &currentState, unsigned long &lastDebounceTime) {
  bool reading = digitalRead(pin);
  
  if (reading != lastState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != currentState) {
      currentState = reading;
      
      // Return true only on button PRESS (HIGH to LOW transition)
      if (currentState == LOW) {
        lastState = reading;
        return true;
      }
    }
  }
  
  lastState = reading;
  return false;
}

bool InputDisplay::debounceButtonState(int pin, bool &lastState, bool &currentState, unsigned long &lastDebounceTime) {
  bool reading = digitalRead(pin);
  
  if (reading != lastState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    currentState = reading;
  }
  
  lastState = reading;
  return (currentState == LOW);
}

// ============================================================================
// BUTTON EVENT HANDLERS
// ============================================================================
void InputDisplay::handleStartPress() {
  Serial.println("START button pressed");
  
  switch (currentState) {
    case STATE_READY:
      // Show starting message
      display.clearDisplay();
      display.setCursor(0, 15);
      display.setTextSize(2);
      display.println("STARTING");
      display.setTextSize(1);
      display.println();
      display.print(getPetCount());
      display.println(" pets");
      display.display();
      delay(1200);
      
      Serial.print("Starting mission: "); Serial.print(getPetCount()); Serial.println(" pets");
      setSystemState(STATE_RUNNING);
      sendStartCommand();
      
      if (onStartCallback) {
        onStartCallback();
      }
      break;
      
    case STATE_RUNNING:
      // Show already running message
      display.clearDisplay();
      display.setCursor(0, 20);
      display.setTextSize(1);
      display.println("ALREADY RUNNING!");
      display.println("Mission in progress");
      display.display();
      delay(1000);
      inputs.display_needs_update = true;
      Serial.println("Mission already running");
      break;
      
    case STATE_INIT:
      Serial.println("System not ready");
      break;
      
    case STATE_ERROR:
      Serial.println("System error - cannot start");
      break;
  }
}

void InputDisplay::handleResetComplete() {
  Serial.println("Reset complete");
  
  // Show reset complete message
  display.clearDisplay();
  display.setCursor(0, 20);
  display.setTextSize(2);
  display.println("RESET");
  display.println("COMPLETE");
  display.display();
  delay(200);
  
  switch (currentState) {
    case STATE_RUNNING:
    case STATE_ERROR:
      clearErrors();
      setSystemState(STATE_READY);
      break;
      
    default:
      // Already in ready or init state
      break;
  }
  
  inputs.display_needs_update = true;
}

// ============================================================================
// DISPLAY MANAGEMENT
// ============================================================================
void InputDisplay::updateDisplayContent() {
  if (currentState == STATE_ERROR) {
    displayErrorScreen();
    return;
  }
  
  // If reset is active, show ONLY the full-screen reset bar
  if (inputs.reset_hold_active) {
    drawFullScreenResetBar();
    return;
  }
  
  display.clearDisplay();
  display.setCursor(0, 5);
  
  // Mission info - same text size for "Mission:" and pet count
  display.setTextSize(2);
  display.print(getPetCount());
  display.println(" pets");
  
  // System status and instructions
  switch (currentState) {
    case STATE_INIT:
      display.println("initializing...");
      break;
      
    case STATE_READY:
      display.println("ready to");
      display.println("start");
      break;
      
    case STATE_RUNNING:
      display.println("mission");
      display.println("running");
      break;
  }
  
  display.display();
}

void InputDisplay::drawFullScreenResetBar() {
  display.clearDisplay();
  
  // Title
  display.setCursor(0, 5);
  display.setTextSize(2);
  display.print("resetting");
  
  display.setTextSize(1);
  display.println();
  
  // Large progress bar
  int barY = 25;
  int barHeight = 20;
  int barWidth = SCREEN_WIDTH - 8;
  int fillWidth = map(inputs.reset_progress_percent, 0, 100, 0, barWidth);
  
  // Draw border
  display.drawRect(4, barY, barWidth, barHeight, SSD1306_WHITE);
  display.drawRect(3, barY-1, barWidth+2, barHeight+2, SSD1306_WHITE);
  
  // Draw fill
  if (fillWidth > 2) {
    display.fillRect(5, barY+1, fillWidth-2, barHeight-2, SSD1306_WHITE);
  }
  
  // Percentage
  display.setCursor(0, barY + barHeight + 5);
  display.print("Progress: ");
  display.print(inputs.reset_progress_percent);
  display.print("%");
  
  display.display();
}

void InputDisplay::displayErrorScreen() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println("ERROR");
  
  display.setTextSize(1);
  display.println();
  display.println("System error");
  display.println();
  display.println(lastError);
  display.println();
  display.println("Check connections");
  
  display.display();
}

// ============================================================================
// STATE MANAGEMENT
// ============================================================================
void InputDisplay::setSystemState(SystemState state) {
  if (currentState != state) {
    SystemState oldState = currentState;
    currentState = state;
    inputs.display_needs_update = true;
    
    Serial.print("State: ");
    Serial.print(getStateString(oldState));
    Serial.print(" → ");
    Serial.println(getStateString(state));
    
    switch (state) {
      case STATE_RUNNING:
        missionActive = true;
        break;
      case STATE_READY:
        missionActive = false;
        break;
      case STATE_ERROR:
        missionActive = false;
        break;
      case STATE_INIT:
        missionActive = false;
        break;
    }
  }
}

void InputDisplay::setReady(bool ready) {
  systemReady = ready;
  if (ready && currentState == STATE_INIT) {
    setSystemState(STATE_READY);
  }
  inputs.display_needs_update = true;
}

void InputDisplay::setMissionActive(bool active) {
  missionActive = active;
  if (active) {
    setSystemState(STATE_RUNNING);
  } else if (currentState == STATE_RUNNING) {
    setSystemState(STATE_READY);
  }
}

// ============================================================================
// MODE AND CONFIGURATION
// ============================================================================
PetMode InputDisplay::getCurrentMode() const {
  // Read switch states (HIGH = not pressed with pullup)
  bool sw1 = (!digitalRead(SWITCH_1_PIN) == HIGH);
  bool sw2 = (!digitalRead(SWITCH_2_PIN) == HIGH);

  int mode = 0;

  if (sw1) {
    mode += 1;
  }

  if (sw2) {
    mode += 2;
  }

  return (PetMode)constrain(mode, 0, 3);
}

int InputDisplay::getPetCount() const {
  switch (currentMode) {
    case MODE_ONE: return 2;
    case MODE_TWO: return 5;
    case MODE_THREE: return 6;
    case MODE_FOUR: return 7;  // As per your original code
    default: return 7;
  }
}

// ============================================================================
// CALLBACK REGISTRATION
// ============================================================================
void InputDisplay::setStartCallback(StartCallback callback) {
  onStartCallback = callback;
}

void InputDisplay::setResetCallback(ResetCallback callback) {
  onResetCallback = callback;
}

void InputDisplay::setModeChangeCallback(ModeChangeCallback callback) {
  onModeChangeCallback = callback;
}

// ============================================================================
// COMMUNICATION HELPERS
// ============================================================================
void InputDisplay::sendStartCommand() {
  String command = "ESP:START," + String(getPetCount());
  Serial.println(command);
  
  String params = "ESP:MODE," +
                  String(inputs.switch2_current ? 1 : 0) + "," +
                  String(inputs.switch1_current ? 1 : 0);
  Serial.println(params);
}

void InputDisplay::sendResetCommand() {
  Serial.println("ESP:RESET");
  Serial.println("ESP:ABORT");
}

void InputDisplay::sendStatusUpdate() {
  String status = "ESP:STATUS,";
  status += getStateString(currentState) + ",";
  status += String(getPetCount()) + ",";
  status += (missionActive ? "ACTIVE" : "IDLE");
  Serial.println(status);
}

// ============================================================================
// UTILITY METHODS
// ============================================================================
String InputDisplay::getStateString(SystemState state) const {
  switch (state) {
    case STATE_INIT: return "INIT";
    case STATE_READY: return "READY";
    case STATE_RUNNING: return "RUNNING";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void InputDisplay::forceDisplayUpdate() {
  inputs.display_needs_update = true;
  inputs.last_display_update = 0;
}

void InputDisplay::showMessage(const String& title, const String& message, int displayTime) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(title);
  display.setTextSize(1);
  display.println();
  display.println(message);
  display.display();
  
  if (displayTime > 0) {
    delay(displayTime);
    inputs.display_needs_update = true;
  }
}

void InputDisplay::printStatus() {
  Serial.println("=== INPUT DISPLAY STATUS ===");
  Serial.print("Initialized: "); Serial.println(initialized ? "YES" : "NO");
  Serial.print("State: "); Serial.println(getStateString(currentState));
  Serial.print("Mode: "); Serial.print(getPetCount()); Serial.println(" pets");
  Serial.println();
  
  Serial.println("Switches:");
  Serial.print("  SW1: "); Serial.println(inputs.switch1_current ? "HIGH" : "LOW");
  Serial.print("  SW2: "); Serial.println(inputs.switch2_current ? "HIGH" : "LOW");
  Serial.println();
  
  Serial.println("Buttons:");
  Serial.print("  START: "); Serial.println(getStartButtonState() ? "PRESSED" : "RELEASED");
  Serial.print("  RESET: "); Serial.println(getResetButtonState() ? "PRESSED" : "RELEASED");
  
  if (inputs.reset_hold_active) {
    Serial.print("  Reset Progress: "); Serial.print(inputs.reset_progress_percent); Serial.println("%");
  }
  
  Serial.print("System Ready: "); Serial.println(systemReady ? "YES" : "NO");
  Serial.print("Mission Active: "); Serial.println(missionActive ? "YES" : "NO");
  Serial.println("=============================");
}

bool InputDisplay::isSystemHealthy() const {
  return initialized && !hasError && systemReady && currentState != STATE_ERROR;
}

void InputDisplay::showError(const String& error) {
  handleDisplayError(error);
}

void InputDisplay::clearErrors() {
  hasError = false;
  lastError = "";
  if (currentState == STATE_ERROR) {
    setSystemState(STATE_READY);
  }
  Serial.println("Errors cleared");
}

void InputDisplay::handleDisplayError(const String& error) {
  lastError = error;
  hasError = true;
  currentState = STATE_ERROR;
  inputs.display_needs_update = true;
  
  Serial.print("DISPLAY ERROR: ");
  Serial.println(error);
}