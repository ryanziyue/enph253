#ifndef INPUT_DISPLAY_H
#define INPUT_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Forward declaration
class PiComm;

// Pin assignments
#define SWITCH_1_PIN        8
#define SWITCH_2_PIN        7
#define BUTTON_START_PIN    25
#define BUTTON_RESET_PIN    26
#define SDA_PIN             9
#define SCK_PIN             10

// OLED display settings
#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       64
#define OLED_RESET          -1
#define OLED_ADDRESS        0x3C

// Timing constants
#define DEBOUNCE_DELAY      50
#define RESET_HOLD_TIME     1000  
#define DISPLAY_UPDATE_MS   200   

// System states
enum SystemState {
  STATE_INIT,
  STATE_READY,
  STATE_RUNNING,
  STATE_ERROR,
};

// Pet modes (based on switch combinations)
enum PetMode {
  MODE_ONE = 0,
  MODE_TWO = 1,
  MODE_THREE = 2,
  MODE_FOUR = 3,
};

class InputDisplay {
private:
  // Display object
  Adafruit_SSD1306 display;
  
  // System state
  SystemState currentState;
  PetMode currentMode;
  bool systemReady;
  bool missionActive;
  bool initialized;
  
  // Communication
  PiComm* piComm;

  // Input state tracking
  struct {
    // Switch states
    bool switch1_current, switch1_last;
    bool switch2_current, switch2_last;
    
    // START button - event-based debouncing
    bool start_last_state, start_current_state;
    unsigned long start_last_debounce_time;
    
    // RESET button - state-based debouncing  
    bool reset_last_state, reset_current_state;
    unsigned long reset_last_debounce_time;
    
    // Reset hold timing
    unsigned long reset_press_start;
    bool reset_hold_active;
    int reset_progress_percent;
    bool reset_needs_release;
    
    // Display timing
    unsigned long last_display_update;
    bool display_needs_update;
  } inputs;
  
  // Error tracking
  String lastError;
  bool hasError;
  
  // Private helper methods
  bool debounceButtonPress(int pin, bool &lastState, bool &currentState, unsigned long &lastDebounceTime);
  bool debounceButtonState(int pin, bool &lastState, bool &currentState, unsigned long &lastDebounceTime);
  void updateSwitches();
  void updateButtons();
  void handleStartPress();
  void handleResetComplete();
  void displayErrorScreen();
  void drawFullScreenResetBar();
  String getStateString(SystemState state) const;
  void handleDisplayError(const String& error);

public:
  // Constructor - now takes PiComm pointer
  InputDisplay(PiComm* piCommPtr);
  
  // Core functionality
  bool init();
  void update();                  
  void forceDisplayUpdate();
  
  // State management
  void setSystemState(SystemState state);
  void setReady(bool ready);
  void setMissionActive(bool active);
  
  // Mode and configuration
  PetMode getCurrentMode() const;
  int getPetCount() const;
  SystemState getSystemState() const { return currentState; }
  bool isSystemReady() const { return systemReady; }
  bool isMissionActive() const { return missionActive; }
  bool isInitialized() const { return initialized; }
  
  // Display management
  void updateDisplayContent();
  void showMessage(const String& title, const String& message, int displayTime = 2000);
  void showError(const String& error);
  void clearErrors();
  
  // Communication helpers - now use PiComm directly
  void sendStartCommand();
  void sendResetCommand();
  void sendStatusUpdate();
  void handleModeChange(PetMode newMode, int petCount);
  
  void printStatus();
  bool isSystemHealthy() const;

  // Direct input reading (for debugging)
  bool getSwitch1State() const { return inputs.switch1_current; }
  bool getSwitch2State() const { return inputs.switch2_current; }
  bool getStartButtonState() const { return inputs.start_current_state == LOW; }
  bool getResetButtonState() const { return inputs.reset_current_state == LOW; }
  int getResetProgress() const { return inputs.reset_progress_percent; }
};

#endif // INPUT_DISPLAY_H