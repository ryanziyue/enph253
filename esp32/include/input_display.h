#ifndef INPUT_DISPLAY_H
#define INPUT_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// pinouts
#define SWITCH_1_PIN    7
#define SWITCH_2_PIN    8
#define BUTTON_1_PIN    25
#define BUTTON_2_PIN    26
#define SDA_PIN         9
#define SCK_PIN         10

// oled display
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDRESS    0x3C

// timing
#define DEBOUNCE_DELAY      50
#define RESET_HOLD_TIME     3000  
#define DISPLAY_UPDATE_MS   100   

// enums
enum SystemState {
  STATE_INIT,
  STATE_READY,
  STATE_RUNNING,
  STATE_PAUSED,
  STATE_ERROR,
};

enum PetMode {
  MODE_5_PETS = 0,
  MODE_6_PETS = 1,
  MODE_7_PETS = 2,
  MODE_8_PETS = 3,
};

// callbacks
typedef void (*StartCallback)();
typedef void (*ResetCallback)();
typedef void (*ModeChangeCallback)(PetMode newMode, int petCount);

class InputDisplay {
private:
  // display
  Adafruit_SSD1306 display;
  
  // system state
  SystemState currentState;
  PetMode currentMode;
  bool systemReady;
  bool missionActive;
  bool initialized;
  
  // input state tracking
  struct {
    // switch states
    bool switch1_current, switch1_last;
    bool switch2_current, switch2_last;
    
    // start button - event debouncing
    bool start_last_state, start_current_state;
    unsigned long start_last_debounce_time;
    
    // reset button - state debouncing  
    bool reset_last_state, reset_current_state;
    unsigned long reset_last_debounce_time;
    
    // reset hold timing
    unsigned long reset_press_start;
    bool reset_hold_active;
    int reset_progress_percent;
    
    // display timing
    unsigned long last_display_update;
    bool display_needs_update;
  } inputs;
  
  // error tracking
  String lastError;
  bool hasError;
  
  // callbacks
  StartCallback onStartCallback;
  ResetCallback onResetCallback;
  ModeChangeCallback onModeChangeCallback;
  
  // helper methods
  bool debounceButtonPress(int pin, bool &lastState, bool &currentState, unsigned long &lastDebounceTime);
  bool debounceButtonState(int pin, bool &lastState, bool &currentState, unsigned long &lastDebounceTime);
  void updateSwitches();
  void updateButtons();
  void handleStartPress();
  void handleResetComplete();
  void displayErrorScreen();
  void drawResetProgressBar();
  String getStateString(SystemState state);
  void handleDisplayError(const String& error);

public:
  InputDisplay();
  
  bool init();
  void update();                  
  void forceDisplayUpdate();
  
  // state management
  void setSystemState(SystemState state);
  void setReady(bool ready);
  void setMissionActive(bool active);
  
  // Mode and configuration
  PetMode getCurrentMode() const;
  int getPetCount() const;
  String getModeDescription() const;
  SystemState getSystemState() const { return currentState; }
  bool isSystemReady() const { return systemReady; }
  bool isMissionActive() const { return missionActive; }
  bool isInitialized() const { return initialized; }
  
  // Callback registration
  void setStartCallback(StartCallback callback);
  void setResetCallback(ResetCallback callback);  
  void setModeChangeCallback(ModeChangeCallback callback);
  
  void updateDisplayContent();
  void showMessage(const String& title, const String& message, int displayTime = 2000);
  void showError(const String& error);
  void clearErrors();
  
  // communication helpers
  void sendStartCommand();
  void sendResetCommand();
  void sendStatusUpdate();
  
  // debug
  void printStatus();
  void testDisplay();
  bool isSystemHealthy() const;
  
  // input reading
  bool getSwitch1State() const { return inputs.switch1_current; }
  bool getSwitch2State() const { return inputs.switch2_current; }
  bool getStartButtonState() const { return inputs.start_current_state == LOW; }
  bool getResetButtonState() const { return inputs.reset_current_state == LOW; }
  int getResetProgress() const { return inputs.reset_progress_percent; }
};

#endif
