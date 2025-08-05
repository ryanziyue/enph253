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
  STATE_ERROR
};

enum PetMode {
  MODE_5_PETS = 0,    // SW1=LOW,  SW2=LOW  (binary: 00)
  MODE_6_PETS = 1,    // SW1=HIGH, SW2=LOW  (binary: 01)
  MODE_7_PETS = 2,    // SW1=LOW,  SW2=HIGH (binary: 10)
  MODE_8_PETS = 3     // SW1=HIGH, SW2=HIGH (binary: 11)
};

// Display object
extern Adafruit_SSD1306 display_handler;

// “ready” flag
extern bool ready;

// initialize everything (call once from setup)
void initDisplayInputs();

// call in loop()
void pollOtherInputs();

// set “ready” true/false
void setReady(bool state);

void updateLCD();

// register a callback: void callback(bool newState)
void onSwitch1Change(void (*cb)(bool));
void onSwitch2Change(void (*cb)(bool));
void onButton1Change(void (*cb)(bool));
void onButton2Change(void (*cb)(bool));

#endif // INPUT_DISPLAY_H
