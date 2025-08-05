#ifndef INPUT_DISPLAY_H
#define INPUT_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pin-outs
#define SWITCH_1_PIN   13
#define SWITCH_2_PIN   14
#define BUTTON_1_PIN    9
#define BUTTON_2_PIN   10

// OLED
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT    64
#define OLED_RESET      -1

// Debounce
extern const unsigned long DEBOUNCE_DELAY;

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
