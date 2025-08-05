#include "input_display.h"

// —————— globals ——————
const unsigned long DEBOUNCE_DELAY = 50;

Adafruit_SSD1306 display_handler(SCREEN_WIDTH,
                                 SCREEN_HEIGHT,
                                 &Wire,
                                 OLED_RESET);

// last‐known switch states
static bool lastSwitch1State = HIGH;
static bool lastSwitch2State = HIGH;

// button1 raw+debounce state (already in your file)
bool   button1_last_state          = HIGH;
bool   button1_current_state       = HIGH;
unsigned long button1_last_debounce_time = 0;

// button2 raw+debounce state
bool   button2_last_state          = HIGH;
bool   button2_current_state       = HIGH;
unsigned long button2_last_debounce_time = 0;

// for enforcing the extra 10 ms quiet‐time on buttons
static bool      lastButton1Press      = false;
static unsigned long lastButton1EventTime = 0;
static bool      lastButton2Press      = false;
static unsigned long lastButton2EventTime = 0;

bool ready = false;
bool main_running = false;

// —————— implementation ——————

void updateLCD() {
    String message = "Team 12 Menu\n";

    if (main_running) {
        message += "Main Running:\n";
        if (lastSwitch1State == HIGH && lastSwitch2State == HIGH) {
            message += "7 Pets Selected";
        }
        else if (lastSwitch1State == HIGH && lastSwitch2State == HIGH) {
            message += "6 Pets Selected";
        }
        else {
            message += "5 Pets Selected";
        }
    } else if (ready) {
        message += "Current Selected Configuration:\n";
        message += lastSwitch1State == HIGH ? "Chute Pet Selected\n" : "Chute Pet Not Selected\n";
        message += "\n";
        message += lastSwitch2State == HIGH ? "Debris Pet Selected" : "Debris Pet Not Selected";
    }
    else {
        message += "Initalizing...\nPress the reset button to refresh states";
    }

    display_handler.clearDisplay();
    display_handler.setCursor(0, 0);
    display_handler.println(message);
    display_handler.display();
}

void initDisplayInputs() {
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);

  // OLED startup (unchanged)
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.display();
  delay(2000);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0, 0);
  display_handler.println("Initializing...");
  display_handler.display();

  lastSwitch1State = digitalRead(SWITCH_1_PIN);
  lastSwitch2State = digitalRead(SWITCH_2_PIN);

  updateLCD();
}

void setReady(bool state) {
    ready = state;
    updateLCD();
}

void startMain() {
    main_running = true;
    String message = "ESP:START,";
    message += lastSwitch1State == HIGH ? "1," : "0,";
    message += lastSwitch2State == HIGH ? "1" : "0";
    Serial.println(message);
    updateLCD();
}

void resetMain() {
    main_running = false;
    ready = false;
    String message = "ESP:RESET";
    Serial.println(message);
    updateLCD();
}

bool debounceButtonState(int pin,
                         bool &lastState,
                         bool &currentState,
                         unsigned long &lastDebounceTime) {
  bool reading = digitalRead(pin);

  if (reading != lastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    currentState = reading;
  }

  lastState = reading;
  // active‐low button → pressed when LOW
  return (currentState == LOW);
}

void pollOtherInputs() {
  // ——— Switch 1 - Toggle Chute Pet ———
  bool currS1 = digitalRead(SWITCH_1_PIN);
  if (currS1 != lastSwitch1State) {
    lastSwitch1State = currS1;
    updateLCD();
    // Serial.print("Switch 1: ");
    // Serial.println(currS1 == LOW ? "ON" : "OFF");
  }

  // ——— Switch 2 - Toggle Debris Pet ———
  bool currS2 = digitalRead(SWITCH_2_PIN);
  if (currS2 != lastSwitch2State) {
    lastSwitch2State = currS2;
    updateLCD();
    // Serial.print("Switch 2: ");
    // Serial.println(currS2 == LOW ? "ON" : "OFF");
  }

  // current time for button‐quiet‐time
  unsigned long now = millis();

  // ——— Button 1 (debounced + 10 ms) - Start ———
  bool pressed1 = debounceButtonState(
    BUTTON_1_PIN,
    button1_last_state,
    button1_current_state,
    button1_last_debounce_time
  );
  if (pressed1 != lastButton1Press &&
      (now - lastButton1EventTime) >= 10) {
    lastButton1EventTime = now;
    lastButton1Press = pressed1;
    updateLCD();
  }

  // ——— Button 2 (debounced + 10 ms) - Reset———
  bool pressed2 = debounceButtonState(
    BUTTON_2_PIN,
    button2_last_state,
    button2_current_state,
    button2_last_debounce_time
  );
  if (pressed2 != lastButton2Press &&
      (now - lastButton2EventTime) >= 10) {
    lastButton2EventTime = now;
    lastButton2Press = pressed2;
    updateLCD();
  }
  
}
