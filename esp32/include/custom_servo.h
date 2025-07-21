#pragma once
#include <Arduino.h>

class CustomServo {
private:
  int pin;
  int channel;
  int current_angle;
  bool attached;
  
  // servo timing constants
  static const int SERVO_FREQ = 50;        // 50Hz for servos
  static const int SERVO_RESOLUTION = 16;  // 16-bit resolution
  static const int MIN_PULSE_WIDTH = 500;  // 0.5ms (0 degrees)
  static const int MAX_PULSE_WIDTH = 2500; // 2.5ms (180 degrees)
  static const int PERIOD_US = 20000;      // 20ms period
  
  // convert angle to duty cycle
  int angleToDuty(int angle);
  
public:
  CustomServo();
  ~CustomServo();
  
  // basic servo functions
  bool attach(int pin, int channel = -1);
  void detach();
  void write(int angle);
  int read();
  bool isAttached();
  
  // smooth movement
  void writeMicroseconds(int microseconds);
};

// servo manager to handle channel allocation
class ServoManager {
private:
  static bool channels_used[16];  // esp32 has 16 ledc channels
  
public:
  static int allocateChannel();
  static void freeChannel(int channel);
  static void init();
};
