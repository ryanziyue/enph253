#include "custom_servo.h"

// static member initialization
bool ServoManager::channels_used[16] = {false};

// servo manager implementation
void ServoManager::init() {
  // mark first 4 channels as used by motor controller
  for (int i = 0; i < 4; i++) {
    channels_used[i] = true;
  }
}

int ServoManager::allocateChannel() {
  for (int i = 4; i < 16; i++) {  // start from channel 4 (motors use 0-3)
    if (!channels_used[i]) {
      channels_used[i] = true;
      return i;
    }
  }
  return -1;  // no free channels
}

void ServoManager::freeChannel(int channel) {
  if (channel >= 0 && channel < 16) {
    channels_used[channel] = false;
  }
}

// custom servo implementation
CustomServo::CustomServo() : pin(-1), channel(-1), current_angle(90), attached(false) {}

CustomServo::~CustomServo() {
  detach();
}

bool CustomServo::attach(int servo_pin, int servo_channel) {
  if (attached) {
    detach();
  }
  
  pin = servo_pin;
  
  // allocate channel if not provided
  if (servo_channel == -1) {
    channel = ServoManager::allocateChannel();
  } else {
    channel = servo_channel;
  }
  
  if (channel == -1) {
    Serial.println("Error: No free LEDC channels for servo");
    return false;
  }
  
  // configure ledc channel
  ledcSetup(channel, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(pin, channel);
  
  // set to center position
  write(90);
  attached = true;
  
  Serial.print("Servo attached to pin ");
  Serial.print(pin);
  Serial.print(" on channel ");
  Serial.println(channel);
  
  return true;
}

void CustomServo::detach() {
  if (attached) {
    ledcDetachPin(pin);
    ServoManager::freeChannel(channel);
    attached = false;
    pin = -1;
    channel = -1;
  }
}

void CustomServo::write(int angle) {
  if (!attached) return;
  
  // constrain angle
  angle = constrain(angle, 0, 180);
  current_angle = angle;
  
  int duty = angleToDuty(angle);
  ledcWrite(channel, duty);
}

int CustomServo::read() {
  return current_angle;
}

bool CustomServo::isAttached() {
  return attached;
}

void CustomServo::writeMicroseconds(int microseconds) {
  if (!attached) return;
  
  // constrain microseconds
  microseconds = constrain(microseconds, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  
  // calculate duty cycle
  // duty = (microseconds / period_us) * max_duty
  int max_duty = (1 << SERVO_RESOLUTION) - 1;  // 2^16 - 1
  int duty = (microseconds * max_duty) / PERIOD_US;
  
  ledcWrite(channel, duty);
  
  // update angle approximation
  current_angle = map(microseconds, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 180);
}

int CustomServo::angleToDuty(int angle) {
  // map angle to pulse width in microseconds
  int pulse_width = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  
  // calculate duty cycle
  int max_duty = (1 << SERVO_RESOLUTION) - 1;  // 2^16 - 1
  int duty = (pulse_width * max_duty) / PERIOD_US;
  
  return duty;
}
