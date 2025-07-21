#pragma once
#include <Arduino.h>

// arm pins
#define SERVO_BASE_PIN     14
#define SERVO_SHOULDER_L   12
#define SERVO_SHOULDER_R   13
#define SERVO_ELBOW_PIN    33
#define SERVO_WRIST_PIN    27
#define SERVO_CLAW_PIN     15

// limit switch
#define LIMIT_SWITCH_PIN    7

// motor pins
#define M1_PIN_FWD         20
#define M1_CHAN_FWD         0
#define M1_PIN_REV         21
#define M1_CHAN_REV         1

#define M2_PIN_FWD         19
#define M2_CHAN_FWD         2
#define M2_PIN_REV         22
#define M2_CHAN_REV         3

// reflectance sensor pins
#define ANALOG_PIN_R1 37  // GPIO37 = ADC1_CHANNEL_1
#define ANALOG_PIN_L1 38  // GPIO38 = ADC1_CHANNEL_2
#define ANALOG_PIN_R2 34  // GPIO34 = ADC1_CHANNEL_6
#define ANALOG_PIN_L2 35  // GPIO35 = ADC1_CHANNEL_7

// pwm settings
#define PWM_FREQ           1000
#define PWM_RES_BITS       8

// forward declarations
class MotorController;
class ServoController;
class PiComm;
struct PiCommand;

// function declarations
void handlePiCommand(const PiCommand& cmd);
void printSystemStatus();
void emergencyStop();