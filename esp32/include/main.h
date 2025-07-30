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

// line following params
#define K_P                     45.0
#define K_I                     0.0
#define K_D                     0.0
#define K_O                     2.0
#define TARGET_POSITION         220.0
#define CURRENT_POSITION        0.0
#define BASE_SPEED              190
#define SENSOR_THRESHOLD_R1     1.7
#define SENSOR_THRESHOLD_L1     1.7
#define SENSOR_THRESHOLD_R2     1.8
#define SENSOR_THRESHOLD_L2     1.8

// speed parameters
#define MOTOR_MIN_SPEED         175
#define MOTOR_MAX_SPEED         255
#define IDX_BASE_SPEED          45.0
#define IDX_SHOULDER_SPEED      30.0
#define IDX_ELBOW_SPEED         60.0
#define IDX_WRIST_SPEED         60.0

#define WRIST_DISABLE_TIME 5000

// servo pins
#define NUM_SERVOS       5
#define IDX_BASE         0
#define IDX_SHOULDER_L   1
#define IDX_SHOULDER_R   2
#define IDX_ELBOW        3
#define IDX_WRIST        4

// forward declarations
class MotorController;
class ServoController;
class PiComm;
struct PiCommand;

// function declarations
void handlePiCommand(const PiCommand& cmd);
void printSystemStatus();
void emergencyStop();