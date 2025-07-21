#pragma once
#include <Arduino.h>

// pin definitions
#define M1_PIN_FWD         20
#define M1_PIN_REV         21
#define M2_PIN_FWD         19
#define M2_PIN_REV         22

#define SERVO_BASE_PIN     14
#define SERVO_SHOULDER_L   12
#define SERVO_SHOULDER_R   13
#define SERVO_ELBOW_PIN    33
#define SERVO_WRIST_PIN    27
#define SERVO_CLAW_PIN     15

#define RX1_PIN            7
#define TX1_PIN            8

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