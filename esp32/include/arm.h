#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "pi.h"

// ——————— CONFIG ———————
// index constants
#define IDX_BASE         0
#define IDX_SHOULDER_L   1
#define IDX_SHOULDER_R   2
#define IDX_ELBOW        3
#define IDX_WRIST        4

// constraints from old code
// Constants from your existing code
#define NUM_SERVOS       5
#define POS_TOLERANCE    0.5
#define L1               16.0     // cm
#define L2               14.5     // cm
#define ELBOW_OFFSET     10.0     // degrees
#define DEG2RAD          (3.14159265/180.0)
#define RAD2DEG          (180.0/3.14159265)


