#pragma once
#include <Arduino.h>
#include "custom_servo.h"

// servo pins
#define NUM_SERVOS       5
#define IDX_BASE         0
#define IDX_SHOULDER_L   1
#define IDX_SHOULDER_R   2
#define IDX_ELBOW        3
#define IDX_WRIST        4

// kinematics constants
#define ARM_L1               19.6
#define ARM_L2               15.8
#define ELBOW_OFFSET         10.0
#define DEG2RAD              (3.14159265/180.0)
#define RAD2DEG              (180.0/3.14159265)
#define WRIST_LOWER_LIMIT    -45
#define WRIST_UPPER_LIMIT     45

#define SHOULDER_R_OFFSET   173.0

struct Point {
  float x, y;
  Point(float x = 0, float y = 0) : x(x), y(y) {}
};

class ServoController {
private:
  CustomServo servos[NUM_SERVOS];
  CustomServo claw;
  
  float current_pos[NUM_SERVOS];
  float target_pos[NUM_SERVOS];
  float speed_cmd[NUM_SERVOS];
  float max_speed[NUM_SERVOS];
  float home_pos[NUM_SERVOS];
  
  float shoulder_r_offset;

  unsigned long last_millis;
  unsigned long last_ik_millis;
  
  // ik velocity mode
  bool ik_vel_enabled;
  float ik_target_x, ik_target_y;
  float ik_vx, ik_vy;
  
  // wrist lock
  bool wrist_lock_enabled;
  float wrist_lock_angle;  // Desired wrist angle relative to horizontal (0 = level)
  unsigned long wrist_lock_disable_until;  // Timestamp when temporary disable expires
  
  bool initialized;
  
  // helper methods
  void updateMotion();
  void applyWristLock();
  float convertToRightShoulderAngle(float left_angle);



public:
  ServoController();
  void init();
  void update();
  
  // direct control methods
  void setTarget(int idx, float angle);
  void setSpeed(int idx, float speed);
  void stopAll();
  void resetPosition();
  void zeroAllServos();

  //shoulder controls
  void setShoulderTarget(float angle);
  void setShoulderSpeed(float speed);
  
  // advanced control
  void setGlobalPosition(float x, float y);
  void setGlobalVelocity(float vx, float vy);
  void setWristLock(bool enabled, float angle_degrees = 0);  // angle relative to horizontal
  void setWristLockAngle(float angle_degrees);  // Just change the angle without toggling lock
  float getWristLockAngle() const { return wrist_lock_angle; }  // Get current lock angle
  void temporarilyDisableWristLock(int duration_ms = 5000);  // Temporarily disable for manual control
  void setClaw(float angle);
  void setMaxSpeed(int idx, float maxSpeed);
  
  // kinematics
  Point forwardKinematics(float theta1, float theta2);
  bool inverseKinematics(float x, float y, float &theta1, float &theta2);
  
  // status queries
  Point getCurrentPosition();
  bool isMoving() const;
  
  // debug
  void printStatus();
};