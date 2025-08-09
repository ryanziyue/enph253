#pragma once
#include <Arduino.h>
#include "custom_servo.h"
#include "main.h"

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
  
  unsigned long last_millis;
  unsigned long last_ik_millis;
  
  // ik velocity mode
  bool ik_vel_enabled;
  float ik_target_x, ik_target_y;
  float ik_vx, ik_vy;
  
  // wrist lock
  bool wrist_lock_enabled;
  float wrist_lock_angle; 
  unsigned long wrist_manual_control_time;
  
  bool initialized;
  
  // helper methods
  void updateMotion();
  void applyWristLock();
  float convertToRightShoulderAngle(float left_angle);
  void clearSpeedCommands();

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
  
  // centralized shoulder control methods
  void setShoulderTarget(float angle);
  void setShoulderSpeed(float speed);
  
  // advanced control
  void setGlobalPosition(float x, float y);
  void setGlobalVelocity(float vx, float vy);
  void setWristLock(bool enabled, float angle_degrees = 0);
  void setWristLockAngle(float angle_degrees);
  float getWristLockAngle() const { return wrist_lock_angle; }
  bool isWristLocked() const { return wrist_lock_enabled; }
  void setClaw(float angle);
  void setMaxSpeed(int idx, float maxSpeed);
  
  // kinematics
  Point forwardKinematics(float theta1, float theta2);
  bool inverseKinematics(float x, float y, float &theta1, float &theta2);
  
  void setBaseTarget(float user_angle);
  void diagnoseWristIssue();   

  // status queries
  Point getCurrentPosition();
  bool isMoving() const;

  // queries
  float getBaseAngle() const;
  
  // debug
  void printStatus();
};