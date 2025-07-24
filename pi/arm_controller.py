#!/usr/bin/env python3
"""
Enhanced Arm Controller

Handles robotic arm with turret, IK, and pet-specific operations.
"""

import time
import math
from typing import Optional, Tuple, Dict, Callable
from enum import Enum

class ArmController:
    """
    Enhanced Robotic Arm Controller with turret and IK capabilities.
    """
    
    def __init__(self, write_serial_func: Callable[[str], bool], 
                 robot_controller=None):
        """Initialize enhanced arm controller."""
        self.write_serial = write_serial_func
        self.robot_controller = robot_controller  # Reference to robot controller for serial reading
        
        # Limits
        self.limits = {
            'turret_min': 0, 'turret_max': 180,
            'joint1_min': 0, 'joint1_max': 180,
            'joint2_min': 0, 'joint2_max': 180,
            'wrist_min': 0, 'wrist_max': 180
        }
        
        # Predefined positions
        self.positions = {
            'neutral': {'joint1': 90, 'joint2': 90, 'wrist': 90},
            'basket': {'joint1': 45, 'joint2': 135, 'wrist': 0},
            'chute': {'joint1': 30, 'joint2': 150, 'wrist': 45}
        }
        
        # Status tracking
        self.last_command_ack = False
        self.movement_complete = False
        self.limit_switch_triggered = False
        self.wrist_locked = False
        
        print("🦾 Enhanced Arm Controller initialized")
    
    # ========== LOW-LEVEL CONTROL ==========
    
    def set_turret_angle(self, angle: float, wait_for_ack: bool = True, timeout=1) -> bool:
        """Set turret rotation angle."""
        
        cmd = f"PI:SP,{angle},-,-"
        if self.write_serial(cmd):
            self.turret_angle = angle
            print(f"🦾 Setting turret to {angle}°")
            
            if wait_for_ack:
                return self._wait_for_movement_complete(timeout=timeout)
            return True
        return False
    
    def set_arm_angles(self, turret=None, shoulder=None, elbow=None, wait_for_ack: bool = True, timeout=1):
        """Set turret rotation angle."""

        cmd = f"PI:SP,{"-" if turret == None else turret},{"-" if shoulder == None else shoulder},{"-" if elbow == None else elbow}"
        if self.write_serial(cmd):

            if wait_for_ack:
                return self._wait_for_movement_complete(timeout=timeout)
            return True
        return False
    
    def set_arm_ik_position(self, x: float, y: float, wait_for_ack: bool = True, timeout=2) -> bool:
        """
        Set arm position using inverse kinematics.
        
        Args:
            x, y: Target position relative to turret base
            wait_for_ack: Wait for ESP confirmation
        """
        cmd = f"PI:GP,{x},{y}"
        if self.write_serial(cmd):
            if wait_for_ack:
                return self._wait_for_movement_complete(timeout=timeout)
            return True
        return False
    
    def set_arm_ik_velocity(self, vx: float, vy: float, wait_for_ack: bool = True, timeout=2) -> bool:
        """Set arm velocity in Cartesian space."""
        cmd = f"PI:GV,{vx:.1f},{vy:.1f}"
        if self.write_serial(cmd):            
            if wait_for_ack:
                return self._wait_for_movement_complete(timeout=timeout)
            return True
        return False
    
    def set_wrist_toggle(self, toggled: bool):
        toggled = 1 if toggled else 0
        cmd = f"PI:WLT,{toggled}"
        if self.write_serial(cmd):
            return True
        return False
    
    def set_wrist_angle(self, angle: float) -> bool:
        
        cmd = f"PI:WLA,{angle}"
        if self.write_serial(cmd):
            return True
        return False
    
    def set_wrist_angle_unlock(self, angle: float):
        cmd = f"PI:WP,{angle}"
        if self.write_serial(cmd):
            return True
        return False
    
    def set_claw_angle(self, angle: int = 50) -> bool:
        cmd = f"PI:CP,{angle}"
        if self.write_serial(cmd):
            return True
        return False
    
    # ========== STATUS AND FEEDBACK ==========
    
    def _wait_for_movement_complete(self, timeout: float = 5.0) -> bool:
        """
        Wait for movement completion acknowledgment from ESP.
        
        Args:
            timeout: Maximum time to wait for acknowledgment
            
        Returns:
            bool: True if movement complete message received, False if timeout
        """
        if self.robot_controller and hasattr(self.robot_controller, 'wait_for_serial_message'):
            # Use serial message watching - timeout is handled in wait_for_serial_message
            print("🦾 Waiting for movement completion...")
            return self.robot_controller.wait_for_serial_message(
                expected_message="ESP:FINISH", 
                timeout=timeout,
                contains=True,  # Allow partial matches like "ACK:MOVE_COMPLETE:TURRET"
                clear_buffer=True  # Clear stale messages before waiting
            )
        else:
            # Fallback: no robot controller available
            print("⚠️ No robot controller available for serial message watching")
            return False
    
    def wait_for_limit_switch(self, timeout: float = 10.0) -> bool:
        """
        Wait for limit switch trigger from ESP.
        
        Args:
            timeout: Maximum time to wait for limit switch activation
            
        Returns:
            bool: True if limit switch message received, False if timeout
        """
        if self.robot_controller and hasattr(self.robot_controller, 'wait_for_serial_message'):
            # Use serial message watching - timeout is handled in wait_for_serial_message
            print("🦾 Waiting for limit switch trigger...")
            return self.robot_controller.wait_for_serial_message(
                expected_message="ESP:SWITCH", 
                timeout=timeout,
                contains=True,  # Allow partial matches like "ACK:LIMIT_SWITCH:TRIGGERED"
                clear_buffer=True  # Clear stale messages before waiting
            )
        else:
            # Fallback: no robot controller available
            print("⚠️ No robot controller available for serial message watching")
            return False
    
    def handle_serial_feedback(self, message: str):
        """Process feedback messages from ESP."""
        if message.startswith("ACK:MOVE_COMPLETE"):
            self.movement_complete = True
        elif message.startswith("ACK:LIMIT_SWITCH"):
            self.limit_switch_triggered = True
        elif message.startswith("STATUS:"):
            # Parse status updates from ESP
            self._parse_status_message(message[7:])
    
    def _parse_status_message(self, status_data: str):
        """Parse status message from ESP."""
        try:
            # Example: "turret:45.0,joint1:90.0,joint2:90.0,wrist:90.0,claw:50"
            parts = status_data.split(',')
            for part in parts:
                key, value = part.split(':')
                if key == 'turret':
                    self.turret_angle = float(value)
                elif key == 'joint1':
                    self.arm_joint1 = float(value)
                elif key == 'joint2':
                    self.arm_joint2 = float(value)
                elif key == 'wrist':
                    self.wrist_angle = float(value)
                elif key == 'claw':
                    self.claw_position = int(value)
        except Exception as e:
            print(f"❌ Error parsing status: {e}")