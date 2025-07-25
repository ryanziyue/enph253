#!/usr/bin/env python3
"""
Motor Controller

Handles robot movement, line following, and navigation.
"""

import time
import math
from typing import Optional, Dict, Callable, List, Tuple
from enum import Enum


class LineFollowingMode(Enum):
    CAMERA = "camera"
    REFLECTANCE = "reflectance"

class MotorController:
    """
    Motor Controller for robot movement and navigation.
    """
    
    def __init__(self, write_serial_func: Callable[[str], bool]):
        """Initialize motor controller."""
        self.write_serial = write_serial_func
        
        # Current motor state
        self.left_speed = 0    # -100 to 100
        self.right_speed = 0   # -100 to 100
        self.is_moving = False
        
        # Line following state
        self.line_following_mode = LineFollowingMode.CAMERA
        self.line_following_active = False
        self.last_line_center = 50.0  # percentage (50 = center)
        
        # Navigation state
        self.current_heading = 0.0  # degrees
        self.target_heading = 0.0
        
        # PID parameters for line following
        self.pid_params = {
            'kp': 1.0,    # Proportional gain
            'ki': 0.1,    # Integral gain  
            'kd': 0.2,    # Derivative gain
            'integral': 0.0,
            'last_error': 0.0
        }
        
        # Movement parameters
        self.base_speed = 60      # Base forward speed
        self.turn_speed = 40      # Speed when turning
        self.max_correction = 30  # Max speed correction for line following
        
        # Motor speed mapping parameters
        self.min_motor_speed = 150  # Minimum speed where motors actually move
        self.max_motor_speed = 255  # Maximum motor speed
        
        # Detection thresholds  
        self.curvature_threshold = 25.0    # angle difference threshold in degrees to detect curve
        self.barrier_brown_threshold = 70  # % brown pixels to detect barrier
        self.barrier_point_threshold = 3   # min points needed to continue
        
        # Callbacks for detection events
        self.curvature_callback = None
        self.barrier_callback = None
        
        # Debug mode for verbose output
        self.debug_mode = False
        
        print("🚗 Motor Controller initialized")
    
    # ========== LOW-LEVEL MOTOR CONTROL ==========
    
    def map_speed_to_motor(self, controller_speed: float) -> int:
        """
        Map controller speed (-100 to 100) to actual motor speed with dead zone handling.
        
        Controller speed range: -100 to 100 (from PID, etc.)
        Motor speed range: -255 to 255 (actual motor values)
        Dead zone: Below min_motor_speed, motors don't move
        
        Mapping:
        - 0: Maps to 0 (stopped)
        - 1 to 100: Maps to min_motor_speed to max_motor_speed
        - -1 to -100: Maps to -min_motor_speed to -max_motor_speed
        
        Args:
            controller_speed: Speed from controller (-100 to 100)
            
        Returns:
            int: Mapped motor speed (-255 to 255)
        """
        # Clamp input to valid range
        controller_speed = max(-100, min(100, controller_speed))
        
        # Handle zero/stop case
        if abs(controller_speed) < 0.5:  # Small threshold for floating point
            return 0
        
        # Handle positive speeds
        if controller_speed > 0:
            # Map 1-100 to min_motor_speed-max_motor_speed
            # Linear interpolation: y = mx + b
            # At controller_speed = 1: motor_speed = min_motor_speed
            # At controller_speed = 100: motor_speed = max_motor_speed
            slope = (self.max_motor_speed - self.min_motor_speed) / 99.0  # 99 = 100-1
            motor_speed = self.min_motor_speed + slope * (controller_speed - 1)
            return int(motor_speed)
        # Handle negative speeds
        else:  # controller_speed < 0
            # Map -1 to -100 to -min_motor_speed to -max_motor_speed
            # At controller_speed = -1: motor_speed = -min_motor_speed
            # At controller_speed = -100: motor_speed = -max_motor_speed
            abs_controller = abs(controller_speed)
            slope = (self.max_motor_speed - self.min_motor_speed) / 99.0
            abs_motor_speed = self.min_motor_speed + slope * (abs_controller - 1)
            return -int(abs_motor_speed)
    
    def set_motor_speeds_raw(self, left_speed: int, right_speed: int) -> bool:
        """
        Set individual motor speeds using raw motor values.
        
        Args:
            left_speed: Left motor speed (-255 to 255)
            right_speed: Right motor speed (-255 to 255)
        """
        # Clamp speeds to valid range
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))
        
        cmd = f"PI:MC,{left_speed},{right_speed}"
        if self.write_serial(cmd):
            self.left_speed = left_speed
            self.right_speed = right_speed
            self.is_moving = (left_speed != 0 or right_speed != 0)
            return True
        return False
    
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> bool:
        """
        Set individual motor speeds using controller values with dead zone mapping.
        
        Args:
            left_speed: Left motor speed (-100 to 100)
            right_speed: Right motor speed (-100 to 100)
        """
        # Map controller speeds to motor speeds
        mapped_left = self.map_speed_to_motor(left_speed)
        mapped_right = self.map_speed_to_motor(right_speed)
        
        if self.debug_mode:
            print(f"🚗 Speed mapping: L({left_speed:.1f} -> {mapped_left}) R({right_speed:.1f} -> {mapped_right})")
        
        return self.set_motor_speeds_raw(mapped_left, mapped_right)
    
    def stop_motors(self) -> bool:
        """Stop all motors."""
        return self.set_motor_speeds(0, 0)
        
    def set_line_following_mode(self, mode: LineFollowingMode) -> bool:
        """Switch line following sensor mode."""
        self.line_following_mode = mode
    
    def start_line_following(self):
        """Start line following behavior."""
        self.line_following_active = True
        
        if self.line_following_mode == LineFollowingMode.REFLECTANCE:
            self.write_serial("PI:LF,1")
    
    def stop_line_following(self):
        """Stop line following behavior."""
        self.line_following_active = False
        self.write_serial("PI:LF,0") # Toggle off line following on ESP

    def set_base_speed(self, speed):
        self.write_serial(f"PI:LBS,{speed}")

    def set_min_speed(self, speed):
        self.write_serial(f"PI:LMS,{speed}")
    
    # ========== LINE FOLLOWING CONTROL ==========

    def update_line_following(self, line_points: List, center_point=50) -> bool:
        """
        Gets line points from robot controller. Returns if the reflectance mode is selected.
        Calculates weighted average of points and sends the error to the PID controller.
        
        Args:
            line_points: List of detected line points with x_percent, y, brown_percentage, etc.
            
        Returns:
            bool: True if line following update was successful
        """
        # Check if we should be using camera mode
        if not self.line_following_active or self.line_following_mode == LineFollowingMode.REFLECTANCE:
            return False
        
        # Check if we have any line points
        if not line_points:
            # No line detected - could trigger line loss behavior
            print("⚠️ No line points detected")
            return False
        
        # Calculate weighted average center
        # Points at the bottom (higher y values) get more weight
        total_weighted_x = 0.0
        total_weight = 0.0
        total_brown = 0.0
        
        # Find the range of y values for normalization
        min_y = min(point['y'] for point in line_points)
        max_y = max(point['y'] for point in line_points)
        y_range = max_y - min_y if max_y > min_y else 1
        
        for point in line_points:
            # Calculate weight based on y position (bottom = higher weight)
            # Normalize y to 0-1 range, then use exponential weighting
            y_normalized = (point['y'] - min_y) / y_range
            weight = math.exp(2 * y_normalized)  # Exponential weighting factor
            
            # Accumulate weighted x position
            x_percent = point.get('x_percent', center_point)
            total_weighted_x += x_percent * weight
            total_weight += weight
        
        # Calculate weighted average center
        if total_weight > 0:
            weighted_center_x = total_weighted_x / total_weight
        else:
            weighted_center_x = center_point  # Default to center if no valid weights
        
        # Calculate error for PID (negative = left of center, positive = right of center)
        error = weighted_center_x - center_point
        
        # Update last line center for next iteration
        self.last_line_center = weighted_center_x
        
        # Apply PID control
        self._pid_line_following(error)
        
        return True
    
    def _pid_line_following(self, error: float):
        """PID-based line following control."""

        # PID calculations
        self.pid_params['integral'] += error
        derivative = error - self.pid_params['last_error']
        
        # Calculate correction
        correction = (self.pid_params['kp'] * error + 
                     self.pid_params['ki'] * self.pid_params['integral'] + 
                     self.pid_params['kd'] * derivative)
        
        # Limit correction
        correction = max(-self.max_correction, min(self.max_correction, correction))
        
        # Apply correction to motor speeds
        left_speed = self.base_speed - correction
        right_speed = self.base_speed + correction
        
        # Update last error
        self.pid_params['last_error'] = error
        
        # Use the new controller speed method (no need to convert to int)
        self.set_motor_speeds(left_speed, right_speed)
    
    # ========== DETECTION ALGORITHMS ==========
    
    def detect_curvature(self, line_points: List, brown_threshold: float = 50.0, 
                        num_lower_points: int = 3, num_upper_points: int = 3):
        """
        Detect curves by comparing the angle difference between lower and upper line segments.
        All angles are calculated relative to the vertical axis (0° = straight up).
        On a straight line, both segments should have similar angles.
        On a curve, the angle difference indicates the curvature.
        
        Args:
            line_points: List of detected line points
            brown_threshold: Threshold to filter out brown points (default: 50.0%)
            num_lower_points: Number of lower points to use for angle calculation (default: 3)
            num_upper_points: Number of upper points to use for angle calculation (default: 3)
            
        Returns:
            float: Angle difference between lower and upper segments in degrees.
                   Positive = turning right, Negative = turning left
        """
        if len(line_points) < 4:  # Need at least 4 points for two segments
            return 0.0
        
        # Filter out brown points
        clean_points = []
        for point in line_points:
            brown_pct = point.get('brown_percentage', 0.0)
            if brown_pct <= brown_threshold:
                clean_points.append(point)
        
        if len(clean_points) < 4:
            if self.debug_mode:
                print(f"🔄 Curvature Detection: Only {len(clean_points)} clean points after brown filtering")
            return 0.0
        
        # Sort clean points by y coordinate (bottom to top)
        sorted_points = sorted(clean_points, key=lambda p: p['y'], reverse=True)
        
        # Get lower points (closer to robot)
        num_lower = min(num_lower_points, len(sorted_points) // 2)
        lower_points = sorted_points[:num_lower]
        
        # Get upper points (further from robot)
        num_upper = min(num_upper_points, len(sorted_points) // 2)
        upper_points = sorted_points[-num_upper:]
        
        # Calculate angle for lower segment
        lower_angle = self._calculate_segment_angle(lower_points)
        if lower_angle is None:
            if self.debug_mode:
                print("🔄 Curvature Detection: Could not calculate lower segment angle")
            return 0.0
        
        # Calculate angle for upper segment
        upper_angle = self._calculate_segment_angle(upper_points)
        if upper_angle is None:
            if self.debug_mode:
                print("🔄 Curvature Detection: Could not calculate upper segment angle")
            return 0.0
        
        # Calculate angle difference
        angle_difference = upper_angle - lower_angle
        
        # Normalize to [-180, 180] range
        while angle_difference > 180:
            angle_difference -= 360
        while angle_difference < -180:
            angle_difference += 360
        
        # Take absolute value for threshold comparison
        abs_angle_difference = abs(angle_difference)
        
        if self.debug_mode:
            print(f"🔄 Curvature Detection:")
            print(f"  Clean points: {len(clean_points)}/{len(line_points)}")
            print(f"  Lower points: {num_lower}, angle: {lower_angle:.1f}°")
            print(f"  Upper points: {num_upper}, angle: {upper_angle:.1f}°")
            print(f"  Angle difference: {angle_difference:.1f}° (abs: {abs_angle_difference:.1f}°)")
            print(f"  Threshold: {self.curvature_threshold}°")
            if angle_difference > 0:
                print("  Direction: Turning RIGHT")
            elif angle_difference < 0:
                print("  Direction: Turning LEFT")
        
        return abs_angle_difference
    
    def _calculate_segment_angle(self, points: List[dict]) -> Optional[float]:
        """
        Calculate the angle of a line segment relative to the vertical axis.
        Uses least squares fitting for robustness.
        
        Args:
            points: List of points forming the segment
            
        Returns:
            float: Angle in degrees relative to vertical (0° = straight up)
                   Positive angles = leaning right, Negative = leaning left
            None: If angle cannot be calculated
        """
        if len(points) < 2:
            return None
        
        # Extract x and y coordinates
        x_coords = []
        y_coords = []
        
        for point in points:
            # Get x coordinate (prefer x_percent if available)
            if 'x_percent' in point:
                x = point['x_percent']
            elif 'x' in point:
                x = point['x']
            else:
                continue
                
            y = point['y']
            x_coords.append(x)
            y_coords.append(y)
        
        if len(x_coords) < 2:
            return None
        
        # Use least squares to fit a line through the points
        # This is more robust than just using endpoints
        x_mean = sum(x_coords) / len(x_coords)
        y_mean = sum(y_coords) / len(y_coords)
        
        # Calculate slope components
        numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_coords, y_coords))
        denominator = sum((y - y_mean) ** 2 for y in y_coords)
        
        if abs(denominator) < 0.001:  # Nearly horizontal line
            # Use x variance instead
            x_variance = sum((x - x_mean) ** 2 for x in x_coords)
            if x_variance < 0.001:  # All points at same location
                return 0.0  # Assume vertical
            else:
                return 90.0 if numerator > 0 else -90.0  # Horizontal line
        
        # Calculate angle from vertical
        # Note: We use y difference in denominator because we want angle from vertical
        dx_dy = numerator / denominator  # Change in x per unit change in y
        angle_rad = math.atan(dx_dy)
        angle_deg = math.degrees(angle_rad)
        
        return angle_deg
    
    def check_barrier_area(self, barrier_percentage: float) -> bool:
        """
        Check for barrier using dedicated area detection.
        
        Args:
            barrier_percentage: Brown percentage from barrier detection area (0-100)
            
        Returns:
            bool: True if barrier detected
        """
        is_barrier = barrier_percentage > self.barrier_brown_threshold
        
        if is_barrier and self.debug_mode:
            print(f"🚧 Barrier detected! Area brown: {barrier_percentage:.1f}% (threshold: {self.barrier_brown_threshold}%)")
        
        # Call barrier callback if set
        if is_barrier and self.barrier_callback:
            self.barrier_callback()
        
        return is_barrier
    
    # ========== BASIC MOVEMENT COMMANDS ==========
    
    def move_forward(self, speed: int) -> bool:
        """Move robot forward at specified speed."""
        return self.set_motor_speeds(speed, speed)
    
    def move_backward(self, speed: int) -> bool:
        """Move robot backward at specified speed."""
        return self.set_motor_speeds(-speed, -speed)
    
    def turn_left(self, speed: int) -> bool:
        """Turn robot left at specified speed."""
        return self.set_motor_speeds(-speed, speed)
    
    def turn_right(self, speed: int) -> bool:
        """Turn robot right at specified speed."""
        return self.set_motor_speeds(speed, -speed)
    
    # ========== CONFIGURATION ==========
    
    def set_pid_parameters(self, kp: float, ki: float, kd: float):
        """Update PID parameters for line following."""
        self.pid_params.update({'kp': kp, 'ki': ki, 'kd': kd})
        print(f"🚗 PID updated: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def set_speeds(self, base_speed: int, turn_speed: int):
        """Update speed parameters."""
        self.base_speed = base_speed
        self.turn_speed = turn_speed
        print(f"🚗 Speeds updated: Base={base_speed}, Turn={turn_speed}")
    
    def set_debug_mode(self, enabled: bool):
        """Enable or disable debug output."""
        self.debug_mode = enabled
        print(f"🐛 Debug mode: {'enabled' if enabled else 'disabled'}")
    
    def set_curvature_threshold(self, threshold: float):
        """Set curvature detection threshold in degrees."""
        self.curvature_threshold = max(5.0, min(90.0, threshold))
        print(f"🔄 Curvature threshold set to {self.curvature_threshold}°")
    
    def set_motor_mapping(self, min_speed: int, max_speed: int = 255):
        """
        Set motor speed mapping parameters.
        
        Args:
            min_speed: Minimum motor speed where robot actually moves (e.g., 110)
            max_speed: Maximum motor speed (default: 255)
        """
        self.min_motor_speed = max(0, min(255, min_speed))
        self.max_motor_speed = max(self.min_motor_speed, min(255, max_speed))
        print(f"🚗 Motor mapping updated: min={self.min_motor_speed}, max={self.max_motor_speed}")
    
    def get_motor_status(self) -> Dict:
        """Get current motor and navigation status."""
        return {
            'motor_speeds': {'left': self.left_speed, 'right': self.right_speed},
            'is_moving': self.is_moving,
            'line_following': {
                'active': self.line_following_active,
                'mode': self.line_following_mode.value,
                'last_center': self.last_line_center
            },
            'pid_params': self.pid_params.copy(),
            'speeds': {
                'base_speed': self.base_speed,
                'turn_speed': self.turn_speed
            },
            'thresholds': {
                'curvature_threshold': self.curvature_threshold,
                'barrier_brown_threshold': self.barrier_brown_threshold,
                'barrier_point_threshold': self.barrier_point_threshold
            },
            'debug_mode': self.debug_mode
        }