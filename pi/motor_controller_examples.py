#!/usr/bin/env python3
"""
Motor Controller Examples

Comprehensive examples demonstrating all motor controller functionalities.
Run this file to see examples of how to use each feature.
"""

import time
from motor_controller import MotorController, LineFollowingMode


def mock_write_serial(message: str) -> bool:
    """Mock serial write function for testing."""
    print(f"📤 Serial: {message}")
    return True


def basic_movement_examples(motors: MotorController):
    """Examples of basic robot movement."""
    print("\n🚗 === BASIC MOVEMENT EXAMPLES ===")
    
    # Direct motor speed control
    print("\n1. Direct Motor Speed Control:")
    motors.set_motor_speeds(50, 50)    # Move forward at 50% speed
    time.sleep(1)
    motors.set_motor_speeds(-30, -30)  # Move backward at 30% speed  
    time.sleep(1)
    motors.set_motor_speeds(-40, 40)   # Turn left (left motor backward, right forward)
    time.sleep(1)
    motors.set_motor_speeds(40, -40)   # Turn right (left motor forward, right backward)
    time.sleep(1)
    motors.stop_motors()               # Stop
    
    # High-level movement commands
    print("\n2. High-Level Movement Commands:")
    motors.move_forward(60)     # Move forward at 60% speed
    time.sleep(1)
    motors.move_backward(40)    # Move backward at 40% speed
    time.sleep(1)
    motors.turn_left(50)        # Turn left at 50% speed
    time.sleep(1)
    motors.turn_right(50)       # Turn right at 50% speed
    time.sleep(1)
    motors.stop_motors()        # Stop all motors


def line_following_examples(motors: MotorController):
    """Examples of line following functionality."""
    print("\n📏 === LINE FOLLOWING EXAMPLES ===")
    
    # Set line following mode
    print("\n1. Line Following Modes:")
    motors.set_line_following_mode(LineFollowingMode.CAMERA)
    print("   ✅ Camera-based line following enabled")
    
    motors.set_line_following_mode(LineFollowingMode.REFLECTANCE)
    print("   ✅ Reflectance sensor line following enabled")
    
    motors.set_line_following_mode(LineFollowingMode.DISABLED)
    print("   ✅ Line following disabled")
    
    # Start/stop line following
    print("\n2. Line Following Control:")
    motors.set_line_following_mode(LineFollowingMode.CAMERA)
    motors.start_line_following()
    print("   ✅ Line following started")
    
    # Simulate line following updates
    print("\n3. Line Following Simulation:")
    test_scenarios = [
        (50.0, 8, 10),   # Centered line
        (35.0, 8, 15),   # Line to the left
        (65.0, 8, 20),   # Line to the right
        (25.0, 6, 25),   # Sharp left curve
        (75.0, 6, 30),   # Sharp right curve
        (50.0, 2, 80),   # Barrier detected
    ]
    
    for i, (center, points, brown) in enumerate(test_scenarios):
        print(f"   Scenario {i+1}: Line center={center}%, Points={points}, Brown={brown}%")
        motors.update_line_following(center, points, brown)
        time.sleep(0.5)
    
    motors.stop_line_following()
    print("   ✅ Line following stopped")


def pid_tuning_examples(motors: MotorController):
    """Examples of PID parameter tuning."""
    print("\n🎛️ === PID TUNING EXAMPLES ===")
    
    # Show current PID parameters
    print("\n1. Current PID Parameters:")
    params = motors.pid_params
    print(f"   Kp (Proportional): {params['kp']}")
    print(f"   Ki (Integral): {params['ki']}")
    print(f"   Kd (Derivative): {params['kd']}")
    
    # Update PID parameters for different scenarios
    print("\n2. PID Tuning for Different Scenarios:")
    
    # Aggressive turning for sharp curves
    print("   Sharp Curves - Aggressive PID:")
    motors.set_pid_parameters(kp=1.5, ki=0.1, kd=0.3)
    print(f"   ✅ PID set to: Kp=1.5, Ki=0.1, Kd=0.3")
    
    # Smooth following for straight lines
    print("   Straight Lines - Smooth PID:")
    motors.set_pid_parameters(kp=0.8, ki=0.05, kd=0.15)
    print(f"   ✅ PID set to: Kp=0.8, Ki=0.05, Kd=0.15")
    
    # Default balanced settings
    print("   Default Balanced Settings:")
    motors.set_pid_parameters(kp=1.0, ki=0.1, kd=0.2)
    print(f"   ✅ PID set to: Kp=1.0, Ki=0.1, Kd=0.2")


def speed_control_examples(motors: MotorController):
    """Examples of speed control and configuration."""
    print("\n⚡ === SPEED CONTROL EXAMPLES ===")
    
    # Show current speeds
    print("\n1. Current Speed Settings:")
    status = motors.get_motor_status()
    print(f"   Base Speed: {status['speeds']['base_speed']}")
    print(f"   Turn Speed: {status['speeds']['turn_speed']}")
    
    # Speed configurations for different situations
    print("\n2. Speed Configurations:")
    
    # High speed for open areas
    print("   High Speed Configuration:")
    motors.set_speeds(base_speed=80, turn_speed=60)
    print("   ✅ High speed: Base=80, Turn=60")
    
    # Medium speed for normal operation
    print("   Normal Speed Configuration:")
    motors.set_speeds(base_speed=60, turn_speed=40)
    print("   ✅ Normal speed: Base=60, Turn=40")
    
    # Low speed for precision
    print("   Precision Speed Configuration:")
    motors.set_speeds(base_speed=30, turn_speed=20)
    print("   ✅ Precision speed: Base=30, Turn=20")
    
    # Reset to default
    motors.set_speeds(base_speed=60, turn_speed=40)


def navigation_examples(motors: MotorController):
    """Examples of navigation and heading control."""
    print("\n🧭 === NAVIGATION EXAMPLES ===")
    
    # Heading control
    print("\n1. Heading Control:")
    test_headings = [0, 45, 90, 135, 180, 225, 270, 315]
    
    for heading in test_headings:
        print(f"   Turning to {heading}°...")
        motors.turn_to_heading(heading)
        time.sleep(0.5)
    
    # Turn around when line is lost
    print("\n2. Line Recovery:")
    print("   Simulating line loss - turning around...")
    motors.turn_robot_around(timeout=5.0)


def detection_examples(motors: MotorController):
    """Examples of curvature and barrier detection."""
    print("\n🔍 === DETECTION EXAMPLES ===")
    
    # Curvature detection
    print("\n1. Curvature Detection:")
    test_centers = [50, 45, 35, 20, 75, 85]  # Simulated line centers
    
    motors.last_line_center = 50.0  # Start at center
    
    for center in test_centers:
        is_curve = motors.detect_curvature(center)
        change = abs(center - motors.last_line_center)
        print(f"   Line center: {center}% (change: {change:.1f}%) - {'🔄 CURVE' if is_curve else '📏 Normal'}")
        motors.last_line_center = center
    
    # Barrier detection
    print("\n2. Barrier Detection:")
    test_scenarios = [
        (8, 10),   # Normal line
        (6, 30),   # Some brown pixels
        (3, 50),   # Low points, medium brown
        (1, 80),   # Very few points, high brown
        (8, 80),   # Good points but high brown
    ]
    
    for points, brown in test_scenarios:
        is_barrier = motors.detect_barrier(points, brown)
        print(f"   Points: {points}, Brown: {brown}% - {'🚧 BARRIER' if is_barrier else '✅ Clear'}")


def advanced_scenarios(motors: MotorController):
    """Advanced usage scenarios."""
    print("\n🎯 === ADVANCED SCENARIOS ===")
    
    # Scenario 1: Adaptive line following
    print("\n1. Adaptive Line Following:")
    motors.set_line_following_mode(LineFollowingMode.CAMERA)
    motors.start_line_following()
    
    # Normal following
    print("   Normal line following...")
    motors.update_line_following(line_center_percent=50, point_count=8, brown_percentage=10)
    
    # Detect sharp curve and adapt
    print("   Sharp curve detected - adapting...")
    motors.update_line_following(line_center_percent=25, point_count=6, brown_percentage=15)
    
    # Detect barrier and handle
    print("   Barrier detected - handling...")
    motors.update_line_following(line_center_percent=45, point_count=2, brown_percentage=85)
    
    motors.stop_line_following()
    
    # Scenario 2: Mixed mode operation
    print("\n2. Mixed Mode Operation:")
    print("   Starting with camera mode...")
    motors.set_line_following_mode(LineFollowingMode.CAMERA)
    time.sleep(1)
    
    print("   Switching to reflectance sensor...")
    motors.set_line_following_mode(LineFollowingMode.REFLECTANCE)
    time.sleep(1)
    
    print("   Disabling line following for manual control...")
    motors.set_line_following_mode(LineFollowingMode.DISABLED)
    motors.move_forward(50)
    time.sleep(1)
    motors.stop_motors()


def status_monitoring_examples(motors: MotorController):
    """Examples of status monitoring and diagnostics."""
    print("\n📊 === STATUS MONITORING EXAMPLES ===")
    
    # Get comprehensive status
    status = motors.get_motor_status()
    
    print("\n1. Complete Motor Status:")
    print(f"   Motor Speeds: Left={status['motor_speeds']['left']}, Right={status['motor_speeds']['right']}")
    print(f"   Is Moving: {status['is_moving']}")
    print(f"   Line Following Active: {status['line_following']['active']}")
    print(f"   Line Following Mode: {status['line_following']['mode']}")
    print(f"   Last Line Center: {status['line_following']['last_center']:.1f}%")
    print(f"   Current Heading: {status['navigation']['current_heading']:.1f}°")
    print(f"   Target Heading: {status['navigation']['target_heading']:.1f}°")
    print(f"   Base Speed: {status['speeds']['base_speed']}")
    print(f"   Turn Speed: {status['speeds']['turn_speed']}")
    
    print("\n2. PID Status:")
    pid = status['pid_params']
    print(f"   Kp: {pid['kp']}, Ki: {pid['ki']}, Kd: {pid['kd']}")
    print(f"   Integral: {pid['integral']:.2f}, Last Error: {pid['last_error']:.2f}")


def main():
    """Run all motor controller examples."""
    print("🤖 Motor Controller Examples")
    print("=" * 50)
    
    # Create motor controller with mock serial
    motors = MotorController(mock_write_serial)
    
    try:
        # Run all example categories
        basic_movement_examples(motors)
        line_following_examples(motors)
        pid_tuning_examples(motors)
        speed_control_examples(motors)
        navigation_examples(motors)
        detection_examples(motors)
        advanced_scenarios(motors)
        status_monitoring_examples(motors)
        
        print("\n✅ All motor controller examples completed!")
        print("\nIntegration with Robot Controller:")
        print("- Use 'F' to start/stop line following")
        print("- Use 'P' to show PID parameters")
        print("- Use 'U' to update PID parameters")
        print("- Use 'V' to set motor speeds")
        print("- Use 'L' to switch line following mode")
        print("- Use 'N' to navigate to heading")
        print("- Use 'B' to test barrier handling")
        print("- Use 'X' to show motor status")
        print("- Use 'W/S' for forward/backward")
        print("- Use 'Left/Right' for turning")
        print("- Use 'E' for emergency stop")
        
    except KeyboardInterrupt:
        print("\n\n🛑 Examples interrupted by user")
    finally:
        motors.stop_motors()
        print("🏁 Motor controller examples finished")


if __name__ == "__main__":
    main()