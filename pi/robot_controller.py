#!/usr/bin/env python3
"""
Robot Controller - Main Control Script

Coordinates all robot systems and launches the GUI in a separate thread.
This will be the main entry point for the robot system.
"""

import time
import threading
import signal
import sys
import argparse
import cv2
from typing import Optional, List, Dict
from camera_manager import CameraManager
from robot_gui import RobotGUI, BoundingBox, Point, Line, Text
from line_following_manager import LineFollowingManager
from object_detection_manager import ObjectDetectionManager
from arm_controller import ArmController
from motor_controller import MotorController, LineFollowingMode
import serial

class RobotController:
    """
    Main robot controller that coordinates all subsystems.
    """
    
    def __init__(self, config_file: str = "camera_config.json", enable_gui: bool = True, 
                 serial_port: Optional[str] = None, baudrate: int = 115200):
        """
        Initialize robot controller.
        
        Args:
            config_file: Path to camera configuration file
            enable_gui: Whether to launch the GUI
            serial_port: Serial port path (e.g., '/dev/ttyUSB0' or 'COM3')
            baudrate: Serial baudrate (default: 115200)
        """
        self.config_file = config_file
        self.enable_gui = enable_gui
        self.serial_port = serial_port
        self.baudrate = baudrate
        
        # Initialize subsystems
        print("🤖 Initializing Robot Controller...")
        
        # Camera manager
        self.camera_manager = CameraManager()
        
        # Line following managers for both cameras
        self.line_manager_1: Optional[LineFollowingManager] = None
        self.line_manager_2: Optional[LineFollowingManager] = None
        
        # Object detection manager
        self.object_detection_manager: Optional[ObjectDetectionManager] = None
        
        # Serial port
        self.ser: Optional[serial.Serial] = None
        
        # Arm controller
        self.arm: Optional[ArmController] = None
        
        # Motor controller
        self.motors: Optional[MotorController] = None
        
        # GUI (if enabled)
        self.gui: Optional[RobotGUI] = None
        self.gui_thread: Optional[threading.Thread] = None
        
        # Control state
        self.running = False
        self.control_thread: Optional[threading.Thread] = None
        
        # Performance tracking
        self.loop_count = 0
        self.start_time = time.time()
        
        # Line following statistics
        self.line_stats = {
            'total_distance_1': 0.0,
            'total_distance_2': 0.0,
            'measurement_count_1': 0,
            'measurement_count_2': 0,
            'last_avg_distance_1': 0.0,
            'last_avg_distance_2': 0.0
        }
        
        # Object detection statistics
        self.object_detection_stats = {
            'total_detections': 0,
            'last_detection_time': 0.0,
            'last_detection_confidence': 0.0,
            'last_detection_class': '',
            'detection_rate_per_minute': 0.0
        }
        
        # Timing for 1Hz object detection
        self.last_object_detection_time = 0.0
        
        print("✅ Robot Controller initialized")
    
    def start(self):
        """Start all robot systems."""
        print("🚀 Starting robot systems...")
        
        # Start camera manager
        self.camera_manager.start()
        time.sleep(1)  # Give cameras time to initialize
        
        # Serial is already initialized, nothing more to do
        
        # Initialize line following manager (camera 1 only)
        self.line_manager_1 = LineFollowingManager(self.camera_manager, camera_id=1)
        self.line_manager_2 = None  # No line following on camera 2
        
        # Initialize object detection manager (camera 2 by default)
        try:
            self.object_detection_manager = ObjectDetectionManager(self.camera_manager, camera_id=2, 
                                                                   config_file="object_detection_config_yolov11n.json")
            
            # Configure crop to bottom half of image for object detection
            # Get frame to determine dimensions
            frame = self.camera_manager.get_frame(2)
            if frame is not None:
                height, width = frame.shape[:2]
                # Crop to bottom half
                # self.object_detection_manager.setup_crop(crop_top=height // 2)
                print(f"✅ Object detection initialized with bottom-half crop (top={height//2}px of {height}px)")
            else:
                # Fallback: assume 480p and crop top half
                # self.object_detection_manager.setup_crop(crop_top=240)
                print("✅ Object detection initialized with default bottom-half crop")
                
        except Exception as e:
            print(f"⚠️ Object detection manager failed to initialize: {e}")
            self.object_detection_manager = None
        
        # Initialize serial port
        if self.serial_port:
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                print(f"✅ Serial port initialized: {self.serial_port} @ {self.baudrate} baud")
                
                # Initialize arm controller with reference to self for serial reading
                from arm_controller import ArmController
                self.arm = ArmController(self.write_serial, robot_controller=self)
                print("✅ Arm controller initialized")
                
                # Initialize motor controller
                self.motors = MotorController(self.write_serial)
                
                print("✅ Motor controller initialized with callbacks")
            except Exception as e:
                print(f"⚠️ Serial port failed to initialize: {e}")
                self.ser = None
        
        # Start GUI if enabled (after object detection manager is ready)
        if self.enable_gui:
            self.start_gui()
        
        # Start control loop
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print("✅ All systems started")
    
    def start_gui(self):
        """Start GUI in separate thread."""
        print("🖥️ Starting GUI thread...")
        
        self.gui = RobotGUI(self.camera_manager, self.object_detection_manager)
        self.gui_thread = threading.Thread(target=self.gui.run, daemon=True)
        self.gui_thread.start()
        
        # Give GUI time to initialize
        time.sleep(0.5)
        
        print("✅ GUI thread started")

    def _control_loop(self):
        """Main control loop running in separate thread."""
        print("🔄 Control loop started")

        # self.sweep_thread = threading.Thread(target=self.servo_sweep, daemon=True)
        # self.sweep_thread.start()

        self.motors.set_line_following_mode(LineFollowingMode.REFLECTANCE)
        self.arm.set_wrist_angle(5)
        self.arm.set_claw_angle(90)
        self.motors.set_motor_speeds(0,0)
        self.arm.set_arm_angles(90, 150, 165, wait_for_ack=True, timeout=1)

        input("Test cam")
        while(self.running):
            if self.object_detection_manager:
                result = self.object_detection_manager.run_detection_blocking()
                if result.found:
                    print("Object found")

        input("Enter to start")

        self.main_control()

        # self.motors.set_line_following_base_speed(150)
        # self.line_follow_until_curve(curve_threshold=15, averaging_window=3)
        # time.sleep(1)
        # self.line_follow_time(1)
        # time.sleep(1)
        # self.motors.move_backward(50)
        # time.sleep(0.1)
        # self.turn_around(turn_right=False, starting_speed=70, min_speed=40, after_duration=0.5)
        # self.line_follow_time(1)
        
        input("Enter to pickup")

        # PICKUP

        # self.arm.set_wrist_angle(0)

        # angles_to_check = [45, 55, 65, 75]
        # arm_pos = [15,0,0]
        # self.arm.set_arm_ik_position(15,5,wait_for_ack=True,timeout=1)
        # # self.locate_pet(60,90,45,arm_pos,5, 0.2, 280)
        # self.locate_pet_discrete([45,60,75],arm_pos)

        # grab_pos = [15+10,-6,0]
        # self.grab_pet_direct(grab_pos)
        # time.sleep(1)
        
        # self.arm.set_wrist_angle_unlock(180)
        # self.arm.set_arm_angles(elbow=0,wait_for_ack=True,timeout=0.5)
        # self.arm.set_turret_angle(90)
        # self.arm.set_arm_angles(shoulder=100, wait_for_ack=True,timeout=1.5)
        # self.arm.set_claw_angle(90)
        
        while self.running:
            try:
                # Get current time
                current_time = time.time()
                elapsed = current_time - self.start_time

                input("Press enter to start")
                self.motors.set_base_speed(210)
                self.motors.set_min_speed(195)
                self.line_follow_time(2.5, stop=False)

                self.motors.set_base_speed(190)
                self.motors.set_min_speed(170)
                # 1st and 2nd turn: CT=7, MT=20, AV=3
                self.line_follow_until_curve(curve_threshold=7, max_threshold=20, averaging_window=3)
                
                # # Update motor controller with line following data
                # if self.motors and self.line_manager_1:
                #     self._update_motor_line_following()
                
                # # Process object detection at 1Hz (non-blocking)
                if self.object_detection_manager:
                    self._process_object_detection()
                
                # You can add periodic serial writes here if needed
                # Example: if self.loop_count % 50 == 0:  # Every 5 seconds
                #     self.write_serial("STATUS:OK")
                
                # self.line_follow_until_curve(curve_threshold=5)
                # print("Line reached")

                # points = self.line_manager_1.get_line()
                # print(self.motors.detect_curvature(points))

                # Update loop counter
                self.loop_count += 1
                
                # Control loop rate (10 Hz)
                time.sleep(0.05)
                
            except Exception as e:
                print(f"❌ Control loop error: {e}")
                time.sleep(0.1)
        
        print("🔄 Control loop stopped")
        
    def main_control(self):
        # STEP 1: Move forward until the first curve
        # self.motors.set_base_speed(170)
        # self.motors.set_min_speed(160)
        # self.line_follow_time(1.5, stop=False)
        
        # self.motors.set_base_speed(170)
        # self.motors.set_min_speed(160)
        # # 1st and 2nd turn: CT=7, MT=20, AV=3
        # self.line_follow_until_curve(curve_threshold=7, max_threshold=20, averaging_window=3, stop=True)
        # print("Curve detected")
        # time.sleep(1)
        # self.motors.set_base_speed(190)
        # self.motors.set_min_speed(175)
        # self.line_follow_time(0.75)
        # self.motors.set_motor_speeds_raw(180,180)
        # time.sleep(0.25)
        # self.motors.stop_motors()
        
        # STEP 2: Pick up the pet
        input("Enter to start step 2")
        self.arm.set_wrist_angle(0)

        arm_pos = [15,0,0]
        self.arm.set_arm_ik_position(15,5,wait_for_ack=True,timeout=1)
        # self.locate_pet(60,90,45,arm_pos,5, 0.2, 280)
        self.locate_pet_discrete([55,65,75],arm_pos)

        grab_pos = [15+11,-8,0]
        self.grab_pet_direct(grab_pos)
        time.sleep(1)

        self.arm.set_wrist_angle(180)
        self.arm.set_arm_angles(elbow=0,wait_for_ack=True,timeout=0.5)
        self.arm.set_arm_angles(shoulder=100, wait_for_ack=True,timeout=2)
        self.arm.set_turret_angle(90)
        self.arm.set_arm_angles(elbow=20, wait_for_ack=True,timeout=2)
        
        # self.arm.set_wrist_angle_unlock(180)
        # self.arm.set_arm_angles(elbow=0,wait_for_ack=True,timeout=0.5)
        # self.arm.set_turret_angle(90)
        # self.arm.set_arm_angles(shoulder=100, wait_for_ack=True,timeout=1.5)
        # self.arm.set_claw_angle(90)

        input("Enter to start step 3")

        # STEP 3: Move up the ramp until the third curve
        self.motors.set_base_speed(190)
        self.motors.set_min_speed(165)
        self.line_follow_time(1.5, stop=False)

        print("Step 2")      
        self.motors.set_base_speed(210)
        self.motors.set_min_speed(190)

        self.line_follow_time(2, stop=False)        
        print("Step 3")

        self.motors.set_base_speed(190)
        self.motors.set_min_speed(170)

        self.line_follow_until_curve(curve_threshold=7, max_threshold=20, averaging_window=3)
        self.motors.set_motor_speeds_raw(200,200)
        time.sleep(0.5)
        self.motors.stop_motors()

        # STEP 4: Drop first pet into chute

        input("Enter to start step 4")
        
        self.drop_into_chute()

        # STEP 5: Pick up second pet

        input("Enter to start step 5")

        self.arm.set_wrist_angle(0)

        arm_pos = [15,0,0]
        self.arm.set_arm_ik_position(15,5,wait_for_ack=True,timeout=1.5)
        # self.locate_pet(60,90,45,arm_pos,5, 0.2, 280)
        self.locate_pet_discrete([130,115,100],arm_pos)

        grab_pos = [15+11,-6,0]
        self.grab_pet_direct(grab_pos)
        time.sleep(1)

        # STEP 6: Drop second pet into chute (same as step 4)

        input("Enter to start step 6")
        self.drop_into_chute()

        # STEP 7: Line follow for around 1 sec, then pick up 3rd pet, then drop into basket

        input("Enter to start step 7")
        self.motors.set_base_speed(180)
        self.motors.set_min_speed(165)
        self.line_follow_time(1)

        arm_pos = [15,15,0]
        self.arm.set_arm_ik_position(15,20,wait_for_ack=True,timeout=1.5)
        # self.locate_pet(60,90,45,arm_pos,5, 0.2, 280)
        self.locate_pet_discrete([165,150,135,120,105,90],arm_pos)

        grab_pos = [15+11,9,0]
        self.grab_pet_direct(grab_pos)
        time.sleep(1)

        self.drop_into_basket()
        self.neutral_pos()

    def neutral_pos(self):
        self.arm.set_arm_angles(90, None, 120, wait_for_ack=True, timeout=1)
        self.arm.set_arm_angles(90, 150, 120, wait_for_ack=True, timeout=1)
        self.arm.set_arm_angles(90, 150, 165, wait_for_ack=True, timeout=1)

    def drop_into_basket(self):
        self.arm.set_wrist_angle_unlock(180)
        self.arm.set_arm_angles(elbow=0,wait_for_ack=True,timeout=0.5)
        self.arm.set_turret_angle(90)
        self.arm.set_arm_angles(shoulder=100, wait_for_ack=True,timeout=2)
        self.arm.set_claw_angle(90)

    def drop_into_chute(self):
        self.arm.set_arm_angles(90, 90, 0, wait_for_ack=True, timeout=1)
        self.arm.set_wrist_angle(90)
        self.arm.set_turret_angle(180, wait_for_ack=True, timeout=1)
        self.arm.set_wrist_angle(30)
        self.arm.set_arm_angles(elbow=50, wait_for_ack=True, timeout=1.5)
        self.arm.set_claw_angle(90)
        time.sleep(0.25)
        self.arm.set_turret_angle(90)
    
    def line_follow_until_curve(self, curve_threshold=5, max_threshold=40, 
                               averaging_window=10, debug=False, stop=True):
        """
        Follow line until a curve is detected using averaged curvature values.
        
        Args:
            curve_threshold: Average curvature threshold for curve detection (degrees).
                           Higher values = more pronounced curve needed to stop.
                           Typical values: 5-25 degrees
            max_threshold: Maximum curvature value to ignore outliers (degrees).
                          Values above this are considered noise/errors.
                          Typical value: 45-60 degrees
            averaging_window: Number of recent curvature values to average.
                            Default: 10 samples
            debug: Print debug information about curvature detection
        """
        # Start Line Following
        self.motors.start_line_following()
        
        # Initialize curvature history buffer
        curvature_history = []
        curve_detected = False
        
        if debug:
            print(f"🔄 Starting line follow until curve detection:")
            print(f"   Threshold: {curve_threshold}°")
            print(f"   Max threshold: {max_threshold}°")
            print(f"   Averaging window: {averaging_window} samples")
        
        while not curve_detected:
            # Get line points
            points = self.line_manager_1.get_line()
            
            # Update line following if line found
            line_status = self.line_manager_1.get_detection_status()
            if line_status.get('line_found', False) and points:
                self.motors.update_line_following(points)
                
                # Calculate current curvature
                current_curvature = self.motors.detect_curvature(points)
                
                
                # Filter out invalid readings
                if current_curvature is not None and abs(current_curvature) <= max_threshold:
                    print(current_curvature)
                    # Add to history (use absolute value for averaging)
                    curvature_history.append(abs(current_curvature))
                    
                    # Maintain window size
                    if len(curvature_history) > averaging_window:
                        curvature_history.pop(0)
                    
                    # Calculate average if we have enough samples
                    if len(curvature_history) >= min(5, averaging_window):  # Need at least 5 samples
                        avg_curvature = sum(curvature_history) / len(curvature_history)
                        
                        if debug and self.loop_count % 10 == 0:  # Print every 10 loops
                            print(f"   Current: {current_curvature:.1f}°, "
                                  f"Average ({len(curvature_history)} samples): {avg_curvature:.1f}°")
                        
                        # Check if average exceeds threshold
                        if avg_curvature >= curve_threshold:
                            curve_detected = True
                            print(f"🔄 Curve detected! Average curvature: {avg_curvature:.1f}° "
                                  f"(threshold: {curve_threshold}°)")
                            
                            # Show recent history
                            if debug:
                                print(f"   Recent values: {[f'{v:.1f}' for v in curvature_history[-5:]]}")
                    
                elif debug and current_curvature is not None:
                    print(f"   ⚠️ Ignoring outlier: {current_curvature:.1f}° (max: {max_threshold}°)")
            else:
                # No line found - could add line loss handling here
                if debug:
                    print("   ⚠️ No line detected")
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.025)
        
        # Stop line following
        if stop:
            self.motors.stop_line_following()
        
        if debug:
            print(f"✅ Stopped line following after curve detection")

    def line_follow_time(self, duration=1.0, stop=True):
        # Start Line Following

        self.motors.start_line_following()

        start_time = time.time()
        while (time.time()-start_time < duration):
            points = self.line_manager_1.get_line()
            line_status = self.line_manager_1.get_detection_status()
            if (line_status.get('line_found', False)):
                self.motors.update_line_following(points)

        if(stop):
            self.motors.stop_line_following()
    
    def line_follow_until_barrier(self, barrier_threshold=50, consecutive_detections=5, stop=True):
        """
        Follow line until a curve is detected using averaged curvature values.
        
        Args:
            curve_threshold: Average curvature threshold for curve detection (degrees).
                           Higher values = more pronounced curve needed to stop.
                           Typical values: 5-25 degrees
            max_threshold: Maximum curvature value to ignore outliers (degrees).
                          Values above this are considered noise/errors.
                          Typical value: 45-60 degrees
            averaging_window: Number of recent curvature values to average.
                            Default: 10 samples
            debug: Print debug information about curvature detection
        """
        # Start Line Following
        self.motors.start_line_following()
        
        # Initialize curvature history buffer
        barrier_count = 0
        barrier_detected = False
        
        while barrier_count < consecutive_detections:
            # Get line points
            barrier_percentage = self.get_barrier_data()
            if barrier_percentage > barrier_threshold:
                barrier_count += 1
            else: 
                barrier_count = 0
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.025)
        
        # Stop line following
        if stop:
            self.motors.stop_line_following()

    def turn_around(self, turn_right=True, starting_speed=50, min_speed=25, center_target=0.1, after_duration=0):

        if not self.motors or not self.line_manager_1:
            print("❌ Motors or line manager not available")
            return False
        
        print(f"🔄 Starting turn around ({'right' if turn_right else 'left'})")
        
        # Phase 1: Initial turn for 1 second
        print("🔄 Phase 1: Initial turn (1 second)")
        turn_speed = starting_speed if turn_right else -starting_speed
        
        self.motors.set_motor_speeds(turn_speed, -turn_speed)  # Turn right
        
        time.sleep(0.5)
        
        # Phase 2: Look for line and fine-tune centering
        print("🔄 Phase 2: Looking for line and centering")
        
        line_found = False
        max_search_time = 10.0  # Maximum time to search for line
        search_start = time.time()
        
        while time.time() - search_start < max_search_time:
            # Get line detection
            points = self.line_manager_1.get_line()
            line_status = self.line_manager_1.get_detection_status()
            
            if line_status.get('line_found', False) and points:
                line_found = True
                
                # Calculate average center position of detected points
                total_x_percent = sum(point['x_percent'] for point in points)
                avg_center = total_x_percent / len(points)
                
                # Calculate error from center (50%)
                center_error = avg_center - 50.0
                distance_from_center = abs(center_error)
                
                print(f"🎯 Line center: {avg_center:.1f}%, error: {center_error:.1f}%")
                
                # Check if we're close enough to center
                if distance_from_center <= center_target * 100:  # Convert 0.1 to 10%
                    print(f"✅ Line centered! Final position: {avg_center:.1f}%")

                    print(f"Phase 3: Run for {after_duration} seconds")
                    time.sleep(after_duration)

                    self.motors.stop_motors()
                    return True
                
                # Calculate turning speed based on error
                # Larger error = faster turn, smaller error = slower turn
                speed_factor = min(1.0, distance_from_center / 20.0)  # Scale by 20% max error
                current_speed = min_speed + (starting_speed - min_speed) * speed_factor
                
                # Determine turn direction based on error
                if center_error > 0:  # Line is to the right, need to turn right more
                    if turn_right:
                        self.motors.set_motor_speeds(current_speed, -current_speed)
                    else:
                        # We were turning left but need to go right now
                        self.motors.set_motor_speeds(current_speed, -current_speed)
                else:  # Line is to the left, need to turn left more
                    if turn_right:
                        # We were turning right but need to go left now
                        self.motors.set_motor_speeds(-current_speed, current_speed)
                    else:
                        self.motors.set_motor_speeds(-current_speed, current_speed)
                
                print(f"🔄 Adjusting turn: speed={current_speed:.1f}, error={center_error:.1f}%")
                
            else:
                # No line found, continue turning at current speed
                if turn_right:
                    self.motors.set_motor_speeds(starting_speed, -starting_speed)
                else:
                    self.motors.set_motor_speeds(-starting_speed, starting_speed)
                
                if not line_found:
                    print("🔍 Still searching for line...")
            
            time.sleep(0.05)  # Small delay for control loop
        
        # Timeout reached
        print("⚠️ Turn around timeout - line not properly centered")
        self.motors.stop_motors()
        return False

    def locate_pet_discrete(self, angles_to_check, arm_position, attempts=3):
        
        self.arm.set_turret_angle(angles_to_check[0])
        self.arm.set_arm_ik_position(arm_position[0], arm_position[1], wait_for_ack=True, timeout=1.5)
        current_angle = angles_to_check[0]
        result = self.object_detection_manager.run_detection_blocking()

        if not result.found:

            print("Object was not found, trying other angles")
            for i in range(1,len(angles_to_check)):
                self.arm.set_turret_angle(angles_to_check[i])
                current_angle = angles_to_check[0]
                result = self.object_detection_manager.run_detection_blocking()

                if result.found:
                    break

            if not result.found:
                print("Object was not found, we are cooked")

        print("Object found")
        last_error = 0

        for i in range(attempts):
            result = self.object_detection_manager.run_detection_blocking()
            if (result.found):
                print(((result.bbox[0] + result.bbox[2]) / 2))
                center = (result.bbox_percent[0] + result.bbox_percent[2]) / 200
                error = center - 0.5 # Positive Error = Turn right
                last_error = error
                
                angle = self._calculate_angle_correction(result.bbox, 50)
                current_angle += angle
                self.arm.set_turret_angle(current_angle)
                print(f"Object found - error = {error}, percent = {center}, angle = {angle}")
            else:
                print(f"Object not found, adjusting slightly")
                if last_error > 0: # turn right
                    current_angle -= 5
                    self.arm.set_turret_angle(current_angle)
                else:
                    current_angle += 5
                    self.arm.set_turret_angle(current_angle)
            
            time.sleep(0.1)
 
    def _calculate_angle_correction(self, bbox, camera_fov_degrees):
        """
        Calculate angle correction needed to center pet in frame.
        
        Args:
            bbox: Bounding box [x1, y1, x2, y2] from object detection
            camera_fov_degrees: Camera field of view in degrees
            
        Returns:
            float: Angle correction in degrees (+ = turn right, - = turn left)
        """
        # Get actual frame dimensions from camera
        frame_width = 320  # Default fallback
        if self.camera_manager:
            try:
                frame = self.camera_manager.get_frame(2)  # Object detection camera
                if frame is not None:
                    frame_width = frame.shape[1]
            except:
                pass  # Use default if failed
        
        # Calculate pet center X position
        pet_center_x = (bbox[0] + bbox[2]) / 2

        print(f"Center: {pet_center_x}")
        
        # Calculate frame center
        frame_center_x = frame_width / 2

        print(f"Frame Center: {frame_center_x}")
        
        # Calculate pixel offset from center
        pixel_offset = pet_center_x - frame_center_x
        
        # Convert pixel offset to angle
        pixels_per_degree = frame_width / camera_fov_degrees
        angle_correction = pixel_offset / pixels_per_degree
        
        # Fixed: Remove negative sign to correct direction
        # Positive = pet is right of center = turn right
        # Negative = pet is left of center = turn left
        return -angle_correction

    def predict_object_distance(self, bbox, camera_fov_degrees, known_object_width=None, 
                               known_object_height=None, frame_width=None, frame_height=None):
        """
        Predict the distance to an object using its bounding box and camera FOV.
        
        This function uses similar triangles and camera geometry to estimate distance.
        Two methods are available:
        1. Using known real-world object dimensions (more accurate)
        2. Using angular size estimation (rough approximation)
        
        Args:
            bbox: Bounding box [x1, y1, x2, y2] from object detection
            camera_fov_degrees: Camera field of view in degrees (horizontal)
            known_object_width: Real-world width of object in cm (optional, for method 1)
            known_object_height: Real-world height of object in cm (optional, for method 1)
            frame_width: Frame width in pixels (auto-detected if None)
            frame_height: Frame height in pixels (auto-detected if None)
            
        Returns:
            dict: Distance estimation results containing:
                - distance_cm: Estimated distance in centimeters
                - method: Method used ('known_size' or 'angular_estimate')
                - confidence: Confidence level ('high', 'medium', 'low')
                - angular_width_degrees: Angular width of object in degrees
                - angular_height_degrees: Angular height of object in degrees
                - pixel_width: Object width in pixels
                - pixel_height: Object height in pixels
        """
        # Get frame dimensions
        if frame_width is None or frame_height is None:
            try:
                # Try to get from camera manager
                frame = self.camera_manager.get_frame(2)  # Object detection camera
                if frame is not None:
                    h, w = frame.shape[:2]
                    frame_width = frame_width or w
                    frame_height = frame_height or h
                else:
                    # Use object detection processing size as fallback
                    frame_width = frame_width or 320
                    frame_height = frame_height or 320
            except:
                frame_width = frame_width or 320
                frame_height = frame_height or 320
        
        # Calculate object dimensions in pixels
        x1, y1, x2, y2 = bbox
        pixel_width = abs(x2 - x1)
        pixel_height = abs(y2 - y1)
        
        # Calculate angular size of object
        # Angular width = (pixel_width / frame_width) * camera_fov_degrees
        angular_width_degrees = (pixel_width / frame_width) * camera_fov_degrees
        
        # Calculate vertical FOV assuming 4:3 or 16:9 aspect ratio
        aspect_ratio = frame_width / frame_height
        camera_fov_vertical = camera_fov_degrees / aspect_ratio
        angular_height_degrees = (pixel_height / frame_height) * camera_fov_vertical
        
        # Method 1: Use known object dimensions (most accurate)
        if known_object_width is not None:
            # Distance = (real_width * frame_width) / (2 * pixel_width * tan(fov/2))
            import math
            fov_rad = math.radians(camera_fov_degrees)
            
            # Using horizontal dimension
            distance_cm = (known_object_width * frame_width) / (2 * pixel_width * math.tan(fov_rad / 2))
            method = "known_size_width"
            confidence = "high"
            
            # If height is also known, calculate using height and average
            if known_object_height is not None:
                fov_vertical_rad = math.radians(camera_fov_vertical)
                distance_cm_height = (known_object_height * frame_height) / (2 * pixel_height * math.tan(fov_vertical_rad / 2))
                
                # Average the two estimates
                distance_cm = (distance_cm + distance_cm_height) / 2
                method = "known_size_both"
                confidence = "high"
        
        # Method 2: Angular size estimation (rough approximation)
        else:
            # This method assumes typical object sizes and is less accurate
            # Rule of thumb: Most objects are 10-30cm wide when they fill ~10-30% of frame
            
            # Calculate what percentage of frame the object occupies
            width_percentage = (pixel_width / frame_width) * 100
            height_percentage = (pixel_height / frame_height) * 100
            
            # Rough estimation based on common object sizes
            # This is a heuristic and should be calibrated for specific objects
            if width_percentage > 50:
                # Very close - object fills most of frame
                estimated_distance = 20  # 20cm
                confidence = "low"
            elif width_percentage > 25:
                # Close - object fills quarter of frame
                estimated_distance = 50  # 50cm
                confidence = "low"
            elif width_percentage > 10:
                # Medium distance
                estimated_distance = 100  # 1 meter
                confidence = "low"
            elif width_percentage > 5:
                # Far distance
                estimated_distance = 200  # 2 meters
                confidence = "low"
            else:
                # Very far
                estimated_distance = 500  # 5 meters
                confidence = "low"
            
            distance_cm = estimated_distance
            method = "angular_estimate"
        
        # Package results
        result = {
            'distance_cm': round(distance_cm, 1),
            'method': method,
            'confidence': confidence,
            'angular_width_degrees': round(angular_width_degrees, 2),
            'angular_height_degrees': round(angular_height_degrees, 2),
            'pixel_width': pixel_width,
            'pixel_height': pixel_height,
            'frame_width': frame_width,
            'frame_height': frame_height,
            'width_percentage': round((pixel_width / frame_width) * 100, 1),
            'height_percentage': round((pixel_height / frame_height) * 100, 1)
        }
        
        return distance_cm

    def grab_pet_direct(self, arm_position_1):
        self.arm.set_wrist_angle(arm_position_1[2])
        self.arm.set_arm_ik_position(arm_position_1[0], arm_position_1[1], 
                                    wait_for_ack=True, timeout=2.0)
        time.sleep(0.5)

        self.arm.set_claw_angle(0)
    
    def grab_pet_twostep(self, arm_position_1, speed, arm_position_2):
        self.arm.set_wrist_angle(arm_position_1[2])
        self.arm.set_arm_ik_position(arm_position_1[0], arm_position_1[1], 
                                    wait_for_ack=True, timeout=2.0)
        time.sleep(0.5)

        self.arm.set_wrist_angle(arm_position_2[2])
        self.arm.set_arm_ik_position(arm_position_2[0], arm_position_2[1], 
                                    wait_for_ack=True, timeout=1.0)
        
        time.sleep(0.5)

        self.arm.set_claw_angle(0)
        
    def get_barrier_data(self):
        """
        Get current barrier detection data.
        
        Returns:
            float: barrier percentage
        """
        
        try:
            # Get current barrier percentage
            barrier_percentage = self.line_manager_1.get_barrier_percentage()
            
            # Get barrier area coordinates
            area_coords = self.line_manager_1.get_barrier_area_coords()
            
            # Get configuration
            barrier_config = self.line_manager_1.config.get('barrier_detection', {})
            threshold = barrier_config.get('brown_threshold_percent', 60.0)
            enabled = barrier_config.get('enabled', True)
            
            # Determine if barrier is detected
            barrier_detected = enabled and barrier_percentage > threshold
            
            return barrier_percentage
            
        except Exception as e:
            return

    # ========== TEST FUNCTIONS ==========
    
    def test_serial_communication(self):
        """Test basic serial communication with the robot."""
        print("🧪 Testing serial communication...")
        
        if not self.ser or not self.ser.is_open:
            print("❌ Serial port not available")
            return False
        
        # Test basic serial write/read
        test_messages = ["PI:TEST", "STATUS", "PING"]
        
        for msg in test_messages:
            print(f"📤 Sending: {msg}")
            success = self.write_serial(msg)
            if success:
                # Wait a bit for response
                time.sleep(0.5)
                response = self.read_serial(timeout=1.0)
                if response:
                    print(f"📥 Response: {response}")
                else:
                    print("📥 No response received")
            else:
                print("❌ Failed to send message")
            
            time.sleep(0.5)
        
        print("✅ Serial communication test complete")
        return True
    
    def test_motor_functions(self):
        """Test motor control functions."""
        print("🧪 Testing motor functions...")
        
        if not self.motors:
            print("❌ Motor controller not available")
            return False
        
        try:
            print("🚗 Testing motor speed mapping...")
            # Test the motor speed mapping function
            test_speeds = [-100, -50, -1, 0, 1, 50, 100]
            for speed in test_speeds:
                mapped = self.motors.map_speed_to_motor(speed)
                print(f"   Controller speed {speed:4d} → Motor speed {mapped:4d}")
            
            print("\n🚗 Testing basic movements (short duration)...")
            
            # Test forward movement
            print("   Forward for 1 second...")
            self.motors.set_motor_speeds(30, 30)  # Low speed
            time.sleep(1.0)
            self.motors.stop_motors()
            time.sleep(0.5)
            
            # Test backward movement
            print("   Backward for 1 second...")
            self.motors.set_motor_speeds(-30, -30)
            time.sleep(1.0)
            self.motors.stop_motors()
            time.sleep(0.5)
            
            # Test turning
            print("   Turn right for 1 second...")
            self.motors.set_motor_speeds(25, -25)
            time.sleep(1.0)
            self.motors.stop_motors()
            time.sleep(0.5)
            
            # Test turning left
            print("   Turn left for 1 second...")
            self.motors.set_motor_speeds(-25, 25)
            time.sleep(1.0)
            self.motors.stop_motors()
            
            print("✅ Motor function test complete")
            return True
            
        except Exception as e:
            print(f"❌ Motor test error: {e}")
            self.motors.stop_motors()
            return False
    
    def test_arm_functions(self):
        """Test arm control functions."""
        print("🧪 Testing arm functions...")
        
        if not self.arm:
            print("❌ Arm controller not available")
            return False
        
        try:
            print("🦾 Testing turret movement...")
            
            # Test turret positions
            test_angles = [45, 135, 90]  # Center, left, right, center
            
            for angle in test_angles:
                print(f"   Moving turret to {angle}°...")
                success = self.arm.set_turret_angle(angle, wait_for_ack=True, timeout=2.0)
                if success:
                    print(f"   ✅ Turret moved to {angle}°")
                else:
                    print(f"   ❌ Failed to move turret to {angle}°")
                time.sleep(1.0)
            
            print("\n🦾 Testing arm positioning...")
            
            # Test basic arm positions
            test_positions = [
                [15, 10, 0],  # [x, y, wrist_angle]
                [20, 15, 0],
                [15, 10, 0]   # Return to start
            ]
            
            for i, pos in enumerate(test_positions):
                print(f"   Moving arm to position {i+1}: x={pos[0]}, y={pos[1]}, wrist={pos[2]}°")
                
                # Set wrist angle
                wrist_success = self.arm.set_wrist_angle(pos[2])
                
                # Set arm position
                arm_success = self.arm.set_arm_ik_position(pos[0], pos[1], wait_for_ack=True, timeout=3.0)
                
                if arm_success:
                    print(f"   ✅ Arm moved to position {i+1}")
                else:
                    print(f"   ❌ Failed to move arm to position {i+1}")
                
                time.sleep(1.5)
            
            print("\n🦾 Testing claw...")
            
            # Test claw movement
            print("   Opening claw...")
            self.arm.set_claw_angle(80)  # Open position
            time.sleep(1.0)
            
            print("   Closing claw...")
            self.arm.set_claw_angle(40)  # Closed position
            time.sleep(1.0)
            
            print("✅ Arm function test complete")
            return True
            
        except Exception as e:
            print(f"❌ Arm test error: {e}")
            return False

    def _process_line_following(self):
        """Process line following and compute statistics."""
        # Get line detection points for camera 1 only
        points1 = self.line_manager_1.get_line()

        # print(self.motors.detect_curvature(points1))

        # Process camera 1
        if points1:
            distances = []
            for point in points1:
                # Calculate distance from center (50% = center)
                distance_from_center = abs(point['x_percent'] - 50.0)
                distances.append(distance_from_center)
            
            if distances:
                avg_distance = sum(distances) / len(distances)
                self.line_stats['total_distance_1'] += avg_distance
                self.line_stats['measurement_count_1'] += 1
                self.line_stats['last_avg_distance_1'] = avg_distance
    
    def _process_object_detection(self):
        """Process object detection at 1Hz (non-blocking)."""
        current_time = time.time()
        
        # Run detection every 1 second (this is now non-blocking)
        if current_time - self.last_object_detection_time >= 0.01:
            try:
                # This call is now non-blocking - returns immediately and starts background detection
                result = self.object_detection_manager.run_detection()
                self.last_object_detection_time = current_time
                print(f"\n🎯 Object detected: {result.class_name} ({result.confidence:.3f})")
                
            except Exception as e:
                print(f"\n❌ Object detection error: {e}")
                self.last_object_detection_time = current_time
    
    def test_curvature_detection(self, duration=30.0, show_plot=False):
        """
        Test curvature detection for a specified duration without line following.
        Useful for calibrating threshold values.
        
        Args:
            duration: How long to collect curvature data (seconds)
            show_plot: Whether to show a plot of curvature values (requires matplotlib)
        """
        print(f"🧪 Testing curvature detection for {duration} seconds...")
        print("   Move the robot along different line curvatures")
        
        curvature_values = []
        timestamps = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            points = self.line_manager_1.get_line()
            
            if points:
                curvature = self.motors.detect_curvature(points)
                if curvature is not None:
                    current_time = time.time() - start_time
                    curvature_values.append(curvature)
                    timestamps.append(current_time)
                    
                    # Print every 0.5 seconds
                    print(f"   t={current_time:.1f}s: Curvature = {curvature:.1f}°")
            
            time.sleep(0.05)
        
        if curvature_values:
            # Calculate statistics
            abs_values = [abs(v) for v in curvature_values]
            avg_curvature = sum(abs_values) / len(abs_values)
            max_curvature = max(abs_values)
            min_curvature = min(abs_values)
            
            print(f"\n📊 Curvature Statistics:")
            print(f"   Samples collected: {len(curvature_values)}")
            print(f"   Average |curvature|: {avg_curvature:.1f}°")
            print(f"   Max |curvature|: {max_curvature:.1f}°")
            print(f"   Min |curvature|: {min_curvature:.1f}°")
            print(f"\n   Suggested thresholds:")
            print(f"   - Gentle curves: {avg_curvature + 5:.1f}°")
            print(f"   - Sharp curves: {avg_curvature + 10:.1f}°")
            print(f"   - Max threshold: {max_curvature + 10:.1f}°")
            
            if show_plot:
                try:
                    import matplotlib.pyplot as plt
                    plt.figure(figsize=(10, 6))
                    plt.plot(timestamps, curvature_values, 'b-', label='Curvature')
                    plt.axhline(y=avg_curvature, color='g', linestyle='--', label=f'Avg: {avg_curvature:.1f}°')
                    plt.axhline(y=-avg_curvature, color='g', linestyle='--')
                    plt.xlabel('Time (s)')
                    plt.ylabel('Curvature (degrees)')
                    plt.title('Curvature Detection Test')
                    plt.legend()
                    plt.grid(True)
                    plt.show()
                except ImportError:
                    print("   (matplotlib not available for plotting)")
        else:
            print("❌ No curvature data collected")

    # SERIAL STUFF
    def write_serial(self, message: str) -> bool:
        """Write a message to the serial port."""
        if self.ser and self.ser.is_open:
            try:
                # Ensure message ends with newline
                if not message.endswith('\n'):
                    message += '\n'
                
                # Write to serial port
                self.ser.write(message.encode('utf-8'))
                self.ser.flush()
                print(f"📤 Serial TX: {message.strip()}")
                return True
            except Exception as e:
                print(f"❌ Serial write error: {e}")
                return False
        else:
            print("❌ Serial port not available")
            return False
    
    def read_serial(self, timeout: float = 0.1) -> Optional[str]:
        """
        Read a line from the serial port.
        
        Args:
            timeout: Read timeout in seconds
            
        Returns:
            str: Received message (without newline) or None if no data/error
        """
        if self.ser and self.ser.is_open:
            try:
                # Set timeout for this read
                original_timeout = self.ser.timeout
                self.ser.timeout = timeout
                
                # Read line
                line = self.ser.readline()
                
                # Restore original timeout
                self.ser.timeout = original_timeout
                
                if line:
                    message = line.decode('utf-8').strip()
                    if message:  # Only log non-empty messages
                        print(f"📥 Serial RX: {message}")
                        return message
                return None
                
            except Exception as e:
                print(f"❌ Serial read error: {e}")
                return None
        else:
            return None
    
    def clear_serial_buffer(self):
        """
        Clear/flush the serial input buffer to remove any stale data.
        This is useful before waiting for specific responses.
        """
        if self.ser and self.ser.is_open:
            try:
                # Flush input buffer (clear received data waiting to be read)
                self.ser.reset_input_buffer()
                
                # Also read any remaining data with short timeout
                start_time = time.time()
                cleared_count = 0
                while time.time() - start_time < 0.1:  # 100ms max
                    if self.ser.in_waiting > 0:
                        data = self.ser.readline()
                        if data:
                            cleared_count += 1
                    else:
                        break
                        
                if cleared_count > 0:
                    print(f"📥 Cleared {cleared_count} stale serial messages")
                    
            except Exception as e:
                print(f"❌ Error clearing serial buffer: {e}")
    
    def wait_for_serial_message(self, expected_message: str, timeout: float = 5.0, 
                               contains: bool = False, clear_buffer: bool = True) -> bool:
        """
        Wait for a specific message from the serial port.
        
        Args:
            expected_message: Message to wait for
            timeout: Maximum time to wait in seconds
            contains: If True, check if message contains expected_message, 
                     otherwise check for exact match
            clear_buffer: If True, clear serial buffer before waiting
                     
        Returns:
            bool: True if message received, False if timeout
        """
        # Clear buffer first to avoid stale messages
        if clear_buffer:
            self.clear_serial_buffer()
            
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            message = self.read_serial(timeout=0.1)
            if message:
                if contains:
                    if expected_message in message:
                        return True
                else:
                    if message == expected_message:
                        return True
            time.sleep(0.05)  # Small delay to prevent busy waiting
        
        print(f"⚠️ Timeout waiting for serial message: '{expected_message}'")
        return False
    
    def _update_motor_line_following(self):
        """Bridge method: Update motor controller with line following data."""
        try:
            # Get line detection data from vision system
            line_status = self.line_manager_1.get_detection_status()
            points = self.line_manager_1.get_line()
            
            if line_status.get('line_found', False) and points:
                # Pass the line points directly to motor controller
                # The motor controller will handle weighted averaging and PID
                self.motors.update_line_following(points)
            else:
                # No line detected - handle line loss
                if self.motors.line_following_active:
                    print("⚠️ Line lost - stopping motors")
                    self.motors.stop_motors()
                    
        except Exception as e:
            print(f"❌ Error updating motor line following: {e}")
    
    def run(self):
        """Run the robot controller (blocking)."""
        try:
            print("🤖 Robot Controller running...")
            print("Press Ctrl+C to stop")
            
            while self.running:
                # Print status periodically
                if self.loop_count % 50 == 0:  # Every 5 seconds at 10Hz
                    elapsed = time.time() - self.start_time
                    rate = self.loop_count / elapsed if elapsed > 0 else 0
                    
                    status = self.camera_manager.get_system_status()
                    cam1_fps = status['cameras'][1]['fps']
                    cam2_fps = status['cameras'][2]['fps']
                    
                    # Get line following statistics
                    avg_dist_1 = 1
                    last_dist_1 = self.line_stats['last_avg_distance_1']
                    
                    # Get object detection statistics
                    obj_stats = self.object_detection_stats
                    last_detection_ago = (time.time() - obj_stats['last_detection_time']) if obj_stats['last_detection_time'] > 0 else 999
                    
                    # print(f"\r⚡ Control: {rate:.1f}Hz | "
                    #       f"📷 Cam1: {cam1_fps:.1f}fps | "
                    #       f"📷 Cam2: {cam2_fps:.1f}fps | "
                    #       f"📏 Line1: {last_dist_1:.1f}% (avg: {avg_dist_1:.1f}%) | "
                    #       f"🎯 Obj: {obj_stats['total_detections']} ({last_detection_ago:.0f}s ago) | "
                    #       f"🔄 Loops: {self.loop_count}", 
                    #       end='', flush=True)
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n🛑 Shutdown requested")
    
    def stop(self):
        """Stop all robot systems."""
        print("\n🛑 Stopping robot systems...")
        
        # Stop control loop first
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)

        if self.motors:
            self.motors.stop_motors()
        
        # Stop GUI by setting flag (let it clean itself up)
        if self.gui:
            self.gui.stop()
            # Wait for GUI thread to finish naturally
            if self.gui_thread and self.gui_thread.is_alive():
                print("⏳ Waiting for GUI to close...")
                self.gui_thread.join(timeout=5.0)
                if self.gui_thread.is_alive():
                    print("⚠️ GUI thread did not stop cleanly")
        
        # Stop object detection manager
        if self.object_detection_manager:
            self.object_detection_manager.cleanup()
        
        # Close serial port
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("📡 Serial port closed")
        
        # Stop camera manager
        self.camera_manager.stop()
        
        print("✅ All systems stopped")
    
    def send_annotation(self, annotation: 'Annotation', persistent: bool = False):
        """
        Send annotation to GUI if available.
        
        Args:
            annotation: Annotation object to send
            persistent: Whether annotation should persist across frames
        """
        if self.gui:
            self.gui.add_annotation(annotation, persistent)

# Signal handler
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\n🛑 Interrupt received, shutting down...")
    sys.exit(0)

def main():
    """Main entry point."""
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse arguments
    parser = argparse.ArgumentParser(description="Robot Controller")
    parser.add_argument('--config', type=str, default='camera_config.json',
                       help='Camera configuration file')
    parser.add_argument('--no-gui', action='store_true',
                       help='Run without GUI')
    parser.add_argument('--test', action='store_true',
                       help='Add test annotations')
    parser.add_argument('--downsample', type=float, default=None,
                       help='Set downsample factor for all cameras (e.g., 2.0 for half size)')
    parser.add_argument('--console', action='store_true',
                       help='Enable console controls when running without GUI')
    parser.add_argument('--serial-port', type=str, default=None,
                       help='Serial port path (default: None, no serial)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Serial baudrate (default: 115200)')
    args = parser.parse_args()
    
    # Create robot controller
    controller = RobotController(
        config_file=args.config,
        enable_gui=not args.no_gui,
        serial_port=args.serial_port,
        baudrate=args.baudrate
    )
    
    try:
        # Start all systems
        controller.start()
        
        # Set downsampling if requested
        if args.downsample is not None:
            print(f"🔍 Setting downsample factor to {args.downsample}x for all cameras")
            controller.camera_manager.set_downsample_all(args.downsample)
        
        # Add test annotations if requested
        if args.test and not args.no_gui:
            time.sleep(1)  # Give GUI time to start
            # Example of how to send annotations from controller
            controller.send_annotation(
                Text(camera_id=1, x=10, y=30, text="Test Mode Active",
                     color=(255, 255, 0), font_scale=0.7),
                persistent=True
            )
        
        # Run main loop
        controller.run()
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Clean shutdown
        controller.stop()
        print("✅ Robot Controller shutdown complete")

if __name__ == "__main__":
    main()