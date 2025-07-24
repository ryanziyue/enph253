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
        self.camera_manager = CameraManager(config_file)
        
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
        self.arm.set_arm_angles(90, 150, 165)
                
        input("Enter to start")

        # self.motors.start_line_following()
        # time.sleep(0.1)
        # input("Enter to stop")
        # self.motors.stop_line_following()

        # self.motors.set_line_following_base_speed(150)
        # self.line_follow_until_curve(curve_threshold=15, averaging_window=3)
        # time.sleep(1)
        # self.line_follow_time(1)
        # time.sleep(1)
        # self.motors.move_backward(50)
        # time.sleep(0.1)
        # self.turn_around(turn_right=False, starting_speed=70, min_speed=40, after_duration=0.5)
        # self.line_follow_time(1)
        
        # input("Enter to pickup")


        # self.arm.set_wrist_angle(0)

        # self.test_motor_functions()
        # time.sleep(1)
        # self.test_arm_functions()

        # Test angle correction function
        # self.test_angle_correction()

        angles_to_check = [45, 55, 65, 75]
        arm_pos = [15,2,0]
        
        # self.arm.set_turret_angle(45, True, 1)
        
        # # # Use the new smooth detection method
        # self.locate_pet(angles_to_check, arm_pos, max_search_iterations=1)
        
        # # # Alternative: Use sweep with smooth detection
        # # # self.locate_pet_sweep(120,160,arm_pos)

        # self.locate_pet_sweep(45,90,arm_pos)

        # input("Enter to continue")

        # pickup_pos = [30,-8,-10]
        # drop_pos = [-10,25,-90]
        # self.grab_pet_direct(pickup_pos, drop_pos)
        # self.arm.set_arm_angles(turret=90, shoulder=110, elbow=0, wait_for_ack=True, timeout=3.0)
        # self.arm.set_wrist_angle_unlock(180)
        # time.sleep(3)
        # self.arm.set_claw_angle(90)

        # self.motors.start_line_following()
        # self.line_follow_until_curve(curve_threshold=5, averaging_window=4)
        # self.motors.stop_line_following()
        # self.motors.stop_motors()
        # print("Curve reached")
        # input("Enter to continue line following")
        # self.motors.start_line_following()

        # self.test_curvature_detection(duration=120)

        # self.arm.set_arm_ik_position(17,3,wait_for_ack=True,timeout=1.5)
        
        while self.running:
            try:
                # Get current time
                current_time = time.time()
                elapsed = current_time - self.start_time

                # print("turret")
                # self.arm.wait_for_limit_switch(timeout=5)
                # print("run loop")
                # time.sleep(1)
                
                # Process line following for camera 1 only
                if self.line_manager_1:
                    self._process_line_following()

                # input("Press to start")

                # self.line_follow_until_curve(curve_threshold=25, averaging_window=3)

                # turret_pos = input("Enter turret position")
                # try:
                #     self.arm.set_turret_angle(int(turret_pos), True, 1)
                # except:
                #     pass
                
                # result = self.object_detection_manager.run_detection_blocking()

                # if (not result.found):
                #     print("Not found, try again")
                #     continue

                # for iteration in range(2):
                #     print(f"🔄 Centering iteration {iteration + 1}/{2}")
            
                #     # Calculate angle correction needed
                #     angle_correction = self._calculate_angle_correction(
                #         result.bbox, 40
                #     )
                    
                #     print(f"   Pet center offset: {angle_correction:.1f}°")
                    
                #     # Check if already centered
                #     if abs(angle_correction) <= 5:
                #         print(f"✅ Pet centered! Final angle: {current_angle:.1f}°")
                #         return True, current_angle, result
                    
                #     # Apply correction
                #     new_angle = current_angle + angle_correction
                    
                #     # Clamp angle to reasonable range (adjust as needed for your turret)
                #     new_angle = max(0, min(180, new_angle))
                    
                #     print(f"   Adjusting turret: {current_angle:.1f}° → {new_angle:.1f}°")

                #     input("Enter to move turret")
                    
                #     success = self.arm.set_turret_angle(new_angle, wait_for_ack=True, timeout=2.0)
                    
                #     current_angle = new_angle


                # Test blocking object detection
                # if self.object_detection_manager:
                #     start_time = time.time()
                #     result = self.object_detection_manager.run_detection_blocking(timeout=1.0)
                #     detection_time = time.time() - start_time
                    
                #     # Print detailed results every few loops
                #     if self.loop_count % 10 == 0:  # Every 1 second at 10Hz
                #         print(f"🎯 Blocking Detection Test:")
                #         print(f"   Time taken: {detection_time:.3f}s")
                #         print(f"   Found: {result.found}")
                #         print(f"   Confidence: {result.confidence:.3f}")
                #         print(f"   Class: {result.class_name}")
                #         print(f"   Timestamp: {result.timestamp:.3f}")
                #         if result.found:
                #             print(f"   BBox: {result.bbox}")
                #         print(f"   Loop: {self.loop_count}")
                        
                #         # Check if GUI is getting updated
                #         if self.gui and result.found:
                #             # Send detection annotation to GUI for camera 1
                #             if result.bbox:
                #                 x1, y1, x2, y2 = result.bbox
                #                 bbox_annotation = BoundingBox(
                #                     camera_id=1,  # Using camera 1 now
                #                     x=x1, y=y1, 
                #                     width=x2-x1, height=y2-y1,
                #                     label=f"{result.class_name} ({result.confidence:.2f})",
                #                     color=(0, 255, 0),
                #                     thickness=2
                #                 )
                #                 self.gui.add_annotation(bbox_annotation, persistent=False)
                #                 print(f"   📺 Sent annotation to GUI")
                        
                #         print("-" * 40)
                
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
                time.sleep(0.1)
                
            except Exception as e:
                print(f"❌ Control loop error: {e}")
                time.sleep(0.1)
        
        print("🔄 Control loop stopped")
        
    def line_follow_until_curve(self, curve_threshold=5, max_threshold=40, 
                               averaging_window=10, debug=False):
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
        self.motors.stop_line_following()
        
        if debug:
            print(f"✅ Stopped line following after curve detection")

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
                    if len(curvature_values) % 10 == 0:
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

    def line_follow_time(self, duration=1.0):
        # Start Line Following

        self.motors.start_line_following()

        start_time = time.time()
        while (time.time()-start_time < duration):
            points = self.line_manager_1.get_line()
            line_status = self.line_manager_1.get_detection_status()
            if (line_status.get('line_found', False)):
                self.motors.update_line_following(points)

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

    def locate_pet(self, angles_to_check, arm_position, camera_fov_degrees=40, 
                   max_search_iterations=3, centering_tolerance=5.0):
        """
        Locate and center pet using turret rotation and object detection.
        
        Args:
            angles_to_check: List of turret angles to check for pet detection
            arm_position: 3-element array [x-pos, y-pos, wrist angle]
            camera_fov_degrees: Camera field of view in degrees (default: 120)
            max_search_iterations: Maximum iterations for fine-tuning centering
            centering_tolerance: Angle tolerance for "centered" (degrees)
            
        Returns:
            tuple: (success: bool, final_angle: float, detection_result)
        """
        if not self.arm or not self.object_detection_manager:
            print("❌ Arm or object detection manager not available")
            return False, 0.0, None
        
        print("🔍 Starting pet location sequence...")
        
        # Step 1: Move arm to viewing position
        print(f"🦾 Moving arm to viewing position: {arm_position}")
        self.arm.set_wrist_angle(arm_position[2])
        self.arm.set_arm_ik_position(arm_position[0], arm_position[1], 
                                    wait_for_ack=True, timeout=2.0)

        time.sleep(0.5)  # Allow arm to stabilize
        
        # Step 2: Scan through angles to find pet
        print(f"🔄 Scanning angles: {angles_to_check}")
        pet_found = False
        initial_detection_angle = 0.0
        result = None
        
        for angle in angles_to_check:
            print(f"🔍 Checking angle {angle}°...")
            
            # Move turret to angle with proper synchronization
            success = self.arm.set_turret_angle(angle, wait_for_ack=True, timeout=2.0)
            
            # Run object detection multiple times to ensure fresh results
            print(f"   Running object detection...")
            
            for attempt in range(3):  # Try 3 times
                result = self.object_detection_manager.run_detection_blocking()
                print(f"   Attempt {attempt+1}: Found={result.found}, Confidence={result.confidence:.3f}")
                if result.found:
                    break
                time.sleep(0.2)  # Small delay between attempts
            
            # Use the best detection result            
            print(f"   Result: Found={result.found}, Confidence={result.confidence:.3f}")
            
            if result.found and result.confidence > 0.1:
                print(f"🎯 Pet found at angle {angle}°! Confidence: {result.confidence:.3f}")
                print(f"   BBox: {result.bbox}")
                print(f"   Class: {result.class_name}")
                pet_found = True
                initial_detection_angle = angle
                break
            else:
                print(f"   No pet detected at {angle}° (best confidence: {result.confidence:.3f})")

            time.sleep(1)
        
        if not pet_found:
            print("😞 Pet not found in any of the search angles")
            return False, 0.0, None
        
        # Step 3: Fine-tune centering
        print("🎯 Fine-tuning pet centering...")
        current_angle = initial_detection_angle
        
        for iteration in range(max_search_iterations):
            print(f"🔄 Centering iteration {iteration + 1}/{max_search_iterations}")
    
            # Calculate angle correction needed
            angle_correction = self._calculate_angle_correction(
                result.bbox, camera_fov_degrees
            )
            
            print(f"   Pet center offset: {angle_correction:.1f}°")
            
            # Check if already centered
            if abs(angle_correction) <= centering_tolerance:
                print(f"✅ Pet centered! Final angle: {current_angle:.1f}°")
                return True, current_angle, result
            
            # Apply correction
            new_angle = current_angle + angle_correction
            
            # Clamp angle to reasonable range (adjust as needed for your turret)
            new_angle = max(0, min(180, new_angle))
            
            print(f"   Adjusting turret: {current_angle:.1f}° → {new_angle:.1f}°")
            
            success = self.arm.set_turret_angle(new_angle, wait_for_ack=True, timeout=2.0)
            if not success:
                print("⚠️ Failed to synchronize turret angle")
                break
            
            current_angle = new_angle
            # No additional wait needed - synchronization handles timing
        
        # Final check
        result = self.object_detection_manager.run_detection()
        if result.found:
            final_correction = self._calculate_angle_correction(result.bbox, camera_fov_degrees)
            print(f"🎯 Centering complete. Final offset: {final_correction:.1f}°")
            return True, current_angle, result
        else:
            print("⚠️ Pet lost during centering process")
            return False, current_angle, None
    
    def locate_pet_sweep(self, start_angle, end_angle, arm_position, 
                        centering_tolerance=0.1):
        """
        Simplified pet location using continuous detection and proportional centering.
        Continuously runs detection and adjusts turret until object is centered.
        
        Args:
            start_angle: Starting angle for initial search
            end_angle: Ending angle for initial search  
            arm_position: 3-element array [x-pos, y-pos, wrist angle]
            centering_tolerance: Distance tolerance as fraction of frame width (default: 0.05 = 5%)
            
        Returns:
            tuple: (success: bool, final_angle: float, detection_result)
        """
        if not self.arm or not self.object_detection_manager:
            print("❌ Arm or object detection manager not available")
            return False, 0.0, None
        
        print("🔍 Starting simplified pet location with continuous detection...")
        
        # Move arm to viewing position
        print(f"🦾 Moving arm to viewing position: {arm_position}")
        self.arm.set_wrist_angle(arm_position[2])
        self.arm.set_arm_ik_position(arm_position[0], arm_position[1], 
                                     wait_for_ack=True, timeout=2.0)
        
        # Initial sweep to find pet
        print(f"🔄 Initial sweep from {start_angle}° to {end_angle}°...")
        
        sweep_direction = 1 if end_angle > start_angle else -1
        current_angle = start_angle
        search_step = 5.0  # Fixed step size for initial search
        
        # Move to start position
        self.arm.set_turret_angle(start_angle, wait_for_ack=True, timeout=2.0)
        time.sleep(0.5)
        
        # Find pet with simple sweep
        pet_found = False
        while not pet_found and ((sweep_direction > 0 and current_angle <= end_angle) or 
                                (sweep_direction < 0 and current_angle >= end_angle)):
            
            result = self.object_detection_manager.run_detection_blocking()
            
            if result.found and result.confidence > 0.1:
                print(f"🎯 Pet found at angle {current_angle:.1f}°! Confidence: {result.confidence:.3f}")
                pet_found = True
                break
            
            # Move to next search position
            current_angle += sweep_direction * search_step
            self.arm.set_turret_angle(current_angle, wait_for_ack=False)
            time.sleep(0.3)
            print(f"   Searching... {current_angle:.1f}°")
        
        if not pet_found:
            print("😞 Pet not found during initial sweep")
            return False, current_angle, None
        
        # Continuous centering with proportional control
        print("🎯 Starting proportional centering...")
        
        # Proportional control parameters
        max_turn_speed = 5.0  # Maximum degrees per step
        min_turn_speed = 2.0   # Minimum degrees per step
        kp = 0.5               # Proportional gain (adjusted for pixel-based control)
        
        last_detection_time = time.time()
        max_centering_time = 6.0
        centering_start_time = time.time()
        
        while time.time() - centering_start_time < max_centering_time:
            result = self.object_detection_manager.run_detection_blocking()
            
            if result.found and result.confidence > 0.1:
                last_detection_time = time.time()
                
                # Get frame dimensions
                frame = self.camera_manager.get_frame(self.object_detection_manager.camera_id)
                if frame is None:
                    continue
                frame_width = 320
                
                # Calculate pet center and distance from frame center
                pet_center_x = (result.bbox[0] + result.bbox[2]) / 2
                frame_center_x = frame_width / 2
                pixel_offset = pet_center_x - frame_center_x

                print(f"Pet Center: {pet_center_x}")
                
                # Convert to fraction of frame width
                offset_fraction = pixel_offset / frame_width
                
                # Check if centered (within tolerance)
                if abs(offset_fraction) <= centering_tolerance:
                    print(f"✅ Pet centered! Final angle: {current_angle:.1f}° "
                          f"(offset: {offset_fraction*100:.1f}% of frame)")
                    return True, current_angle, result
                
                # Proportional control based on pixel distance
                # Scale the error by frame width to get a reasonable turn speed
                error_magnitude = abs(offset_fraction) * 50  # Scale factor for degrees
                turn_speed = max(min_turn_speed, min(max_turn_speed, error_magnitude * kp))
                
                # Apply correction
                if offset_fraction > 0:
                    # Pet is to the right, turn right
                    new_angle = current_angle - turn_speed
                    direction = "RIGHT"
                else:
                    # Pet is to the left, turn left
                    new_angle = current_angle + turn_speed
                    direction = "LEFT"
                
                # Clamp angle to reasonable range
                new_angle = max(0, min(180, new_angle))
                
                print(f"🎯 Pet offset: {offset_fraction*100:.1f}% of frame → {direction} {turn_speed:.1f}° "
                      f"({current_angle:.1f}° → {new_angle:.1f}°)")
                
                # Move turret smoothly
                self.arm.set_turret_angle(new_angle, wait_for_ack=False)
                current_angle = new_angle
                
            else:
                # Lost pet - simple recovery
                if time.time() - last_detection_time > 2.0:
                    print("⚠️ Lost pet - minor search...")
                    # Small oscillation to find pet again
                    search_offset = 3.0 if (time.time() % 2) < 1 else -3.0
                    search_angle = max(0, min(180, current_angle + search_offset))
                    self.arm.set_turret_angle(search_angle, wait_for_ack=False)
                    current_angle = search_angle
            
            time.sleep(0.1)  # Control loop rate
        
        # Return final result
        result = self.object_detection_manager.run_detection()
        if result.found:
            frame = self.camera_manager.get_frame(self.object_detection_manager.camera_id)
            if frame is not None:
                frame_width = frame.shape[1]
                pet_center_x = (result.bbox[0] + result.bbox[2]) / 2
                frame_center_x = frame_width / 2
                final_offset = (pet_center_x - frame_center_x) / frame_width
                print(f"🎯 Centering complete. Final offset: {final_offset*100:.1f}% of frame")
            return True, current_angle, result
        else:
            print("⚠️ Centering timeout")
            return False, current_angle, None
    
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
        frame_width = 640  # Default fallback
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

    def grab_pet_direct(self, arm_position_1, arm_position_2):
        self.arm.set_wrist_angle(arm_position_1[2])
        self.arm.set_arm_ik_position(arm_position_1[0], arm_position_1[1], 
                                    wait_for_ack=True, timeout=2.0)
        time.sleep(0.5)

        self.arm.set_claw_angle(0)

        time.sleep(0.5)
        
    def grab_pet(self, arm_position_1, speed, arm_position_2):
        self.arm.set_wrist_angle(arm_position_1[2])
        self.arm.set_arm_ik_position(arm_position_1[0], arm_position_1[1], 
                                    wait_for_ack=True, timeout=3.0)
        time.sleep(0.5)

        self.arm.set_arm_ik_velocity(speed, 0, wait_for_ack=False)
        self.arm.wait_for_limit_switch(timeout=5)

        time.sleep(0.5)

        self.arm.set_claw_angle(40)

        time.sleep(0.5)

        self.arm.set_wrist_angle(arm_position_2[2])
        self.arm.set_arm_ik_position(arm_position_2[0], arm_position_2[1], 
                                    wait_for_ack=True, timeout=2.0)
    
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
    
    def test_angle_correction(self):
        """
        Test the _calculate_angle_correction function with different bounding box values.
        This lets you verify the angle calculation logic independently.
        """
        print("🧪 Testing Angle Correction Function")
        print("=" * 50)
        
        # Get current camera frame dimensions from camera manager if available
        actual_frame_width = 640  # Default
        if self.camera_manager:
            try:
                # Try to get actual frame dimensions
                frame = self.camera_manager.get_frame(2)  # Object detection camera
                if frame is not None:
                    actual_frame_width = frame.shape[1]
                    print(f"📐 Actual frame width: {actual_frame_width} pixels")
            except:
                pass
        
        # Default test parameters
        default_fov = 70  # degrees
        
        print(f"📐 Current settings:")
        print(f"   Frame width: 320 pixels (hardcoded in function)")
        print(f"   Actual frame width: {actual_frame_width} pixels")
        print(f"   Camera FOV: {default_fov}° (default)")
        print(f"   Frame center: {320/2} pixels")
        
        print(f"\n🧪 Test Cases:")
        print(f"   Bbox format: [x1, y1, x2, y2]")
        print(f"   Positive angle = turn right, Negative = turn left")
        
        # Test cases: [x1, y1, x2, y2, description]
        test_cases = [
            # [x1, y1, x2, y2, description]
            [140, 100, 180, 150, "Centered object (should be ~0°)"],
            [200, 100, 240, 150, "Object to the right"],
            [80, 100, 120, 150, "Object to the left"],
            [0, 100, 40, 150, "Object far left"],
            [280, 100, 320, 150, "Object far right"],
            [100, 100, 220, 150, "Wide object centered"],
            [240, 100, 280, 150, "Small object right side"],
            [40, 100, 80, 150, "Small object left side"],
        ]
        
        print(f"\n📊 Test Results:")
        print(f"{'Bbox':<25} {'Pet Center':<12} {'Frame Center':<13} {'Offset':<8} {'Angle':<8} {'Description'}")
        print("-" * 90)
        
        for i, (x1, y1, x2, y2, description) in enumerate(test_cases):
            bbox = [x1, y1, x2, y2]
            
            # Calculate using the actual function
            angle_correction = self._calculate_angle_correction(bbox, default_fov)
            
            # Calculate components manually for display
            pet_center_x = (x1 + x2) / 2
            frame_center_x = 320 / 2
            pixel_offset = pet_center_x - frame_center_x
            
            print(f"{str(bbox):<25} {pet_center_x:<12.1f} {frame_center_x:<13.1f} {pixel_offset:<8.1f} {angle_correction:<8.1f} {description}")
        
        print(f"\n🔍 Interactive Test:")
        while True:
            try:
                print(f"\nEnter bounding box coordinates (or 'q' to quit):")
                user_input = input("Format: x1,y1,x2,y2 (e.g., 140,100,180,150): ").strip()
                
                if user_input.lower() == 'q':
                    break
                
                # Parse input
                coords = [float(x.strip()) for x in user_input.split(',')]
                if len(coords) != 4:
                    print("❌ Please provide exactly 4 coordinates")
                    continue
                
                x1, y1, x2, y2 = coords
                bbox = [x1, y1, x2, y2]
                
                # Optional: allow custom FOV
                fov_input = input(f"Camera FOV in degrees (default {default_fov}): ").strip()
                fov = float(fov_input) if fov_input else default_fov
                
                # Calculate angle correction
                angle_correction = self._calculate_angle_correction(bbox, fov)
                
                # Show detailed calculation
                pet_center_x = (x1 + x2) / 2
                frame_center_x = 320 / 2
                pixel_offset = pet_center_x - frame_center_x
                pixels_per_degree = 320 / fov
                
                print(f"\n📊 Calculation Details:")
                print(f"   Pet center X: {pet_center_x:.1f} pixels")
                print(f"   Frame center X: {frame_center_x:.1f} pixels")
                print(f"   Pixel offset: {pixel_offset:.1f} pixels")
                print(f"   Pixels per degree: {pixels_per_degree:.2f}")
                print(f"   Raw angle: {pixel_offset / pixels_per_degree:.2f}°")
                print(f"   Final angle correction: {angle_correction:.2f}°")
                print(f"   Direction: {'Turn RIGHT' if angle_correction > 0 else 'Turn LEFT' if angle_correction < 0 else 'Already centered'}")
                
                # Show what this means for turret movement
                print(f"\n🔄 Turret Movement:")
                if abs(angle_correction) < 1.0:
                    print(f"   ✅ Pet is well centered (±1°)")
                elif abs(angle_correction) < 5.0:
                    print(f"   ⚠️ Small adjustment needed: {abs(angle_correction):.1f}°")
                else:
                    print(f"   🔄 Large adjustment needed: {abs(angle_correction):.1f}°")
                
            except ValueError:
                print("❌ Invalid input format. Use: x1,y1,x2,y2")
            except Exception as e:
                print(f"❌ Error: {e}")
        
        print(f"\n✅ Angle correction test complete")
  
    def debug_object_detection_initialization(self):
        """Debug object detection manager initialization."""
        print("🔍 DEBUG: Object Detection Initialization")
        print("=" * 60)
        
        if not self.object_detection_manager:
            print("❌ Object detection manager is None!")
            return
        
        # Check basic initialization
        print(f"📊 Basic Status:")
        print(f"   Manager exists: ✅")
        print(f"   Camera ID: {self.object_detection_manager.camera_id}")
        
        # Check model loading
        print(f"\n🤖 Model Status:")
        print(f"   Model loaded: {self.object_detection_manager.model_loaded}")
        
        if hasattr(self.object_detection_manager, 'model'):
            print(f"   Model object exists: {self.object_detection_manager.model is not None}")
        else:
            print(f"   Model attribute missing: ❌")
        
        # Check YOLO availability
        try:
            from ultralytics import YOLOWorld
            print(f"   YOLO available: ✅")
        except ImportError as e:
            print(f"   YOLO available: ❌ ({e})")
        
        # Check configuration
        print(f"\n⚙️ Configuration:")
        if hasattr(self.object_detection_manager, 'config'):
            config = self.object_detection_manager.config
            print(f"   Model file: {config.get('model', {}).get('model_file', 'NOT SET')}")
            print(f"   Classes: {config.get('model', {}).get('classes', 'NOT SET')}")
            print(f"   Confidence threshold: {config.get('model', {}).get('confidence_threshold', 'NOT SET')}")
            print(f"   Crop settings: {config.get('crop', 'NOT SET')}")
        else:
            print(f"   Config missing: ❌")
        
        # Check camera connectivity
        print(f"\n📷 Camera Status:")
        frame = self.camera_manager.get_frame(self.object_detection_manager.camera_id)
        if frame is not None:
            height, width = frame.shape[:2]
            print(f"   Camera {self.object_detection_manager.camera_id}: ✅ ({width}x{height})")
        else:
            print(f"   Camera {self.object_detection_manager.camera_id}: ❌ No frame")
        
        # Test detection status
        print(f"\n🎯 Detection Status:")
        try:
            status = self.object_detection_manager.get_detection_status()
            print(f"   Last detection found: {status.get('detection_found', 'N/A')}")
            print(f"   Detection count: {status.get('detection_count', 'N/A')}")
            print(f"   Last detection time: {status.get('last_detection_time', 'N/A')}")
            
            if 'last_detection' in status:
                last = status['last_detection']
                print(f"   Last result: found={last.found}, conf={last.confidence:.3f}, class='{last.class_name}'")
        except Exception as e:
            print(f"   Status check failed: ❌ ({e})")
        
        # Test model file existence
        print(f"\n📁 File System Check:")
        try:
            model_file = self.object_detection_manager.config['model']['model_file']
            import os
            if os.path.exists(model_file):
                size = os.path.getsize(model_file) / (1024*1024)  # MB
                print(f"   Model file exists: ✅ ({size:.1f} MB)")
            else:
                print(f"   Model file missing: ❌ ({model_file})")
                # Check current directory
                print(f"   Current directory: {os.getcwd()}")
                print(f"   Files in current dir: {os.listdir('.')[:10]}...")  # First 10 files
        except Exception as e:
            print(f"   File check failed: ❌ ({e})")
        
        # Test basic detection call
        print(f"\n🧪 Detection Test:")
        try:
            print("   Testing run_detection()...")
            result = self.object_detection_manager.run_detection()
            print(f"   Result: found={result.found}, conf={result.confidence:.3f}, class='{result.class_name}'")
            print(f"   Timestamp: {result.timestamp}")
            
            print("   Testing run_detection_blocking()...")
            start_time = time.time()
            result_blocking = self.object_detection_manager.run_detection_blocking(timeout=2.0)
            duration = time.time() - start_time
            print(f"   Blocking result: found={result_blocking.found}, conf={result_blocking.confidence:.3f}")
            print(f"   Duration: {duration:.3f}s")
            
        except Exception as e:
            print(f"   Detection test failed: ❌ ({e})")
            import traceback
            traceback.print_exc()
        
        print("=" * 60)
  
    # UNUSED
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
    
    def get_barrier_data(self):
        """
        Get current barrier detection data.
        
        Returns:
            dict: Barrier detection data containing:
                - barrier_percentage: Current brown percentage in detection area (0-100)
                - barrier_detected: Boolean indicating if barrier is detected
                - area_coords: Tuple of (y1, y2, x1, x2) detection area coordinates
                - threshold: Brown threshold percentage for barrier detection
                - enabled: Whether barrier detection is enabled
        """
        if not self.line_manager_1:
            return {
                'barrier_percentage': 0.0,
                'barrier_detected': False,
                'area_coords': None,
                'threshold': 0.0,
                'enabled': False,
                'error': 'Line manager not initialized'
            }
        
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
            
            return {
                'barrier_percentage': barrier_percentage,
                'barrier_detected': barrier_detected,
                'area_coords': area_coords,
                'threshold': threshold,
                'enabled': enabled,
                'area_width_percent': barrier_config.get('area_width_percent', 40.0),
                'area_height_percent': barrier_config.get('area_height_percent', 20.0),
                'center_x_percent': barrier_config.get('center_x_percent', 50.0),
                'bottom_offset_percent': barrier_config.get('bottom_offset_percent', 5.0)
            }
            
        except Exception as e:
            return {
                'barrier_percentage': 0.0,
                'barrier_detected': False,
                'area_coords': None,
                'threshold': 0.0,
                'enabled': False,
                'error': f'Error retrieving barrier data: {e}'
            }
    
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
    
    def handle_keyboard_controls(self):
        """Handle keyboard controls when running without GUI."""
        print("\n📋 Keyboard Controls:")
        print("  R - Reset adaptive centers")
        print("  T - Reset distance statistics")
        print("  1 - Toggle camera 1 line following")
        print("  2 - Toggle camera 2 line following")
        print("  O - Show object detection status")
        print("  D - Launch object detection GUI")
        print("  S - Show serial status")
        print("  M - Send manual serial message")
        print("  A - Show arm status")
        print("  H - Home arm") 
        print("  C - Open/close claw")
        print("  G - Grab object sequence")
        print("  J - Set arm position (x,y)")
        print("  T - Set turret angle")
        print("  K - Set wrist angle")
        print("  Y - Set claw position (0-100)")
        print("  Z - Stop arm movement")
        print("  F - Start/stop line following")
        print("  E - Emergency stop motors")
        print("  W/S - Move forward/backward")  
        print("  Left/Right - Turn left/right")
        print("  P - Show motor PID parameters")
        print("  U - Update PID parameters")
        print("  V - Set motor speeds")
        print("  L - Switch line following mode")
        print("  N - Navigate to heading")
        print("  B - Test barrier handling")
        print("  X - Show motor status")
        print("  ! - Toggle motor debug mode")
        print("  Q - Quit")
        
        while self.running:
            try:
                # Simple keyboard input (requires Enter)
                user_input = input().lower().strip()
                
                if user_input == 'q':
                    self.running = False
                    break
                elif user_input == 'r':
                    if self.line_manager_1:
                        self.line_manager_1.reset_adaptive_center()
                        print("🎯 Adaptive center reset for camera 1")
                elif user_input == 't':
                    self.reset_distance_stats()
                elif user_input == '1':
                    if self.line_manager_1:
                        self.line_manager_1.adaptive_center_enabled = not self.line_manager_1.adaptive_center_enabled
                        status = "enabled" if self.line_manager_1.adaptive_center_enabled else "disabled"
                        print(f"📏 Camera 1 line following {status}")
                elif user_input == '2':
                    if self.object_detection_manager:
                        print(f"📷 Camera 2 is used for object detection, not line following")
                elif user_input == 'o':
                    if self.object_detection_manager:
                        obj_status = self.object_detection_manager.get_detection_status()
                        print(f"\n🎯 Object Detection Status:")
                        print(f"  Model Loaded: {obj_status['model_loaded']}")
                        print(f"  Camera: {obj_status['camera_id']}")
                        print(f"  Total Detections: {obj_status['detection_count']}")
                        print(f"  Last Detection: {obj_status['last_detection'].class_name if obj_status['last_detection'].found else 'None'}")
                        if obj_status['last_detection'].found:
                            print(f"  Confidence: {obj_status['last_detection'].confidence:.3f}")
                            print(f"  Bbox: {obj_status['last_detection'].bbox}")
                        crop = obj_status['crop_settings']
                        print(f"  Crop: T:{crop['crop_top']} B:{crop['crop_bottom']} L:{crop['crop_left']} R:{crop['crop_right']}")
                elif user_input == 'd':
                    print("🎯 Launching object detection GUI...")
                    try:
                        import subprocess
                        subprocess.Popen(['python3', 'object_detection_ui.py', '--camera', str(self.object_detection_manager.camera_id)])
                        print("✅ Object detection GUI launched in separate process")
                    except Exception as e:
                        print(f"❌ Failed to launch GUI: {e}")
                elif user_input == 's':
                    if self.ser and self.ser.is_open:
                        print(f"\n📡 Serial Status:")
                        print(f"  Port: {self.serial_port}")
                        print(f"  Baudrate: {self.baudrate}")
                        print(f"  Open: {self.ser.is_open}")
                    else:
                        print("❌ Serial port not available ._.")
                elif user_input == 'm':
                    if self.ser and self.ser.is_open:
                        test_message = input("Enter message to send: ")
                        if test_message.strip():
                            self.write_serial(test_message.strip())
                    else:
                        print("❌ Serial port not available ._.")
                elif user_input == 'a':
                    if self.arm:
                        status = self.arm.get_arm_status()
                        print(f"\n🦾 Arm Status:")
                        print(f"  Turret: {status['turret_angle']}°")
                        print(f"  Position: {status['position']}")
                        print(f"  Wrist: {status['wrist_angle']}°")
                        print(f"  Claw: {status['claw_state']} ({status['claw_position']}%)")
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'h':
                    if self.arm:
                        self.arm.home_arm()
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'c':
                    if self.arm:
                        if self.arm.claw_state.value == 'open':
                            self.arm.close_claw()
                        else:
                            self.arm.open_claw()
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'g':
                    if self.arm:
                        try:
                            x = float(input("Enter object X position: "))
                            y = float(input("Enter object Y position: "))
                            success = self.arm.grab_object(x, y)
                            print(f"🦾 Grab sequence {'completed' if success else 'failed'}")
                        except ValueError:
                            print("❌ Invalid position input")
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'j':
                    if self.arm:
                        try:
                            x = float(input(f"Enter X position (current: {self.arm.arm_position['x']}): "))
                            y = float(input(f"Enter Y position (current: {self.arm.arm_position['y']}): "))
                            speed = int(input("Enter movement speed (0-100, default 50): ") or "50")
                            success = self.arm.set_arm_position(x, y, speed)
                            if success:
                                print(f"🦾 Moving arm to ({x}, {y})")
                        except ValueError:
                            print("❌ Invalid input")
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'k':
                    if self.arm:
                        try:
                            angle = float(input(f"Enter wrist angle (0-180°, current: {self.arm.wrist_angle}°): "))
                            success = self.arm.set_wrist_angle(angle)
                            if success:
                                print(f"🦾 Wrist angle set to {angle}°")
                        except ValueError:
                            print("❌ Invalid angle input")
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'y':
                    if self.arm:
                        try:
                            position = int(input(f"Enter claw position (0=closed, 100=open, current: {self.arm.claw_position}): "))
                            speed = int(input("Enter speed (0-100, default 50): ") or "50")
                            success = self.arm.set_claw_position(position, speed)
                            if success:
                                print(f"🦾 Claw position set to {position}%")
                        except ValueError:
                            print("❌ Invalid input")
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'z':
                    if self.arm:
                        success = self.arm.stop_arm()
                        if success:
                            print("🛑 Arm movement stopped")
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 't':
                    if self.arm:
                        try:
                            angle = float(input(f"Enter turret angle (0-180°, current: {self.arm.turret_angle}°): "))
                            success = self.arm.set_turret_angle(angle)
                            if success:
                                print(f"🔄 Turret angle set to {angle}°")
                        except ValueError:
                            print("❌ Invalid angle input")
                    else:
                        print("❌ Arm controller not available")
                elif user_input == 'f':
                    if self.motors:
                        if self.motors.line_following_active:
                            self.stop_line_following()
                            print("📏 Line following stopped")
                        else:
                            self.start_line_following('camera')
                            print("📏 Line following started (camera mode)")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'e':
                    self.emergency_stop()
                elif user_input == 'w':
                    if self.motors:
                        self.motors.move_forward(60)
                        print("⬆️ Moving forward")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 's':
                    if self.motors:
                        self.motors.move_backward(60)
                        print("⬇️ Moving backward")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'left':
                    if self.motors:
                        self.motors.turn_left(40)
                        print("⬅️ Turning left")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'right':
                    if self.motors:
                        self.motors.turn_right(40)
                        print("➡️ Turning right")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'stop':
                    if self.motors:
                        self.motors.stop_motors()
                        print("🛑 Motors stopped")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'p':
                    if self.motors:
                        params = self.motors.pid_params
                        print(f"\\n🎛️ PID Parameters:")
                        print(f"  Kp (Proportional): {params['kp']}")
                        print(f"  Ki (Integral): {params['ki']}")
                        print(f"  Kd (Derivative): {params['kd']}")
                        print(f"  Current Integral: {params['integral']:.2f}")
                        print(f"  Last Error: {params['last_error']:.2f}")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'u':
                    if self.motors:
                        try:
                            kp = float(input("Enter new Kp value (current: {:.1f}): ".format(self.motors.pid_params['kp'])))
                            ki = float(input("Enter new Ki value (current: {:.1f}): ".format(self.motors.pid_params['ki'])))
                            kd = float(input("Enter new Kd value (current: {:.1f}): ".format(self.motors.pid_params['kd'])))
                            self.motors.set_pid_parameters(kp, ki, kd)
                            print("✅ PID parameters updated")
                        except ValueError:
                            print("❌ Invalid input - PID parameters not changed")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'v':
                    if self.motors:
                        try:
                            base_speed = int(input(f"Enter base speed (current: {self.motors.base_speed}): "))
                            turn_speed = int(input(f"Enter turn speed (current: {self.motors.turn_speed}): "))
                            self.motors.set_speeds(base_speed, turn_speed)
                            print("✅ Motor speeds updated")
                        except ValueError:
                            print("❌ Invalid input - speeds not changed")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'l':
                    if self.motors:
                        print("\\nLine Following Modes:")
                        print("  1 - Camera-based")
                        print("  2 - Reflectance sensor")
                        print("  3 - Disabled")
                        choice = input("Select mode (1-3): ").strip()
                        
                        if choice == '1':
                            self.motors.set_line_following_mode(LineFollowingMode.CAMERA)
                        elif choice == '2':
                            self.motors.set_line_following_mode(LineFollowingMode.REFLECTANCE)
                        elif choice == '3':
                            self.motors.set_line_following_mode(LineFollowingMode.DISABLED)
                        else:
                            print("❌ Invalid choice")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'n':
                    if self.motors:
                        try:
                            heading = float(input(f"Enter target heading (0-360°, current: {self.motors.current_heading:.1f}°): "))
                            if 0 <= heading <= 360:
                                self.motors.turn_to_heading(heading)
                            else:
                                print("❌ Heading must be between 0-360°")
                        except ValueError:
                            print("❌ Invalid heading input")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'b':
                    if self.motors:
                        print("🚧 Testing barrier handling...")
                        # Simulate barrier detection
                        barrier_detected = self.motors.detect_barrier(point_count=2, brown_percentage=80)
                        if barrier_detected:
                            print("✅ Barrier detection triggered")
                            success = self.motors.handle_barrier()
                            print(f"🔄 Barrier handling {'successful' if success else 'failed'}")
                        else:
                            print("ℹ️ No barrier detected with test parameters")
                    else:
                        print("❌ Motor controller not available")
                elif user_input == 'x':
                    if self.motors:
                        status = self.motors.get_motor_status()
                        print(f"\\n🚗 Motor Controller Status:")
                        print(f"  Left Speed: {status['motor_speeds']['left']}")
                        print(f"  Right Speed: {status['motor_speeds']['right']}")
                        print(f"  Is Moving: {status['is_moving']}")
                        print(f"  Line Following: {status['line_following']['active']} ({status['line_following']['mode']})")
                        print(f"  Last Line Center: {status['line_following']['last_center']:.1f}%")
                        print(f"  Current Heading: {status['navigation']['current_heading']:.1f}°")
                        print(f"  Target Heading: {status['navigation']['target_heading']:.1f}°")
                        print(f"  Base Speed: {status['speeds']['base_speed']}")
                        print(f"  Turn Speed: {status['speeds']['turn_speed']}")
                    else:
                        print("❌ Motor controller not available")
                        
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break

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
        
        # Run main loop - with or without console controls
        if args.no_gui and args.console:
            # Start console controls in a separate thread
            console_thread = threading.Thread(target=controller.handle_keyboard_controls, daemon=True)
            console_thread.start()
        
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