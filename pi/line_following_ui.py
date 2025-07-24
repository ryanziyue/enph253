#!/usr/bin/env python3
"""
Line Following Test UI

Based on legacy/src/line_following_controller.py GUI implementation.
Simple visualization interface for line following manager.
"""

import cv2
import numpy as np
import time
import signal
import sys
import argparse
from typing import List, Dict
from camera_manager import CameraManager
from line_following_manager import LineFollowingManager


class LineFollowingUI:
    """
    Test UI for line following visualization.
    Based on legacy controller GUI implementation.
    """
    
    def __init__(self, camera_manager: CameraManager, camera_id: int = 1):
        """Initialize line following UI."""
        self.camera_manager = camera_manager
        self.camera_id = camera_id
        
        # Create line following manager
        self.line_manager = LineFollowingManager(camera_manager, camera_id)
        
        # UI settings
        self.window_name = f"Line Following - Camera {camera_id}"
        self.running = False
        
        # Performance tracking
        self.ui_fps = 0.0
        self.last_ui_time = time.time()
        
        print(f"🖥️ Line Following UI initialized for camera {camera_id}")
    
    def draw_search_region(self, frame: np.ndarray, detection: Dict) -> np.ndarray:
        """Draw search region boundaries."""
        height, width = frame.shape[:2]
        
        # Draw search region
        if 'search_region' in detection:
            search = detection['search_region']
            cv2.rectangle(frame, (search['left'], 0), (search['right'], height), (255, 0, 255), 2)
            cv2.putText(frame, "SEARCH REGION", (search['left'] + 5, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        return frame
    
    def draw_detected_points(self, frame: np.ndarray, detection: Dict) -> np.ndarray:
        """Draw detected line points."""
        if detection['found'] and detection['detected_points']:
            for i, point in enumerate(detection['detected_points']):
                # Color code based on brown content
                brown_pct = point.get('brown_percentage', 0)
                if brown_pct <= 25:
                    color = (0, 255, 0)  # Green - clean
                elif brown_pct <= 50:
                    color = (0, 255, 255)  # Yellow - acceptable
                else:
                    color = (0, 165, 255)  # Orange - filtered
                
                cv2.circle(frame, (point['x'], point['y']), 4, color, -1)
                # Show point number
                cv2.putText(frame, str(i+1), (point['x'] + 6, point['y']), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
        
        return frame
    
    def draw_status_info(self, frame: np.ndarray, detection: Dict) -> np.ndarray:
        """Draw status information overlay."""
        height, width = frame.shape[:2]
        
        # Show detection info
        info_y = 30
        if detection['found']:
            cv2.putText(frame, f"LINE DETECTED", (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            info_y += 25
            cv2.putText(frame, f"Points: {len(detection['detected_points'])}", (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cv2.putText(frame, f"NO LINE", (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        info_y += 25
        cv2.putText(frame, f"UI FPS: {self.ui_fps:.1f}", (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show adaptive center
        if self.line_manager.adaptive_center_enabled and self.line_manager.adaptive_center_x:
            cv2.line(frame, (self.line_manager.adaptive_center_x, 0), 
                    (self.line_manager.adaptive_center_x, height), (255, 165, 0), 2)
            cv2.putText(frame, "ADAPTIVE CENTER", (self.line_manager.adaptive_center_x + 5, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 165, 0), 1)
        
        # Controls info
        controls = [
            "Controls:",
            "R - Reset adaptive center",
            "W/S - Adjust search width",
            "B/V - Adjust brightness",
            "C - Change camera",
            "Q - Quit"
        ]
        
        ctrl_y_start = height - len(controls) * 20 - 10
        for i, control in enumerate(controls):
            y_pos = ctrl_y_start + 15 + i * 20
            cv2.putText(frame, control, (15, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return frame
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input. Returns False if should quit."""
        if key == ord('q') or key == ord('Q') or key == 27:  # Quit
            return False
        elif key == ord('r') or key == ord('R'):  # Reset adaptive center
            self.line_manager.reset_adaptive_center()
        elif key == ord('w') or key == ord('W'):  # Increase search width
            current_width = self.line_manager.config['detection']['search_width_percent']
            new_width = min(80.0, current_width + 5.0)
            self.line_manager.set_search_width(new_width)
        elif key == ord('s') or key == ord('S'):  # Decrease search width
            current_width = self.line_manager.config['detection']['search_width_percent']
            new_width = max(10.0, current_width - 5.0)
            self.line_manager.set_search_width(new_width)
        elif key == ord('b') or key == ord('B'):  # Increase brightness threshold
            current_brightness = self.line_manager.config['detection']['max_brightness']
            self.line_manager.config['detection']['max_brightness'] = min(150, current_brightness + 10)
            print(f"💡 Brightness threshold: {self.line_manager.config['detection']['max_brightness']}")
        elif key == ord('v') or key == ord('V'):  # Decrease brightness threshold
            current_brightness = self.line_manager.config['detection']['max_brightness']
            self.line_manager.config['detection']['max_brightness'] = max(30, current_brightness - 10)
            print(f"💡 Brightness threshold: {self.line_manager.config['detection']['max_brightness']}")
        elif key == ord('c') or key == ord('C'):  # Change camera
            new_camera = 2 if self.camera_id == 1 else 1
            self.line_manager.set_camera_id(new_camera)
            self.camera_id = new_camera
        
        return True
    
    def update_ui_fps(self):
        """Update UI FPS calculation."""
        current_time = time.time()
        dt = current_time - self.last_ui_time
        if dt > 0:
            self.ui_fps = 0.9 * self.ui_fps + 0.1 * (1.0 / dt)
        self.last_ui_time = current_time
    
    def run(self):
        """Main UI loop based on legacy controller GUI."""
        print("🖥️ Starting Line Following UI...")
        print("Controls: R=reset center, W/S=search width, B/V=brightness, C=change camera, Q=quit")
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        self.running = True
        
        try:
            while self.running:
                # Get camera frame
                frame = self.camera_manager.get_frame(self.camera_id)
                
                if frame is not None:
                    # Get line detections
                    points = self.line_manager.get_line()
                    detection = self.line_manager.last_detection
                    
                    if detection is not None:
                        # Create display frame
                        display_frame = frame.copy()
                        
                        # Draw visualizations based on legacy controller
                        display_frame = self.draw_search_region(display_frame, detection)
                        display_frame = self.draw_detected_points(display_frame, detection)
                        display_frame = self.draw_status_info(display_frame, detection)
                        
                        # Show frame
                        cv2.imshow(self.window_name, display_frame)
                    else:
                        # Show raw frame if no detection yet
                        cv2.imshow(self.window_name, frame)
                else:
                    # No camera feed
                    no_feed_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(no_feed_frame, f"NO CAMERA FEED - Camera {self.camera_id}",
                               (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.imshow(self.window_name, no_feed_frame)
                
                # Update UI FPS
                self.update_ui_fps()
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    if not self.handle_key(key):
                        break
                
                time.sleep(0.01)  # Small delay
                
        except KeyboardInterrupt:
            print("\n🛑 UI interrupted")
        except Exception as e:
            print(f"❌ UI error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up UI resources."""
        self.running = False
        cv2.destroyAllWindows()
        print("✅ Line Following UI cleaned up")


# Signal handler
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\n🛑 Shutting down line following UI...")
    sys.exit(0)


if __name__ == "__main__":
    """Main entry point for line following UI."""
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse arguments
    parser = argparse.ArgumentParser(description="Line Following Test UI")
    parser.add_argument('--camera', type=int, default=1, 
                       help='Camera ID for line following (default: 1)')
    args = parser.parse_args()
    
    print("🚀 Starting Line Following Test UI...")
    
    try:
        # Create and start camera manager
        camera_manager = CameraManager()
        camera_manager.start()
        
        # Create and run UI
        ui = LineFollowingUI(camera_manager, args.camera)
        ui.run()
        
    except Exception as e:
        print(f"❌ Failed to start UI: {e}")
    finally:
        if 'camera_manager' in locals():
            camera_manager.stop()
        print("✅ Line Following UI stopped")