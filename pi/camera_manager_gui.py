#!/usr/bin/env python3
"""
Camera Manager GUI - Dual Camera Testing Interface

Simple GUI that displays both camera feeds side-by-side with status information.
Separated from the main CameraManager class for clean architecture.
"""

import cv2
import numpy as np
import time
import signal
import sys
import threading
from typing import Optional, Dict, Tuple
from camera_manager import CameraManager


class CameraManagerGUI:
    """
    Testing GUI for dual camera display with status monitoring.
    """
    
    def __init__(self, config_file: str = "camera_config.json"):
        """Initialize GUI with camera manager."""
        self.config_file = config_file
        self.camera_manager = CameraManager(config_file)
        
        # GUI state
        self.running = False
        self.window_name = "Camera Manager - Dual Feed"
        
        # Display settings
        self.single_width = 400
        self.single_height = 300
        self.total_width = self.single_width * 2 + 60  # Gap between cameras
        self.total_height = self.single_height + 150   # Space for status
        
        # Colors for status display
        self.colors = {
            'connected': (0, 255, 0),      # Green
            'disconnected': (0, 0, 255),  # Red
            'warning': (0, 255, 255),     # Yellow
            'text': (255, 255, 255),      # White
            'background': (0, 0, 0),      # Black
            'border': (128, 128, 128)     # Gray
        }
        
        # Performance tracking
        self.gui_fps = 0.0
        self.last_gui_time = time.time()
        
        print(f"🖥️ Camera Manager GUI initialized")
        print(f"📄 Using config: {config_file}")
    
    def resize_frame(self, frame: np.ndarray) -> np.ndarray:
        """Resize frame to fit in display area."""
        if frame is None:
            return None
        
        return cv2.resize(frame, (self.single_width, self.single_height))
    
    def create_no_signal_frame(self, camera_id: int) -> np.ndarray:
        """Create a 'no signal' frame for disconnected cameras."""
        frame = np.zeros((self.single_height, self.single_width, 3), dtype=np.uint8)
        
        # Draw border
        cv2.rectangle(frame, (2, 2), (self.single_width-3, self.single_height-3), 
                     self.colors['disconnected'], 2)
        
        # Add text
        text_lines = [
            f"Camera {camera_id}",
            "NO SIGNAL",
            "Check connection"
        ]
        
        for i, text in enumerate(text_lines):
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            text_x = (self.single_width - text_size[0]) // 2
            text_y = (self.single_height // 2) - 30 + (i * 30)
            
            cv2.putText(frame, text, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.colors['disconnected'], 2)
        
        return frame
    
    def draw_camera_status(self, frame: np.ndarray, camera_id: int, status: Dict, x_offset: int) -> np.ndarray:
        """Draw status information for a camera."""
        y_start = self.single_height + 10
        
        # Camera title
        title = f"Camera {camera_id} (Device {status.get('device_id', '?')})"
        cv2.putText(frame, title, (x_offset, y_start), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['text'], 2)
        
        # Connection status
        connected = status.get('connected', False)
        conn_color = self.colors['connected'] if connected else self.colors['disconnected']
        conn_text = "CONNECTED" if connected else "DISCONNECTED"
        cv2.putText(frame, f"Status: {conn_text}", (x_offset, y_start + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, conn_color, 2)
        
        # FPS and frame count
        fps = status.get('fps', 0.0)
        frame_count = status.get('frame_count', 0)
        target_fps = status.get('target_fps', 30)
        
        fps_color = self.colors['connected'] if fps > target_fps * 0.8 else self.colors['warning']
        cv2.putText(frame, f"FPS: {fps:.1f}/{target_fps}", (x_offset, y_start + 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, fps_color, 1)
        
        cv2.putText(frame, f"Frames: {frame_count}", (x_offset, y_start + 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        
        # Resolution
        resolution = status.get('resolution', [0, 0])
        cv2.putText(frame, f"Resolution: {resolution[0]}x{resolution[1]}", (x_offset, y_start + 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        
        # Error information
        error_count = status.get('error_count', 0)
        if error_count > 0:
            error_color = self.colors['warning'] if error_count < 5 else self.colors['disconnected']
            cv2.putText(frame, f"Errors: {error_count}", (x_offset, y_start + 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, error_color, 1)
            
            # Show last error (truncated)
            last_error = status.get('last_error', '')
            if last_error:
                error_text = last_error[:30] + '...' if len(last_error) > 30 else last_error
                cv2.putText(frame, error_text, (x_offset, y_start + 125), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, error_color, 1)
        
        return frame
    
    def draw_system_info(self, frame: np.ndarray, system_status: Dict) -> np.ndarray:
        """Draw system-wide information."""
        # System status at top
        system_text = f"Camera Manager - Running: {system_status.get('running', False)}"
        cv2.putText(frame, system_text, (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['text'], 2)
        
        # GUI FPS
        cv2.putText(frame, f"GUI FPS: {self.gui_fps:.1f}", (10, 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        
        # Config file
        cv2.putText(frame, f"Config: {self.config_file}", (10, self.total_height - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        
        # Controls
        controls_text = "Controls: R=Restart cameras, S=Save screenshot, Q=Quit"
        cv2.putText(frame, controls_text, (10, self.total_height - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        
        return frame
    
    def create_display_frame(self) -> np.ndarray:
        """Create the complete display frame with both cameras and status."""
        # Create main display frame
        display_frame = np.zeros((self.total_height, self.total_width, 3), dtype=np.uint8)
        
        # Get frames from both cameras
        frame1, frame2 = self.camera_manager.get_both_frames()
        
        # Get system status
        system_status = self.camera_manager.get_system_status()
        
        # Process camera 1
        if frame1 is not None:
            resized_frame1 = self.resize_frame(frame1)
            display_frame[70:70+self.single_height, 10:10+self.single_width] = resized_frame1
        else:
            no_signal1 = self.create_no_signal_frame(1)
            display_frame[70:70+self.single_height, 10:10+self.single_width] = no_signal1
        
        # Process camera 2
        x_offset_cam2 = self.single_width + 50
        if frame2 is not None:
            resized_frame2 = self.resize_frame(frame2)
            display_frame[70:70+self.single_height, x_offset_cam2:x_offset_cam2+self.single_width] = resized_frame2
        else:
            no_signal2 = self.create_no_signal_frame(2)
            display_frame[70:70+self.single_height, x_offset_cam2:x_offset_cam2+self.single_width] = no_signal2
        
        # Draw camera borders
        cv2.rectangle(display_frame, (8, 68), (12+self.single_width, 72+self.single_height), 
                     self.colors['border'], 2)
        cv2.rectangle(display_frame, (x_offset_cam2-2, 68), 
                     (x_offset_cam2+self.single_width+2, 72+self.single_height), 
                     self.colors['border'], 2)
        
        # Draw status information
        cam1_status = system_status['cameras'].get(1, {})
        cam2_status = system_status['cameras'].get(2, {})
        
        display_frame = self.draw_camera_status(display_frame, 1, cam1_status, 10)
        display_frame = self.draw_camera_status(display_frame, 2, cam2_status, x_offset_cam2)
        display_frame = self.draw_system_info(display_frame, system_status)
        
        return display_frame
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input. Returns False if should quit."""
        if key == ord('q') or key == ord('Q') or key == 27:  # 'q' or ESC
            return False
        elif key == ord('r') or key == ord('R'):  # 'r' - restart cameras
            print("🔄 Restarting cameras...")
            self.camera_manager.stop()
            time.sleep(1)
            self.camera_manager.start()
        elif key == ord('s') or key == ord('S'):  # 's' - save screenshot
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"camera_screenshot_{timestamp}.jpg"
            display_frame = self.create_display_frame()
            cv2.imwrite(filename, display_frame)
            print(f"📸 Screenshot saved: {filename}")
        
        return True
    
    def update_gui_fps(self):
        """Update GUI FPS calculation."""
        current_time = time.time()
        dt = current_time - self.last_gui_time
        if dt > 0:
            self.gui_fps = 0.9 * self.gui_fps + 0.1 * (1.0 / dt)
        self.last_gui_time = current_time
    
    def run(self):
        """Main GUI loop."""
        print("🖥️ Starting Camera Manager GUI...")
        
        # Start camera manager
        self.camera_manager.start()
        
        # Create window
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        self.running = True
        print("🎮 Controls: R=restart cameras, S=save screenshot, Q=quit")
        
        try:
            while self.running:
                # Create display frame
                display_frame = self.create_display_frame()
                
                # Update GUI FPS
                self.update_gui_fps()
                
                # Show frame
                cv2.imshow(self.window_name, display_frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key != 255:  # Key was pressed
                    if not self.handle_key(key):
                        break
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n🛑 GUI interrupted")
        except Exception as e:
            print(f"❌ GUI error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up GUI and camera resources."""
        print("🛑 Cleaning up...")
        self.running = False
        
        # Stop camera manager
        self.camera_manager.stop()
        
        # Close GUI windows
        cv2.destroyAllWindows()
        
        print("✅ GUI cleanup completed")


# Signal handler for clean shutdown
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\n🛑 Shutting down camera manager GUI...")
    sys.exit(0)


if __name__ == "__main__":
    """
    Launch camera manager GUI.
    Usage: python camera_manager_gui.py [--config CONFIG_FILE]
    """
    import argparse
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Camera Manager GUI")
    parser.add_argument('--config', type=str, default='camera_config.json', 
                       help='Camera configuration file (default: camera_config.json)')
    args = parser.parse_args()
    
    print("🎥 Camera Manager GUI")
    
    try:
        # Create and run GUI
        gui = CameraManagerGUI(config_file=args.config)
        gui.run()
        
    except Exception as e:
        print(f"❌ Failed to start GUI: {e}")
        sys.exit(1)
    
    print("✅ Camera Manager GUI stopped")