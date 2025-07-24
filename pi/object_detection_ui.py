#!/usr/bin/env python3
"""
Object Detection Test UI

Visualization interface for object detection manager.
Shows camera feed with crop region (darkened areas) and latest detection.
"""

import cv2
import numpy as np
import time
import signal
import sys
import argparse
from typing import Optional, Tuple
from camera_manager import CameraManager
from object_detection_manager import ObjectDetectionManager


class ObjectDetectionUI:
    """
    Test UI for object detection visualization.
    Shows crop region and latest detections.
    """
    
    def __init__(self, camera_manager: CameraManager, camera_id: int = 2):
        """Initialize object detection UI."""
        self.camera_manager = camera_manager
        self.camera_id = camera_id
        
        # Create object detection manager
        self.detection_manager = ObjectDetectionManager(camera_manager, camera_id, config_file="object_detection_config_yolov11n.json")
        
        # UI settings
        self.window_name = f"Object Detection - Camera {camera_id}"
        self.running = False
        
        # Display settings
        self.display_width = 640
        self.display_height = 480
        
        # Colors
        self.colors = {
            'detection_box': (0, 255, 0),     # Green for detection
            'crop_overlay': (0, 0, 0),        # Black for darkened areas
            'crop_border': (255, 255, 0),     # Yellow for crop borders
            'text': (255, 255, 255),          # White for text
            'no_detection': (0, 0, 255)       # Red for no detection
        }
        
        # Performance tracking
        self.ui_fps = 0.0
        self.last_ui_time = time.time()
        self.detection_fps = 0.0
        self.last_detection_time = time.time()
        
        print(f"🎯 Object Detection UI initialized for camera {camera_id}")
    
    def draw_crop_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Draw crop region overlay - darken areas that are cropped out."""
        height, width = frame.shape[:2]
        overlay = frame.copy()
        
        # Get crop settings
        params = self.detection_manager.get_parameters()
        crop = params['crop']
        
        crop_top = crop['crop_top']
        crop_bottom = crop['crop_bottom']
        crop_left = crop['crop_left']
        crop_right = crop['crop_right']
        
        # Calculate crop boundaries
        top = max(0, crop_top)
        bottom = height - max(0, crop_bottom) if crop_bottom > 0 else height
        left = max(0, crop_left)
        right = width - max(0, crop_right) if crop_right > 0 else width
        
        # Create darkened overlay for cropped areas
        alpha = 0.6  # Transparency
        
        # Darken top area
        if top > 0:
            cv2.rectangle(overlay, (0, 0), (width, top), self.colors['crop_overlay'], -1)
        
        # Darken bottom area
        if bottom < height:
            cv2.rectangle(overlay, (0, bottom), (width, height), self.colors['crop_overlay'], -1)
        
        # Darken left area
        if left > 0:
            cv2.rectangle(overlay, (0, top), (left, bottom), self.colors['crop_overlay'], -1)
        
        # Darken right area
        if right < width:
            cv2.rectangle(overlay, (right, top), (width, bottom), self.colors['crop_overlay'], -1)
        
        # Blend overlay with original
        frame = cv2.addWeighted(frame, 1 - alpha, overlay, alpha, 0)
        
        # Draw crop region border
        if top < bottom and left < right:
            cv2.rectangle(frame, (left, top), (right, bottom), self.colors['crop_border'], 2)
            
            # Label crop region
            cv2.putText(frame, "CROP REGION", (left + 5, top + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['crop_border'], 2)
        
        return frame
    
    def draw_detection(self, frame: np.ndarray) -> np.ndarray:
        """Draw latest detection on frame."""
        detection = self.detection_manager.get_last_detection()
        
        if detection.found and detection.bbox:
            x1, y1, x2, y2 = detection.bbox
            
            # Ensure coordinates are within frame bounds
            height, width = frame.shape[:2]
            x1 = max(0, min(x1, width - 1))
            y1 = max(0, min(y1, height - 1))
            x2 = max(0, min(x2, width - 1))
            y2 = max(0, min(y2, height - 1))
            
            if x2 > x1 and y2 > y1:
                # Draw detection box
                cv2.rectangle(frame, (x1, y1), (x2, y2), self.colors['detection_box'], 3)
                
                # Draw label with confidence
                label = f"{detection.class_name}: {detection.confidence:.2f}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                
                # Draw label background
                cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                             (x1 + label_size[0] + 10, y1), self.colors['detection_box'], -1)
                
                # Draw label text
                cv2.putText(frame, label, (x1 + 5, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return frame
    
    def draw_status_info(self, frame: np.ndarray) -> np.ndarray:
        """Draw status information overlay."""
        height, width = frame.shape[:2]
        
        # Get status
        status = self.detection_manager.get_detection_status()
        detection = self.detection_manager.get_last_detection()
        model_loaded = status['model_loaded']
        
        # Status info background
        info_bg_height = 180
        cv2.rectangle(frame, (10, 10), (400, info_bg_height), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (400, info_bg_height), self.colors['text'], 2)
        
        # Status lines
        y_offset = 35
        line_height = 20
        
        # Model status
        model_color = self.colors['detection_box'] if model_loaded else self.colors['no_detection']
        cv2.putText(frame, f"🤖 Model: {'Loaded' if model_loaded else 'Not Loaded'}", 
                   (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, model_color, 1)
        y_offset += line_height
        
        # Detection status
        if detection.found:
            cv2.putText(frame, f"🎯 Detection: {detection.class_name} ({detection.confidence:.3f})", 
                       (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['detection_box'], 1)
            y_offset += line_height
            
            if detection.bbox:
                cv2.putText(frame, f"📍 Box: ({detection.bbox[0]}, {detection.bbox[1]}) to ({detection.bbox[2]}, {detection.bbox[3]})", 
                           (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
                y_offset += line_height
        else:
            cv2.putText(frame, f"🎯 Detection: None ({detection.class_name})", 
                       (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['no_detection'], 1)
            y_offset += line_height
        
        # Crop settings
        params = self.detection_manager.get_parameters()
        crop = params['crop']
        cv2.putText(frame, f"✂️ Crop: T:{crop['crop_top']} B:{crop['crop_bottom']} L:{crop['crop_left']} R:{crop['crop_right']}", 
                   (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        y_offset += line_height
        
        # Performance info
        cv2.putText(frame, f"📊 UI FPS: {self.ui_fps:.1f} | Det FPS: {self.detection_fps:.1f}", 
                   (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        y_offset += line_height
        
        # Total detections
        cv2.putText(frame, f"🔢 Total Detections: {status['detection_count']}", 
                   (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        
        # Controls info
        controls = [
            "Controls:",
            "T/G - Adjust crop top",
            "Y/H - Adjust crop bottom", 
            "U/J - Adjust crop left",
            "I/K - Adjust crop right",
            "R - Reset crop",
            "C - Change camera",
            "Q - Quit"
        ]
        
        ctrl_y_start = height - len(controls) * 18 - 10
        for i, control in enumerate(controls):
            cv2.putText(frame, control, (15, ctrl_y_start + i * 18), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, self.colors['text'], 1)
        
        return frame
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input. Returns False if should quit."""
        if key == ord('q') or key == ord('Q') or key == 27:  # Quit
            return False
        elif key == ord('r') or key == ord('R'):  # Reset crop
            self.detection_manager.setup_crop(0, 0, 0, 0)
        elif key == ord('t') or key == ord('T'):  # Increase crop top
            params = self.detection_manager.get_parameters()
            new_top = params['crop']['crop_top'] + 10
            self.detection_manager.setup_crop(crop_top=new_top)
        elif key == ord('g') or key == ord('G'):  # Decrease crop top
            params = self.detection_manager.get_parameters()
            new_top = max(0, params['crop']['crop_top'] - 10)
            self.detection_manager.setup_crop(crop_top=new_top)
        elif key == ord('y') or key == ord('Y'):  # Increase crop bottom
            params = self.detection_manager.get_parameters()
            new_bottom = params['crop']['crop_bottom'] + 10
            self.detection_manager.setup_crop(crop_bottom=new_bottom)
        elif key == ord('h') or key == ord('H'):  # Decrease crop bottom
            params = self.detection_manager.get_parameters()
            new_bottom = max(0, params['crop']['crop_bottom'] - 10)
            self.detection_manager.setup_crop(crop_bottom=new_bottom)
        elif key == ord('u') or key == ord('U'):  # Increase crop left
            params = self.detection_manager.get_parameters()
            new_left = params['crop']['crop_left'] + 10
            self.detection_manager.setup_crop(crop_left=new_left)
        elif key == ord('j') or key == ord('J'):  # Decrease crop left
            params = self.detection_manager.get_parameters()
            new_left = max(0, params['crop']['crop_left'] - 10)
            self.detection_manager.setup_crop(crop_left=new_left)
        elif key == ord('i') or key == ord('I'):  # Increase crop right
            params = self.detection_manager.get_parameters()
            new_right = params['crop']['crop_right'] + 10
            self.detection_manager.setup_crop(crop_right=new_right)
        elif key == ord('k') or key == ord('K'):  # Decrease crop right
            params = self.detection_manager.get_parameters()
            new_right = max(0, params['crop']['crop_right'] - 10)
            self.detection_manager.setup_crop(crop_right=new_right)
        elif key == ord('c') or key == ord('C'):  # Change camera
            new_camera = 1 if self.camera_id == 2 else 2
            self.detection_manager.set_camera_id(new_camera)
            self.camera_id = new_camera
        
        return True
    
    def update_fps(self):
        """Update FPS calculations."""
        current_time = time.time()
        
        # UI FPS
        dt = current_time - self.last_ui_time
        if dt > 0:
            self.ui_fps = 0.9 * self.ui_fps + 0.1 * (1.0 / dt)
        self.last_ui_time = current_time
        
        # Detection FPS tracking - just monitor when results change
        last_detection = self.detection_manager.get_last_detection()
        
        if last_detection.timestamp > self.last_detection_time:
            detection_time = current_time - self.last_detection_time
            if detection_time > 0:
                self.detection_fps = 0.9 * self.detection_fps + 0.1 * (1.0 / detection_time)
            self.last_detection_time = last_detection.timestamp
    
    def run(self):
        """Main UI loop."""
        print("🎯 Starting Object Detection UI...")
        print("Controls: T/G=crop top, Y/H=crop bottom, U/J=crop left, I/K=crop right, R=reset, C=camera, Q=quit")
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        self.running = True
        
        try:
            while self.running:
                # Get camera frame
                frame = self.camera_manager.get_frame(self.camera_id)

                self.detection_manager.run_detection()
                
                if frame is not None:
                    # Resize frame for display
                    display_frame = cv2.resize(frame, (self.display_width, self.display_height))
                    
                    # Draw crop overlay (darkened areas)
                    display_frame = self.draw_crop_overlay(display_frame)
                    
                    # Draw latest detection
                    display_frame = self.draw_detection(display_frame)
                    
                    # Draw status information
                    display_frame = self.draw_status_info(display_frame)
                    
                    # Show frame
                    cv2.imshow(self.window_name, display_frame)
                else:
                    # No camera feed
                    no_feed_frame = np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8)
                    cv2.putText(no_feed_frame, f"NO CAMERA FEED - Camera {self.camera_id}",
                               (50, self.display_height//2),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, self.colors['text'], 2)
                    cv2.imshow(self.window_name, no_feed_frame)
                
                # Update FPS and run detection
                self.update_fps()
                
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
        if hasattr(self.detection_manager, 'cleanup'):
            self.detection_manager.cleanup()
        cv2.destroyAllWindows()
        print("✅ Object Detection UI cleaned up")


# Signal handler
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\n🛑 Shutting down object detection UI...")
    sys.exit(0)


if __name__ == "__main__":
    """Main entry point for object detection UI."""
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse arguments
    parser = argparse.ArgumentParser(description="Object Detection Test UI")
    parser.add_argument('--camera', type=int, default=2, 
                       help='Camera ID for object detection (default: 2)')
    args = parser.parse_args()
    
    print("🚀 Starting Object Detection Test UI...")
    
    try:
        # Create and start camera manager
        camera_manager = CameraManager()
        camera_manager.start()
        
        # Create and run UI
        ui = ObjectDetectionUI(camera_manager, args.camera)
        ui.run()
        
    except Exception as e:
        print(f"❌ Failed to start UI: {e}")
    finally:
        if 'camera_manager' in locals():
            camera_manager.stop()
        print("✅ Object Detection UI stopped")