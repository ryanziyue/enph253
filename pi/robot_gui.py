#!/usr/bin/env python3
"""
Robot GUI - Main Visual Interface

Displays dual camera feeds with extensible architecture for annotations.
Designed to easily add bounding boxes, points, lines, and other visual elements.
"""

import cv2
import numpy as np
import time
import threading
import queue
from typing import Optional, Dict, List, Tuple, Callable
from dataclasses import dataclass
from camera_manager import CameraManager
from line_following_manager import LineFollowingManager
from object_detection_manager import ObjectDetectionManager


@dataclass
class Annotation:
    """Base class for annotations to draw on frames."""
    camera_id: int  # Which camera (1 or 2)
    color: Tuple[int, int, int] = (0, 255, 0)  # BGR color
    thickness: int = 2


class BoundingBox(Annotation):
    """Bounding box annotation."""
    def __init__(self, camera_id: int, x: int, y: int, width: int, height: int, 
                 label: str = "", color: Tuple[int, int, int] = (0, 255, 0), thickness: int = 2):
        super().__init__(camera_id, color, thickness)
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.label = label


class Point(Annotation):
    """Point annotation."""
    def __init__(self, camera_id: int, x: int, y: int, radius: int = 5, 
                 filled: bool = True, color: Tuple[int, int, int] = (0, 255, 0), thickness: int = 2):
        super().__init__(camera_id, color, thickness)
        self.x = x
        self.y = y
        self.radius = radius
        self.filled = filled


class Line(Annotation):
    """Line annotation."""
    def __init__(self, camera_id: int, x1: int, y1: int, x2: int, y2: int,
                 color: Tuple[int, int, int] = (0, 255, 0), thickness: int = 2):
        super().__init__(camera_id, color, thickness)
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2


class Text(Annotation):
    """Text annotation."""
    def __init__(self, camera_id: int, x: int, y: int, text: str,
                 font_scale: float = 0.5, color: Tuple[int, int, int] = (0, 255, 0), thickness: int = 2):
        super().__init__(camera_id, color, thickness)
        self.x = x
        self.y = y
        self.text = text
        self.font_scale = font_scale


class RobotGUI:
    """
    Main robot GUI with extensible annotation system.
    """
    
    def __init__(self, camera_manager: CameraManager, object_detection_manager: Optional[ObjectDetectionManager] = None):
        """
        Initialize GUI with camera manager.
        
        Args:
            camera_manager: Initialized CameraManager instance
            object_detection_manager: Optional ObjectDetectionManager instance
        """
        self.camera_manager = camera_manager
        self.object_detection_manager = object_detection_manager
        
        # Line following manager for camera 1 only
        self.line_manager_1 = LineFollowingManager(camera_manager, camera_id=1)
        self.line_manager_2 = None  # No line following on camera 2
        
        # GUI state
        self.running = False
        self.window_name = "Robot Vision System"
        
        # Display settings
        self.single_width = 400
        self.single_height = 300
        self.total_width = self.single_width * 2 + 60
        self.total_height = self.single_height + 140  # Extra space for line following info
        
        # Colors
        self.colors = {
            'connected': (0, 255, 0),
            'disconnected': (0, 0, 255),
            'warning': (0, 255, 255),
            'text': (255, 255, 255),
            'background': (0, 0, 0),
            'border': (128, 128, 128)
        }
        
        # Annotation system with thread-safe queue
        self.annotation_queue = queue.Queue(maxsize=100)
        self.annotations: List[Annotation] = []
        self.persistent_annotations: List[Annotation] = []
        
        # Callback system for custom drawing
        self.draw_callbacks: List[Callable] = []
        
        # Performance tracking
        self.gui_fps = 0.0
        self.last_gui_time = time.time()
        
        # Line following display settings
        self.show_line_following = True
        
        print("🤖 Robot GUI initialized with line following")
    
    def add_annotation(self, annotation: Annotation, persistent: bool = False):
        """
        Add an annotation to be drawn (thread-safe).
        
        Args:
            annotation: Annotation object to draw
            persistent: If True, annotation persists across frames
        """
        try:
            self.annotation_queue.put_nowait((annotation, persistent))
        except queue.Full:
            print("⚠️ Annotation queue full, dropping annotation")
    
    def _process_annotation_queue(self):
        """Process pending annotations from queue."""
        while not self.annotation_queue.empty():
            try:
                annotation, persistent = self.annotation_queue.get_nowait()
                if persistent:
                    self.persistent_annotations.append(annotation)
                else:
                    self.annotations.append(annotation)
            except queue.Empty:
                break
    
    def clear_annotations(self, persistent: bool = False):
        """
        Clear annotations.
        
        Args:
            persistent: If True, clear persistent annotations, otherwise clear frame annotations
        """
        if persistent:
            self.persistent_annotations.clear()
        else:
            self.annotations.clear()
    
    def add_draw_callback(self, callback: Callable):
        """
        Add a custom drawing callback.
        
        Args:
            callback: Function that takes (frame, camera_id) and returns modified frame
        """
        self.draw_callbacks.append(callback)
    
    def draw_annotations(self, frame: np.ndarray, camera_id: int) -> np.ndarray:
        """Draw all annotations for a specific camera."""
        # Draw persistent annotations
        for ann in self.persistent_annotations:
            if ann.camera_id == camera_id:
                frame = self._draw_single_annotation(frame, ann)
        
        # Draw frame-specific annotations
        for ann in self.annotations:
            if ann.camera_id == camera_id:
                frame = self._draw_single_annotation(frame, ann)
        
        return frame
    
    def _draw_single_annotation(self, frame: np.ndarray, annotation: Annotation) -> np.ndarray:
        """Draw a single annotation based on its type."""
        if isinstance(annotation, BoundingBox):
            cv2.rectangle(frame, 
                         (annotation.x, annotation.y),
                         (annotation.x + annotation.width, annotation.y + annotation.height),
                         annotation.color, annotation.thickness)
            if annotation.label:
                cv2.putText(frame, annotation.label,
                           (annotation.x, annotation.y - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                           annotation.color, 1)
        
        elif isinstance(annotation, Point):
            if annotation.filled:
                cv2.circle(frame, (annotation.x, annotation.y), 
                          annotation.radius, annotation.color, -1)
            else:
                cv2.circle(frame, (annotation.x, annotation.y), 
                          annotation.radius, annotation.color, annotation.thickness)
        
        elif isinstance(annotation, Line):
            cv2.line(frame, (annotation.x1, annotation.y1),
                    (annotation.x2, annotation.y2),
                    annotation.color, annotation.thickness)
        
        elif isinstance(annotation, Text):
            cv2.putText(frame, annotation.text,
                       (annotation.x, annotation.y),
                       cv2.FONT_HERSHEY_SIMPLEX, annotation.font_scale,
                       annotation.color, annotation.thickness)
        
        return frame
    
    def draw_line_following(self, frame: np.ndarray, camera_id: int) -> np.ndarray:
        """Draw line following visualization on frame."""
        if not self.show_line_following:
            return frame
            
        height, width = frame.shape[:2]
        
        # Get line manager for this camera (only camera 1 has line following)
        if camera_id == 1:
            line_manager = self.line_manager_1
        else:
            return frame  # No line following on camera 2
        
        # Get line detection points
        points = line_manager.get_line()
        detection = line_manager.last_detection
        
        if detection and detection['found']:
            # Draw search region
            if 'search_region' in detection:
                search = detection['search_region']
                cv2.rectangle(frame, (search['left'], 0), (search['right'], height), (255, 0, 255), 1)
            
            # Draw detected points
            for i, point in enumerate(points):
                # Convert percentage back to pixels for display
                x = int(point['x'])
                y = int(point['y'])
                point_width = int(point['width'])
                brown_pct = point.get('brown_percentage', 0)
                brown_rejected = point.get('brown_rejected', False)
                
                # Color code based on brown content
                if brown_rejected:
                    color = (0, 0, 255)  # Red for rejected
                elif brown_pct <= 25:
                    color = (0, 255, 0)  # Green - clean
                elif brown_pct <= 50:
                    color = (0, 255, 255)  # Yellow - acceptable
                else:
                    color = (0, 165, 255)  # Orange
                
                # Draw line segment
                half_width = point_width // 2
                cv2.line(frame, (x - half_width, y), (x + half_width, y), color, 2)
                
                # Draw center point
                cv2.circle(frame, (x, y), 3, color, -1)
                
                # Show brown percentage for first few points
                if i < 3:
                    cv2.putText(frame, f"{brown_pct:.0f}%", (x + 5, y - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
            
            # Draw adaptive center
            if line_manager.adaptive_center_enabled and line_manager.adaptive_center_x:
                cv2.line(frame, (line_manager.adaptive_center_x, 0), 
                        (line_manager.adaptive_center_x, height), (255, 165, 0), 1)
        
        # Draw barrier detection area if enabled
        barrier_config = line_manager.config.get('barrier_detection', {})
        show_barrier_area = line_manager.config.get('visualization', {}).get('show_barrier_area', False)
        
        if (barrier_config.get('enabled', True) and show_barrier_area and 
            hasattr(line_manager, 'get_barrier_area_coords')):
            
            # Get current barrier percentage and area coordinates
            barrier_percentage = line_manager.get_barrier_percentage()
            barrier_coords = line_manager.get_barrier_area_coords()
            
            if barrier_coords:
                y1, y2, x1, x2 = barrier_coords
                
                # Choose color based on barrier percentage
                threshold = barrier_config.get('brown_threshold_percent', 60.0)
                if barrier_percentage > threshold:
                    # Red - barrier detected
                    barrier_color = (0, 0, 255)
                    barrier_thickness = 3
                elif barrier_percentage > threshold * 0.7:
                    # Orange - warning level
                    barrier_color = (0, 127, 255)
                    barrier_thickness = 2
                else:
                    # Green - safe
                    barrier_color = (0, 255, 0)
                    barrier_thickness = 2
                
                # Draw barrier detection rectangle
                cv2.rectangle(frame, (x1, y1), (x2, y2), barrier_color, barrier_thickness)
                
                # Add label with barrier percentage
                label = f"Barrier: {barrier_percentage:.1f}%"
                label_x = x1 + 5
                label_y = y1 - 10 if y1 > 25 else y1 + 20
                
                # Draw label background
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                cv2.rectangle(frame, (label_x - 2, label_y - label_size[1] - 2),
                            (label_x + label_size[0] + 2, label_y + 2), 
                            (0, 0, 0), -1)  # Black background
                
                # Draw label text
                cv2.putText(frame, label, (label_x, label_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, barrier_color, 1)
        
        return frame
    
    def draw_object_detection(self, frame: np.ndarray, camera_id: int) -> np.ndarray:
        """Draw object detection results on frame."""
        if camera_id != 2 or not self.object_detection_manager:
            return frame
        
        # Get latest detection result (don't trigger new detection, just get cached result)
        detection = self.object_detection_manager.get_last_detection()
        
        if detection.found and detection.bbox:
            x1, y1, x2, y2 = detection.bbox
            
            # Ensure coordinates are within frame bounds
            height, width = frame.shape[:2]
            x1 = max(0, min(int(x1), width - 1))
            y1 = max(0, min(int(y1), height - 1))
            x2 = max(0, min(int(x2), width - 1))
            y2 = max(0, min(int(y2), height - 1))
            
            if x2 > x1 and y2 > y1:
                # Draw detection box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                
                # Draw label with confidence
                label = f"{detection.class_name}: {detection.confidence:.2f}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                
                # Draw label background
                cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                             (x1 + label_size[0] + 10, y1), (0, 255, 0), -1)
                
                # Draw label text
                cv2.putText(frame, label, (x1 + 5, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return frame
    
    def resize_frame(self, frame: np.ndarray) -> np.ndarray:
        """Resize frame to fit display area."""
        if frame is None:
            return None
        return cv2.resize(frame, (self.single_width, self.single_height))
    
    def create_no_signal_frame(self, camera_id: int) -> np.ndarray:
        """Create a 'no signal' frame for disconnected cameras."""
        frame = np.zeros((self.single_height, self.single_width, 3), dtype=np.uint8)
        
        cv2.rectangle(frame, (2, 2), (self.single_width-3, self.single_height-3), 
                     self.colors['disconnected'], 2)
        
        text = f"Camera {camera_id} - NO SIGNAL"
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        text_x = (self.single_width - text_size[0]) // 2
        text_y = self.single_height // 2
        
        cv2.putText(frame, text, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.colors['disconnected'], 2)
        
        return frame
    
    def draw_camera_info(self, frame: np.ndarray, camera_id: int, status: Dict, x_offset: int):
        """Draw camera status information."""
        y_start = self.single_height + 10
        
        # Camera title
        title = f"Camera {camera_id}"
        cv2.putText(frame, title, (x_offset, y_start), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['text'], 2)
        
        # FPS
        fps = status.get('fps', 0.0)
        target_fps = status.get('target_fps', 30)
        fps_color = self.colors['connected'] if fps > target_fps * 0.8 else self.colors['warning']
        cv2.putText(frame, f"FPS: {fps:.1f}/{target_fps}", (x_offset, y_start + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, fps_color, 1)
        
        # Connection status
        connected = status.get('connected', False)
        conn_color = self.colors['connected'] if connected else self.colors['disconnected']
        conn_text = "Connected" if connected else "Disconnected"
        cv2.putText(frame, conn_text, (x_offset, y_start + 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, conn_color, 1)
    
    def draw_line_following_info(self, frame: np.ndarray):
        """Draw line following status information."""
        y_start = self.single_height + 75
        
        # Get line detection status for camera 1 only
        status1 = self.line_manager_1.get_detection_status()
        
        # Camera 1 line info
        points1 = status1.get('points', [])
        line_found1 = status1.get('line_found', False)
        
        color1 = self.colors['connected'] if line_found1 else self.colors['disconnected']
        cv2.putText(frame, f"Line 1: {'✓' if line_found1 else '✗'} ({len(points1)} pts)", 
                   (10, y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color1, 1)
        
        # Camera 2 - show object detection status
        x_offset_cam2 = self.single_width + 50
        if self.object_detection_manager:
            obj_status = self.object_detection_manager.get_detection_status()
            obj_found = obj_status['detection_found']
            obj_count = obj_status['detection_count']
            obj_color = self.colors['connected'] if obj_found else self.colors['disconnected']
            cv2.putText(frame, f"Obj 2: {'✓' if obj_found else '✗'} ({obj_count} total)", 
                       (x_offset_cam2, y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.5, obj_color, 1)
        else:
            cv2.putText(frame, f"Obj 2: Not Available", 
                       (x_offset_cam2, y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['disconnected'], 1)
        
        # Show line following parameters and barrier detection
        cv2.putText(frame, f"Search Width: {self.line_manager_1.config['detection']['search_width_percent']:.0f}%", 
                   (10, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        
        # Show barrier detection status
        barrier_percentage = status1.get('barrier_percentage', 0.0)
        barrier_config = self.line_manager_1.config.get('barrier_detection', {})
        barrier_threshold = barrier_config.get('brown_threshold_percent', 60.0)
        barrier_enabled = barrier_config.get('enabled', True)
        
        if barrier_enabled:
            barrier_color = self.colors['warning'] if barrier_percentage > barrier_threshold else self.colors['connected']
            cv2.putText(frame, f"Barrier: {barrier_percentage:.1f}% ({'⚠️' if barrier_percentage > barrier_threshold else '✓'})", 
                       (10, y_start + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, barrier_color, 1)
        else:
            cv2.putText(frame, f"Barrier: Disabled", 
                       (10, y_start + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['disconnected'], 1)
        
        # Show object detection info
        if self.object_detection_manager:
            obj_status = self.object_detection_manager.get_detection_status()
            last_detection = obj_status['last_detection']
            if last_detection.found:
                cv2.putText(frame, f"Last: {last_detection.class_name} ({last_detection.confidence:.2f})", 
                           (x_offset_cam2, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['connected'], 1)
            else:
                cv2.putText(frame, f"Object Display: ON", 
                           (x_offset_cam2, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
        else:
            cv2.putText(frame, f"Line Display: {'ON' if self.show_line_following else 'OFF'}", 
                       (x_offset_cam2, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
    
    def create_display_frame(self) -> np.ndarray:
        """Create the complete display frame."""
        # Create main display frame
        display_frame = np.zeros((self.total_height, self.total_width, 3), dtype=np.uint8)
        
        # Get frames
        frame1, frame2 = self.camera_manager.get_both_frames()
        
        # Get system status
        system_status = self.camera_manager.get_system_status()
        
        # Process camera 1
        if frame1 is not None:
            # Apply custom draw callbacks
            for callback in self.draw_callbacks:
                frame1 = callback(frame1, 1)
            
            # Draw line following visualization
            frame1 = self.draw_line_following(frame1, 1)
            
            # Draw annotations
            frame1 = self.draw_annotations(frame1, 1)
            
            # Resize and place
            resized_frame1 = self.resize_frame(frame1)
            display_frame[50:50+self.single_height, 10:10+self.single_width] = resized_frame1
        else:
            no_signal1 = self.create_no_signal_frame(1)
            display_frame[50:50+self.single_height, 10:10+self.single_width] = no_signal1
        
        # Process camera 2
        x_offset_cam2 = self.single_width + 50
        if frame2 is not None:
            # Apply custom draw callbacks
            for callback in self.draw_callbacks:
                frame2 = callback(frame2, 2)
            
            # Draw object detection visualization (no line following on camera 2)
            frame2 = self.draw_object_detection(frame2, 2)
            
            # Draw annotations
            frame2 = self.draw_annotations(frame2, 2)
            
            # Resize and place
            resized_frame2 = self.resize_frame(frame2)
            display_frame[50:50+self.single_height, x_offset_cam2:x_offset_cam2+self.single_width] = resized_frame2
        else:
            no_signal2 = self.create_no_signal_frame(2)
            display_frame[50:50+self.single_height, x_offset_cam2:x_offset_cam2+self.single_width] = no_signal2
        
        # Draw borders
        cv2.rectangle(display_frame, (8, 48), (12+self.single_width, 52+self.single_height), 
                     self.colors['border'], 2)
        cv2.rectangle(display_frame, (x_offset_cam2-2, 48), 
                     (x_offset_cam2+self.single_width+2, 52+self.single_height), 
                     self.colors['border'], 2)
        
        # Draw status
        cv2.putText(display_frame, "Robot Vision System", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.colors['text'], 2)
        
        cv2.putText(display_frame, f"GUI FPS: {self.gui_fps:.1f}", (self.total_width - 120, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        
        # Camera info
        cam1_status = system_status['cameras'].get(1, {})
        cam2_status = system_status['cameras'].get(2, {})
        self.draw_camera_info(display_frame, 1, cam1_status, 10)
        self.draw_camera_info(display_frame, 2, cam2_status, x_offset_cam2)
        
        # Line following info
        self.draw_line_following_info(display_frame)
        
        # Controls
        controls = "Controls: Q=Quit, P=Screenshot, R=Reset Centers, W/S=Search Width, L=Toggle Lines, B=Toggle Barrier"
        cv2.putText(display_frame, controls, (10, self.total_height - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, self.colors['text'], 1)
        
        # Clear frame-specific annotations
        self.clear_annotations(persistent=False)
        
        return display_frame
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input."""
        if key == ord('q') or key == ord('Q') or key == 27:
            return False
        elif key == ord('p') or key == ord('P'):  # Screenshot (changed from S)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"robot_screenshot_{timestamp}.jpg"
            display_frame = self.create_display_frame()
            cv2.imwrite(filename, display_frame)
            print(f"📸 Screenshot saved: {filename}")
        elif key == ord('r') or key == ord('R'):  # Reset adaptive center
            self.line_manager_1.reset_adaptive_center()
            print("🎯 Adaptive center reset for camera 1")
        elif key == ord('w') or key == ord('W'):  # Increase search width
            current_width = self.line_manager_1.config['detection']['search_width_percent']
            new_width = min(80.0, current_width + 5.0)
            self.line_manager_1.set_search_width(new_width)
            print(f"🔍 Search width: {new_width}%")
        elif key == ord('s') or key == ord('S'):  # Decrease search width
            current_width = self.line_manager_1.config['detection']['search_width_percent']
            new_width = max(10.0, current_width - 5.0)
            self.line_manager_1.set_search_width(new_width)
            print(f"🔍 Search width: {new_width}%")
        elif key == ord('l') or key == ord('L'):  # Toggle line following display
            self.show_line_following = not self.show_line_following
            print(f"📏 Line following display: {'ON' if self.show_line_following else 'OFF'}")
        elif key == ord('b') or key == ord('B'):  # Toggle barrier area display
            current_show = self.line_manager_1.config.get('visualization', {}).get('show_barrier_area', False)
            self.line_manager_1.config['visualization']['show_barrier_area'] = not current_show
            print(f"🚧 Barrier area display: {'ON' if not current_show else 'OFF'}")
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
        print("🤖 Starting Robot GUI...")
        
        window_created = False
        try:
            # Create window in the GUI thread
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            window_created = True
            self.running = True
            
            while self.running:
                # Process any pending annotations from other threads
                self._process_annotation_queue()
                
                display_frame = self.create_display_frame()
                self.update_gui_fps()
                
                # Show frame
                cv2.imshow(self.window_name, display_frame)
                
                # Check for window close
                try:
                    if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                        print("🖥️ GUI window closed")
                        break
                except cv2.error:
                    # Window might be closed already
                    break
                
                # Handle keyboard
                key = cv2.waitKey(30) & 0xFF  # 30ms wait
                if key != 255 and key != -1:
                    if not self.handle_key(key):
                        break
                
        except Exception as e:
            print(f"❌ GUI error: {e}")
        finally:
            self.running = False
            if window_created:
                try:
                    cv2.destroyWindow(self.window_name)
                    cv2.waitKey(1)  # Process pending GUI events
                except:
                    pass
            print("✅ GUI thread finished")
    
    def stop(self):
        """Stop the GUI."""
        self.running = False
    
    def cleanup(self):
        """Clean up GUI resources."""
        # Cleanup is now handled in the run() method
        pass


# Example of how to add custom drawing
def example_custom_draw(frame, camera_id):
    """Example custom drawing function."""
    # Draw crosshair at center
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2
    cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 255), 1)
    cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 255), 1)
    return frame


if __name__ == "__main__":
    """Test the GUI standalone."""
    print("🤖 Testing Robot GUI...")
    
    # Create camera manager
    camera_manager = CameraManager()
    camera_manager.start()
    
    # Create and run GUI
    gui = RobotGUI(camera_manager)
    
    # Example: Add some test annotations
    gui.add_annotation(
        BoundingBox(camera_id=1, x=100, y=100, width=50, height=50, 
                   label="Test", color=(0, 255, 0)),
        persistent=True
    )
    
    gui.add_annotation(
        Point(camera_id=2, x=200, y=150, radius=10, color=(255, 0, 0)),
        persistent=True
    )
    
    # Add custom drawing
    gui.add_draw_callback(example_custom_draw)
    
    try:
        gui.run()
    finally:
        camera_manager.stop()
        print("✅ GUI test completed")