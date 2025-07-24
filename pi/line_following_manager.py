#!/usr/bin/env python3
"""
Line Following Manager

Clean implementation based on legacy/src/line_following_controller.py
Uses the proven simple_chunk_detection algorithm.
"""

import cv2
import numpy as np
import time
import threading
import json
import os
from typing import Optional, Dict, List, Tuple
from camera_manager import CameraManager


def load_config(config_file="simple_chunk_config.json"):
    """Load configuration from JSON file."""
    default_config = {
        "detection": {
            "max_brightness": 80,
            "min_line_width_percent": 2.0,
            "search_width_percent": 30.0,
            "adaptive_center": True,
            "brown_filtering": True,
            "brown_threshold_percent": 50.0,
            "brown_rect_height_percent": 2.0
        },
        "sampling": {
            "num_rows": 11,
            "row_spacing_percent": 5.0,
            "start_from_bottom_percent": 0.0
        },
        "visualization": {
            "show_search_region": True,
            "show_all_detections": True,
            "show_center_line": True
        }
    }
    
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
                # Merge with defaults
                for section in default_config:
                    if section in config:
                        default_config[section].update(config[section])
                    else:
                        config[section] = default_config[section]
                return config
        except Exception as e:
            print(f"Error loading config: {e}")
            return default_config
    else:
        # Create default config file
        with open(config_file, 'w') as f:
            json.dump(default_config, f, indent=2)
        print(f"Created default config: {config_file}")
        return default_config


def detect_brown_percentage(image_region):
    """
    Detect the percentage of brown pixels in an image region.
    """
    if image_region.size == 0:
        return 0.0
    
    # Convert to HSV for better color detection
    hsv = cv2.cvtColor(image_region, cv2.COLOR_BGR2HSV)
    
    # Brown color ranges in HSV
    brown_ranges = [
        # Dark brown
        {'lower': np.array([10, 50, 20]), 'upper': np.array([20, 255, 200])},
        # Medium brown  
        {'lower': np.array([5, 30, 30]), 'upper': np.array([25, 180, 150])},
        # Light brown/tan
        {'lower': np.array([8, 20, 80]), 'upper': np.array([22, 120, 200])},
    ]
    
    # Create mask for brown pixels
    brown_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    
    for brown_range in brown_ranges:
        mask = cv2.inRange(hsv, brown_range['lower'], brown_range['upper'])
        brown_mask = cv2.bitwise_or(brown_mask, mask)
    
    # Calculate percentage
    total_pixels = image_region.shape[0] * image_region.shape[1]
    brown_pixels = np.sum(brown_mask > 0)
    brown_percentage = (brown_pixels / total_pixels) * 100 if total_pixels > 0 else 0
    
    return brown_percentage


def find_largest_black_chunk_multi_row(image, config, adaptive_center_x=None):
    """
    Find the largest black chunk across multiple configurable rows.
    Based on legacy simple_chunk_detection.py
    """
    height, width = image.shape[:2]
    
    # Get parameters from config
    max_brightness = config['detection']['max_brightness']
    min_line_width_percent = config['detection']['min_line_width_percent']
    search_width_percent = config['detection']['search_width_percent']
    brown_filtering = config['detection'].get('brown_filtering', True)
    brown_threshold = config['detection'].get('brown_threshold_percent', 50.0)
    brown_rect_height_percent = config['detection'].get('brown_rect_height_percent', 2.0)
    num_rows = config['sampling']['num_rows']
    row_spacing_percent = config['sampling']['row_spacing_percent']
    start_from_bottom_percent = config['sampling']['start_from_bottom_percent']
    
    # Calculate minimum line width in pixels
    min_line_width_pixels = int(width * min_line_width_percent / 100.0)
    
    # Calculate search region
    if adaptive_center_x is not None:
        center_x = adaptive_center_x
    else:
        center_x = width // 2
    
    search_width_pixels = int(width * search_width_percent / 100.0)
    search_left = max(0, center_x - search_width_pixels // 2)
    search_right = min(width, center_x + search_width_pixels // 2)
    
    # Calculate rows to check
    row_positions = []
    for i in range(num_rows):
        percent_from_bottom = start_from_bottom_percent + (i * row_spacing_percent)
        y_from_bottom = int(height * percent_from_bottom / 100.0)
        y = height - 1 - y_from_bottom
        
        if 0 <= y < height:
            row_positions.append((y, percent_from_bottom))
    
    all_detections = []
    best_detection = None
    
    for y, percent_from_bottom in row_positions:
        # Get row within search region
        row = image[y, search_left:search_right]
        
        # Convert to grayscale for speed
        gray_row = cv2.cvtColor(row.reshape(1, -1, 3), cv2.COLOR_BGR2GRAY)[0]
        
        # Find black pixels
        is_black = gray_row <= max_brightness
        
        # Find largest contiguous chunk
        chunks = []
        start = None
        
        for i, black in enumerate(is_black):
            if black and start is None:
                start = i
            elif not black and start is not None:
                chunks.append((start, i - start))
                start = None
        
        # Handle chunk at end
        if start is not None:
            chunks.append((start, len(is_black) - start))
        
        if chunks:
            # Find largest chunk in this row
            largest = max(chunks, key=lambda x: x[1])
            start_x, chunk_width = largest
            
            # Check minimum width
            if chunk_width >= min_line_width_pixels:
                # Convert to absolute coordinates
                abs_left_x = search_left + start_x
                abs_right_x = abs_left_x + chunk_width
                abs_center_x = (abs_left_x + abs_right_x) // 2
                
                # Apply brown filtering if enabled
                brown_percentage = 0.0
                if brown_filtering:
                    # Create rectangle with same width but configurable height
                    rect_height = max(1, int(height * brown_rect_height_percent / 100.0))
                    
                    # Center the rectangle around the detected line
                    rect_top = max(0, y - rect_height // 2)
                    rect_bottom = min(height, rect_top + rect_height)
                    
                    # Extract the rectangle region for brown analysis
                    rect_region = image[rect_top:rect_bottom, abs_left_x:abs_right_x]
                    brown_percentage = detect_brown_percentage(rect_region)
                    
                    # Only accept detection if brown content is below threshold
                    if brown_percentage > brown_threshold:
                        continue  # Skip this detection
                
                # Accept this detection
                detection = {
                    'found': True,
                    'center_x': abs_center_x,
                    'left_x': abs_left_x,
                    'right_x': abs_right_x,
                    'width': chunk_width,
                    'y': y,
                    'percent_from_bottom': percent_from_bottom,
                    'brown_percentage': brown_percentage
                }
                
                all_detections.append(detection)
                
                # Keep the largest chunk found so far
                if best_detection is None or chunk_width > best_detection['width']:
                    best_detection = detection
    
    if best_detection is None:
        return {
            'found': False,
            'center_x': None,
            'left_x': None,
            'right_x': None,
            'width': 0,
            'y': None,
            'detections': [],
            'search_region': {
                'left': search_left,
                'right': search_right,
                'center': center_x,
                'width': search_width_pixels
            },
            'detected_points': [],
            'lowest_point': None
        }
    
    # Find the lowest point (highest y value) for adaptive center
    lowest_detection = max(all_detections, key=lambda x: x['y'])
    
    # Create list of all detected points for custom postprocessing
    detected_points = []
    for det in all_detections:
        detected_points.append({
            'x': det['center_x'],
            'y': det['y'],
            'width': det['width'],
            'left_x': det['left_x'],
            'right_x': det['right_x'],
            'percent_from_bottom': det['percent_from_bottom'],
            'brown_percentage': det['brown_percentage']
        })
    
    # Sort points by y coordinate (bottom to top)
    detected_points.sort(key=lambda x: x['y'], reverse=True)
    
    # Return the best detection with all detections included
    result = best_detection.copy()
    result['detections'] = all_detections
    result['search_region'] = {
        'left': search_left,
        'right': search_right,
        'center': center_x,
        'width': search_width_pixels
    }
    result['detected_points'] = detected_points
    result['lowest_point'] = lowest_detection
    
    return result


class LineFollowingManager:
    """
    Line Following Manager - Based on legacy controller architecture.
    """
    
    def __init__(self, camera_manager: CameraManager, camera_id: int = 1, config_file="simple_chunk_config.json"):
        """Initialize line following manager."""
        self.camera_manager = camera_manager
        self.camera_id = camera_id
        self.config_file = config_file
        
        # Load configuration
        self.config = load_config(config_file)
        
        # State
        self.adaptive_center_x = None
        # Get adaptive_center setting from config
        self.adaptive_center_enabled = self.config.get('detection', {}).get('adaptive_center', True)
        self.detection_enabled = True
        self.last_detection = None
        
        print(f"🎯 Adaptive center: {'enabled' if self.adaptive_center_enabled else 'disabled'} (from config)")
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.last_detection_time = 0.0
        
        # Barrier detection state
        self.last_barrier_percentage = 0.0
        self.barrier_area_coords = None
        
        print(f"📏 Line Following Manager initialized for camera {camera_id}")
    
    def get_line(self) -> List[Dict]:
        """
        Detect line points across multiple rows with brown filtering.
        
        Returns:
            List of detected point dictionaries with position and metadata.
            X coordinates are returned as % of image width.
        """
        with self.lock:
            # Get frame from camera
            frame = self.camera_manager.get_frame(self.camera_id)
            if frame is None:
                return []
            
            self.frame_count += 1
            height, width = frame.shape[:2]
            
            # Detect line using legacy algorithm
            # Only pass adaptive center if enabled
            center_to_use = self.adaptive_center_x if self.adaptive_center_enabled else None
            detection = find_largest_black_chunk_multi_row(frame, self.config, center_to_use)
            
            # Update internal state
            self.last_detection = detection
            
            # Update adaptive center if enabled and line found
            if (self.adaptive_center_enabled and detection['found'] and 
                detection['lowest_point']):
                self.adaptive_center_x = detection['lowest_point']['center_x']
            
            # Return detected points with coordinates as % of image width
            if detection['found']:
                self.detection_count += 1
                self.last_detection_time = time.time()
                
                # Convert points to percentage coordinates
                points_percent = []
                for point in detection['detected_points']:
                    point_copy = point.copy()
                    point_copy['x_percent'] = (point['x'] / width) * 100.0
                    point_copy['left_x_percent'] = (point['left_x'] / width) * 100.0
                    point_copy['right_x_percent'] = (point['right_x'] / width) * 100.0
                    point_copy['width_percent'] = (point['width'] / width) * 100.0
                    points_percent.append(point_copy)
                
                return points_percent
            
            return []
    
    def get_barrier_percentage(self) -> float:
        """
        Calculate brown/barrier percentage in configured detection area.
        
        Returns:
            float: Percentage of brown pixels in barrier detection area (0-100)
        """
        with self.lock:
            # Get frame from camera
            frame = self.camera_manager.get_frame(self.camera_id)
            if frame is None:
                return 0.0
            
            # Check if barrier detection is enabled
            barrier_config = self.config.get('barrier_detection', {})
            if not barrier_config.get('enabled', True):
                return 0.0
            
            # Calculate barrier detection area coordinates
            self.barrier_area_coords = self._calculate_barrier_area(frame.shape)
            
            # Extract the barrier detection region
            y1, y2, x1, x2 = self.barrier_area_coords
            barrier_region = frame[y1:y2, x1:x2]
            
            # Calculate brown percentage in this region
            brown_percentage = detect_brown_percentage(barrier_region)
            
            # Store for later use
            self.last_barrier_percentage = brown_percentage
            
            return brown_percentage
    
    def get_barrier_area_coords(self) -> Optional[Tuple[int, int, int, int]]:
        """Get the current barrier detection area coordinates (y1, y2, x1, x2)."""
        return self.barrier_area_coords
    
    def _calculate_barrier_area(self, frame_shape: Tuple[int, int, int]) -> Tuple[int, int, int, int]:
        """
        Calculate barrier detection area coordinates based on configuration.
        
        Args:
            frame_shape: (height, width, channels) of the frame
            
        Returns:
            Tuple of (y1, y2, x1, x2) coordinates
        """
        height, width = frame_shape[:2]
        barrier_config = self.config.get('barrier_detection', {})
        
        # Get configuration parameters
        area_width_percent = barrier_config.get('area_width_percent', 40.0)
        area_height_percent = barrier_config.get('area_height_percent', 20.0)
        center_x_percent = barrier_config.get('center_x_percent', 50.0)
        bottom_offset_percent = barrier_config.get('bottom_offset_percent', 5.0)
        
        # Calculate area dimensions in pixels
        area_width_px = int(width * area_width_percent / 100.0)
        area_height_px = int(height * area_height_percent / 100.0)
        
        # Calculate center position
        center_x = int(width * center_x_percent / 100.0)
        
        # Calculate vertical position (from bottom)
        bottom_offset_px = int(height * bottom_offset_percent / 100.0)
        
        # Calculate boundaries
        x1 = max(0, center_x - area_width_px // 2)
        x2 = min(width, center_x + area_width_px // 2)
        y2 = height - bottom_offset_px  # Bottom edge
        y1 = max(0, y2 - area_height_px)  # Top edge
        
        return (y1, y2, x1, x2)
    
    def get_detection_status(self):
        """Get current detection status and data."""
        with self.lock:
            points = []
            if self.last_detection and self.last_detection['detected_points']:
                points = self.last_detection['detected_points'].copy()
            
            return {
                'line_found': self.last_detection['found'] if self.last_detection else False,
                'points': points,
                'search_center': self.adaptive_center_x,
                'detection_enabled': self.detection_enabled,
                'adaptive_center_enabled': self.adaptive_center_enabled,
                'frame_count': self.frame_count,
                'detection_count': self.detection_count,
                'last_detection_time': self.last_detection_time,
                'barrier_percentage': self.last_barrier_percentage,
                'barrier_area_coords': self.barrier_area_coords
            }
    
    def reset_adaptive_center(self):
        """Reset adaptive center to image center."""
        with self.lock:
            self.adaptive_center_x = None
            print("🎯 Adaptive center reset")
    
    def set_adaptive_center_enabled(self, enabled: bool):
        """Enable or disable adaptive center."""
        with self.lock:
            self.adaptive_center_enabled = enabled
            if not enabled:
                # Reset center when disabling
                self.adaptive_center_x = None
            print(f"🎯 Adaptive center {'enabled' if enabled else 'disabled'}")
    
    def set_search_width(self, width_percent: float):
        """Set search width percentage."""
        with self.lock:
            self.config['detection']['search_width_percent'] = max(10.0, min(100.0, width_percent))
            print(f"🔍 Search width set to: {width_percent}%")
    
    def set_camera_id(self, camera_id: int):
        """Change camera being used."""
        with self.lock:
            self.camera_id = camera_id
            print(f"📷 Camera changed to: {camera_id}")
    
    def reload_config(self):
        """Reload configuration from file."""
        with self.lock:
            self.config = load_config(self.config_file)
            # Update adaptive center setting from new config
            self.adaptive_center_enabled = self.config.get('detection', {}).get('adaptive_center', True)
            print(f"📝 Configuration reloaded from {self.config_file}")
    
    def get_parameters(self) -> Dict:
        """Get current detection parameters."""
        with self.lock:
            params = self.config.copy()
            params.update({
                'camera_id': self.camera_id,
                'adaptive_center_x': self.adaptive_center_x,
                'adaptive_center_enabled': self.adaptive_center_enabled,
                'frame_count': self.frame_count,
                'detection_count': self.detection_count,
                'last_detection_time': self.last_detection_time
            })
            return params


if __name__ == "__main__":
    """Test line following manager."""
    print("🧪 Testing Line Following Manager...")
    
    # Create camera manager
    camera_manager = CameraManager()
    camera_manager.start()
    
    # Create line following manager
    line_manager = LineFollowingManager(camera_manager, camera_id=1)
    
    try:
        for i in range(50):  # Test 50 detections
            points = line_manager.get_line()
            
            if points:
                print(f"Frame {i}: {len(points)} points detected")
                if points:
                    closest = max(points, key=lambda p: p['y'])
                    print(f"  Closest: x={closest['x']}, y={closest['y']}, width={closest['width']}")
            else:
                print(f"Frame {i}: No line detected")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted")
    finally:
        camera_manager.stop()
        print("✅ Test completed")