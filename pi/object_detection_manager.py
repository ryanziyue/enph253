#!/usr/bin/env python3
"""
Object Detection Manager

Clean implementation of YOLO World object detection based on legacy system.
Integrates with camera manager and provides crop functionality.
"""

import cv2
import numpy as np
import time
import threading
import json
import os
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from camera_manager import CameraManager

# Try to import YOLO dependencies
try:
    from ultralytics import YOLOWorld, YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ ultralytics not available. Install with: pip install ultralytics")


@dataclass
class DetectionResult:
    """Result from object detection."""
    bbox: Optional[Tuple[int, int, int, int]]  # (x1, y1, x2, y2) in original image coordinates
    confidence: float
    class_name: str
    found: bool
    timestamp: float
    # Additional fields for integration
    bbox_percent: Optional[Tuple[float, float, float, float]] = None  # Bbox as % of image size


def load_detection_config(config_file="object_detection_config.json"):
    """Load configuration from JSON file."""
    default_config = {
        "model": {
            "model_type": "yolo-world",  # Options: "yolo-world", "yolov11", "yolov8"
            "model_file": "yolov8s-worldv2.pt",
            "confidence_threshold": 0.1,
            "classes": ["plush toy"]
        },
        "processing": {
            "input_width": 320,
            "input_height": 160
        },
        "crop": {
            "crop_top": 0,
            "crop_bottom": 0,
            "crop_left": 0,
            "crop_right": 0
        },
        "detection": {
            "detection_interval": 1.0,
            "max_detections": 5
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


class ObjectDetectionManager:
    """
    Object Detection Manager - Clean implementation based on legacy YOLO system.
    """
    
    def __init__(self, camera_manager: CameraManager, camera_id: int = 2, config_file="object_detection_config.json"):
        """Initialize object detection manager."""
        self.camera_manager = camera_manager
        self.camera_id = camera_id
        self.config_file = config_file
        
        # Load configuration
        self.config = load_detection_config(config_file)
        
        # Model state
        self.model = None
        self.model_loaded = False
        self.class_filter = None  # For standard YOLO models
        
        # Detection state
        self.last_detection = DetectionResult(None, 0.0, "", False, 0.0)
        self.last_raw_frame = None
        self.last_cropped_frame = None
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Statistics
        self.detection_count = 0
        self.last_detection_time = 0.0
        
        # Async detection state
        self.detection_thread = None
        self.detection_in_progress = False
        
        # Initialize model
        self._load_model()
        
        print(f"🎯 Object Detection Manager initialized for camera {camera_id}")
    
    def _load_model(self):
        """Load the YOLO model based on configuration."""
        if not YOLO_AVAILABLE:
            print("❌ YOLO not available, object detection disabled")
            return
            
        try:
            model_type = self.config['model'].get('model_type', 'yolo-world')
            model_file = self.config['model']['model_file']
            
            # Check if model file exists
            if not os.path.exists(model_file):
                print(f"⚠️ Model file not found: {model_file}")
                print(f"   Please ensure the model file is in the correct location")
                self.model_loaded = False
                return
            
            print(f"📦 Loading {model_type} model: {model_file}")
            
            # Load model based on type
            if model_type == 'yolo-world':
                self.model = YOLOWorld(model_file)
                classes = self.config['model']['classes']
                self.model.set_classes(classes)
                print(f"✅ Model loaded successfully with classes: {classes}")
            elif model_type in ['yolov11', 'yolov11n', 'yolov8']:
                # Standard YOLO models (v8, v11, etc.)
                self.model = YOLO(model_file)
                print(f"✅ {model_type.upper()} model loaded successfully")
                # Note: Standard YOLO models use pre-trained classes
                if 'classes' in self.config['model'] and self.config['model']['classes']:
                    print(f"   ⚠️ Note: Standard YOLO models use COCO classes")
                    print(f"   Configured class filter: {self.config['model']['classes']}")
                    self.class_filter = self.config['model']['classes']
                else:
                    self.class_filter = None
            else:
                raise ValueError(f"Unknown model type: {model_type}")
            
            self.model_loaded = True
            
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            self.model_loaded = False
    
    def setup_crop(self, crop_top: int = None, crop_bottom: int = None, 
                   crop_left: int = None, crop_right: int = None):
        """
        Set up the crop region for object detection.
        
        Args:
            crop_top: Pixels to crop from top
            crop_bottom: Pixels to crop from bottom  
            crop_left: Pixels to crop from left
            crop_right: Pixels to crop from right
        """
        with self.lock:
            if crop_top is not None:
                self.config['crop']['crop_top'] = max(0, crop_top)
            if crop_bottom is not None:
                self.config['crop']['crop_bottom'] = max(0, crop_bottom)
            if crop_left is not None:
                self.config['crop']['crop_left'] = max(0, crop_left)
            if crop_right is not None:
                self.config['crop']['crop_right'] = max(0, crop_right)
            
            print(f"🔍 Crop updated: top={self.config['crop']['crop_top']}, "
                  f"bottom={self.config['crop']['crop_bottom']}, "
                  f"left={self.config['crop']['crop_left']}, "
                  f"right={self.config['crop']['crop_right']}")
    
    def _apply_crop(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        Apply crop to frame and return cropped frame with crop info.
        
        Returns:
            Tuple of (cropped_frame, crop_info_dict)
        """
        height, width = frame.shape[:2]
        
        # Get crop parameters
        crop_top = self.config['crop']['crop_top']
        crop_bottom = self.config['crop']['crop_bottom']
        crop_left = self.config['crop']['crop_left']
        crop_right = self.config['crop']['crop_right']
        
        # Calculate crop boundaries
        top = max(0, crop_top)
        bottom = height - max(0, crop_bottom) if crop_bottom > 0 else height
        left = max(0, crop_left)
        right = width - max(0, crop_right) if crop_right > 0 else width
        
        # Ensure valid crop region
        if top >= bottom or left >= right:
            print(f"⚠️ Invalid crop region, using full frame")
            top, bottom, left, right = 0, height, 0, width
        
        # Apply crop
        cropped_frame = frame[top:bottom, left:right]
        
        crop_info = {
            'top': top,
            'bottom': bottom,
            'left': left,
            'right': right,
            'original_width': width,
            'original_height': height,
            'cropped_width': right - left,
            'cropped_height': bottom - top
        }
        
        return cropped_frame, crop_info
    
    def _resize_for_model(self, frame: np.ndarray) -> np.ndarray:
        """Resize frame for model input."""
        target_width = self.config['processing']['input_width']
        target_height = self.config['processing']['input_height']
        
        return cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
    
    def run_detection_async(self) -> DetectionResult:
        """
        Run object detection asynchronously (non-blocking).
        Returns immediately with the last result, starts new detection in background if not already running.
        
        Returns:
            DetectionResult with highest confidence detection (may be from previous run)
        """
        # Always return the current result immediately (non-blocking)
        with self.lock:
            current_result = self.last_detection
        
        # Start new detection in background if not already running
        if not self.detection_in_progress:
            self.detection_in_progress = True
            self.detection_thread = threading.Thread(target=self._run_detection_background, daemon=True)
            self.detection_thread.start()
        
        return current_result
    
    def _run_detection_background(self):
        """Run detection in background thread and update results."""
        try:
            new_result = self._run_detection_sync()
            with self.lock:
                self.last_detection = new_result
                if new_result.found:
                    self.detection_count += 1
                    self.last_detection_time = new_result.timestamp
        except Exception as e:
            print(f"❌ Background detection error: {e}")
            error_result = DetectionResult(None, 0.0, f"error: {str(e)}", False, time.time())
            with self.lock:
                self.last_detection = error_result
        finally:
            self.detection_in_progress = False
    
    def _run_detection_sync(self) -> DetectionResult:
        """
        Run object detection synchronously (called from detection thread).
        
        Returns:
            DetectionResult with highest confidence detection
        """
        if not self.model_loaded:
            return DetectionResult(None, 0.0, "model_not_loaded", False, time.time())
        
        # Get frame from camera (this should be fast)
        frame = self.camera_manager.get_frame(self.camera_id)
        if frame is None:
            return DetectionResult(None, 0.0, "no_frame", False, time.time())
        
        # Copy frame data before releasing any locks
        frame_copy = frame.copy()
        
        with self.lock:
            self.last_raw_frame = frame_copy.copy()
            
            # Apply crop
            cropped_frame, crop_info = self._apply_crop(frame_copy)
            self.last_cropped_frame = cropped_frame.copy()
        
        # Resize for model (outside lock)
        model_frame = self._resize_for_model(cropped_frame)
        
        try:
            # Run inference (this is the slow part, outside any locks)
            confidence_threshold = self.config['model']['confidence_threshold']
            results = self.model.predict(model_frame, 
                                       imgsz=(self.config['processing']['input_height'], 
                                              self.config['processing']['input_width']),
                                       conf=confidence_threshold,
                                       verbose=False)
            
            result = results[0]
            timestamp = time.time()
            
            # Find highest confidence detection (with optional class filtering)
            if result.boxes is not None and len(result.boxes) > 0:
                best_detection = None
                best_conf = 0.0
                model_type = self.config['model'].get('model_type', 'yolo-world')
                
                for i, box in enumerate(result.boxes):
                    conf = float(box.conf[0].cpu().numpy())
                    cls_idx = int(box.cls[0].cpu().numpy())
                    
                    # Get class name based on model type
                    if model_type == 'yolo-world':
                        # YOLO World uses custom classes
                        classes = self.config['model']['classes']
                        class_name = classes[cls_idx] if cls_idx < len(classes) else "unknown"
                    else:
                        # Standard YOLO models use COCO class names
                        if hasattr(result, 'names') and cls_idx in result.names:
                            class_name = result.names[cls_idx]
                        else:
                            # Fallback to model's class names
                            class_name = self.model.names.get(cls_idx, f"class_{cls_idx}")
                        
                        # Apply class filter if configured
                        if hasattr(self, 'class_filter') and self.class_filter:
                            if class_name not in self.class_filter:
                                # Skip this detection if not in filter
                                continue
                    
                    # Check if this is the best detection so far
                    if conf > best_conf:
                        best_conf = conf
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        best_detection = {
                            'box': (x1, y1, x2, y2),
                            'confidence': conf,
                            'class_name': class_name
                        }
                
                # Process best detection if found
                if best_detection is not None:
                    x1, y1, x2, y2 = best_detection['box']
                    confidence = best_detection['confidence']
                    class_name = best_detection['class_name']
                    
                    # Scale bbox back to cropped frame coordinates
                    model_height, model_width = model_frame.shape[:2]
                    scale_x = crop_info['cropped_width'] / model_width
                    scale_y = crop_info['cropped_height'] / model_height
                    
                    # Scale to cropped frame
                    x1_crop = int(x1 * scale_x)
                    y1_crop = int(y1 * scale_y)
                    x2_crop = int(x2 * scale_x)
                    y2_crop = int(y2 * scale_y)
                    
                    # Convert to original frame coordinates
                    x1_orig = x1_crop + crop_info['left']
                    y1_orig = y1_crop + crop_info['top']
                    x2_orig = x2_crop + crop_info['left']
                    y2_orig = y2_crop + crop_info['top']
                    
                    bbox = (x1_orig, y1_orig, x2_orig, y2_orig)
                    
                    # Calculate bbox as percentage of original frame
                    bbox_percent = (
                        (x1_orig / crop_info['original_width']) * 100,
                        (y1_orig / crop_info['original_height']) * 100,
                        (x2_orig / crop_info['original_width']) * 100,
                        (y2_orig / crop_info['original_height']) * 100
                    )
                    
                    detection = DetectionResult(bbox, confidence, class_name, True, timestamp, bbox_percent)
                else:
                    # No valid detections found (filtered out by class filter)
                    detection = DetectionResult(None, 0.0, "no_detection", False, timestamp)
            else:
                detection = DetectionResult(None, 0.0, "no_detection", False, timestamp)
            
            # Update statistics (with lock)
            with self.lock:
                self.last_detection = detection
                if detection.found:
                    self.detection_count += 1
                    self.last_detection_time = timestamp
            
            return detection
            
        except Exception as e:
            print(f"❌ Detection error: {e}")
            error_result = DetectionResult(None, 0.0, f"error: {str(e)}", False, time.time())
            with self.lock:
                self.last_detection = error_result
            return error_result
    
    def run_detection(self) -> DetectionResult:
        """
        Run detection asynchronously when called by robot controller.
        Returns immediately with latest result, triggers new detection in background.
        
        Returns:
            DetectionResult with highest confidence detection
        """
        return self.run_detection_async()
    
    def run_detection_blocking(self, timeout: float = 2.0) -> DetectionResult:
        """
        Run object detection synchronously (blocking).
        Waits for a fresh detection result from the current camera position.
        
        Args:
            timeout: Maximum time to wait for detection result (seconds)
            
        Returns:
            DetectionResult from fresh detection run
        """
        if not self.model_loaded:
            print("❌ Model not loaded")
            return DetectionResult(None, 0.0, "model_not_loaded", False, time.time())
        
        # Record when we started this detection request
        start_time = time.time()
        
        # If there's already a detection in progress, wait for it to complete
        wait_start = time.time()
        while self.detection_in_progress and (time.time() - wait_start) < timeout:
            time.sleep(0.01)
        
        # Start a fresh detection
        detection_start = time.time()
        self.detection_in_progress = True
        
        try:
            # Run detection synchronously
            result = self._run_detection_sync()
            
            # Update the cached result
            with self.lock:
                self.last_detection = result
                if result.found:
                    self.detection_count += 1
                    self.last_detection_time = result.timestamp
            
            return result
            
        except Exception as e:
            print(f"❌ Blocking detection error: {e}")
            error_result = DetectionResult(None, 0.0, f"error: {str(e)}", False, time.time())
            with self.lock:
                self.last_detection = error_result
            return error_result
            
        finally:
            self.detection_in_progress = False
    
    def get_last_detection(self) -> DetectionResult:
        """
        Get the last detection result without triggering new detection.
        
        Returns:
            DetectionResult from last detection run
        """
        with self.lock:
            return self.last_detection
    
    def get_detection_status(self) -> Dict:
        """Get current detection status and data."""
        with self.lock:
            return {
                'detection_found': self.last_detection.found,
                'last_detection': self.last_detection,
                'detection_count': self.detection_count,
                'last_detection_time': self.last_detection_time,
                'model_loaded': self.model_loaded,
                'camera_id': self.camera_id,
                'crop_settings': self.config['crop'].copy()
            }
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get the last raw and cropped frames for visualization.
        
        Returns:
            Tuple of (raw_frame, cropped_frame)
        """
        with self.lock:
            return self.last_raw_frame, self.last_cropped_frame
    
    def set_classes(self, classes: List[str]):
        """Update detection classes."""
        if not self.model_loaded:
            return False
        
        try:
            self.config['model']['classes'] = classes
            self.model.set_classes(classes)
            print(f"📝 Classes updated: {classes}")
            return True
        except Exception as e:
            print(f"❌ Failed to update classes: {e}")
            return False
    
    def set_confidence_threshold(self, threshold: float):
        """Update confidence threshold."""
        with self.lock:
            self.config['model']['confidence_threshold'] = max(0.0, min(1.0, threshold))
            print(f"🎯 Confidence threshold set to: {threshold}")
    
    def set_camera_id(self, camera_id: int):
        """Change camera being used."""
        with self.lock:
            self.camera_id = camera_id
            print(f"📷 Camera changed to: {camera_id}")
    
    def get_parameters(self) -> Dict:
        """Get current detection parameters."""
        with self.lock:
            params = self.config.copy()
            params.update({
                'camera_id': self.camera_id,
                'model_loaded': self.model_loaded,
                'detection_count': self.detection_count,
                'last_detection_time': self.last_detection_time,
                'detection_in_progress': self.detection_in_progress
            })
            return params
    
    def cleanup(self):
        """Clean up detection manager resources."""
        print("🧹 Cleaning up object detection manager...")
        # Wait for any ongoing detection to complete
        if self.detection_thread and self.detection_thread.is_alive():
            self.detection_thread.join(timeout=2.0)
            if self.detection_thread.is_alive():
                print("⚠️ Detection thread did not stop cleanly")


if __name__ == "__main__":
    """Test object detection manager."""
    print("🧪 Testing Object Detection Manager...")
    
    # Create camera manager
    camera_manager = CameraManager()
    camera_manager.start()
    
    # Create object detection manager
    detection_manager = ObjectDetectionManager(camera_manager, camera_id=2)
    
    try:
        for i in range(10):  # Test 10 detections
            result = detection_manager.run_detection()
            
            if result.found:
                print(f"Detection {i}: {result.class_name} at {result.bbox} with confidence {result.confidence:.3f}")
            else:
                print(f"Detection {i}: No object detected ({result.class_name})")
            
            time.sleep(1.0)  # 1 second interval
    
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted")
    finally:
        camera_manager.stop()
        print("✅ Test completed")