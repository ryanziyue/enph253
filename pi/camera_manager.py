#!/usr/bin/env python3
"""
Camera Manager - Cross-Platform Dual Camera Feed Management

Manages two camera feeds with platform-appropriate backends:
- Linux: V4L2 backend (optimized)
- Windows: DirectShow backend  
- macOS: AVFoundation backend
- Fallback: OpenCV default backend

Provides thread-safe access to camera frames with configurable parameters.
"""

import cv2
import numpy as np
import json
import time
import threading
import signal
import sys
import platform
import os
import glob
from typing import Optional, Dict, Tuple, List, Union
from dataclasses import dataclass


@dataclass
class CameraStatus:
    """Camera status information."""
    connected: bool = False
    fps: float = 0.0
    frame_count: int = 0
    last_frame_time: float = 0.0
    error_count: int = 0
    last_error: str = ""


def resolve_device_identifier(device_identifier: Union[int, str]) -> int:
    """
    Resolve device identifier to OpenCV-compatible device ID.
    
    Args:
        device_identifier: Either an integer device ID or a string device name/path
        
    Returns:
        int: OpenCV device ID
        
    Raises:
        ValueError: If device identifier cannot be resolved
    """
    # If it's already an integer, return as-is
    if isinstance(device_identifier, int):
        return device_identifier
    
    # If it's a string, try to resolve it
    if isinstance(device_identifier, str):
        # Check if it's a numeric string
        if device_identifier.isdigit():
            return int(device_identifier)
        
        # On Linux, try to resolve device name to /dev/video* path
        if platform.system().lower() == 'linux':
            return _resolve_linux_device_name(device_identifier)
        
        # On other platforms, try to parse as direct path
        if device_identifier.startswith('/dev/video'):
            try:
                return int(device_identifier.replace('/dev/video', ''))
            except ValueError:
                pass
        
        # If all else fails, try to use it directly as a path
        # Some OpenCV builds support string paths
        try:
            # Test if OpenCV can open it directly
            test_cap = cv2.VideoCapture(device_identifier)
            if test_cap.isOpened():
                test_cap.release()
                return device_identifier  # Return string if it works
        except:
            pass
    
    raise ValueError(f"Could not resolve device identifier: {device_identifier}")

def _resolve_linux_device_name(device_name: str) -> int:
    """
    Resolve Linux device name to /dev/video* device ID.
    
    Args:
        device_name: Device name like "usb-xhci-hcd.0-1"
        
    Returns:
        int: Device ID number
        
    Raises:
        ValueError: If device name cannot be resolved
    """
    try:
        # Look in /sys/class/video4linux/ for device links
        video_devices = glob.glob('/sys/class/video4linux/video*')
        
        for video_path in video_devices:
            try:
                # Read the device symlink to get the actual device path
                device_link = os.readlink(video_path)
                
                # Check if our device name appears in the link path
                if device_name in device_link:
                    # Extract video device number
                    video_num = os.path.basename(video_path).replace('video', '')
                    device_id = int(video_num)
                    print(f"📷 Resolved device '{device_name}' to /dev/video{device_id}")
                    return device_id
                    
            except (OSError, ValueError) as e:
                # Skip devices we can't read or parse
                continue
        
        # Alternative method: check udev info
        return _resolve_linux_device_udev(device_name)
        
    except Exception as e:
        raise ValueError(f"Could not resolve Linux device name '{device_name}': {e}")

def _resolve_linux_device_udev(device_name: str) -> int:
    """
    Alternative method using udev information to resolve device names.
    
    Args:
        device_name: Device name like "usb-xhci-hcd.0-1"
        
    Returns:
        int: Device ID number
        
    Raises:
        ValueError: If device name cannot be resolved
    """
    try:
        import subprocess
        
        # Use v4l2-ctl to list devices if available
        try:
            result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                found_device = False
                
                for i, line in enumerate(lines):
                    if device_name in line:
                        found_device = True
                        continue
                    
                    # Look for /dev/video* in subsequent lines
                    if found_device and '/dev/video' in line:
                        video_path = line.strip()
                        device_id = int(video_path.replace('/dev/video', ''))
                        print(f"📷 Resolved device '{device_name}' to {video_path} (ID: {device_id})")
                        return device_id
                        
        except (subprocess.SubprocessError, FileNotFoundError):
            pass
        
        # Last resort: try lsusb and map to video devices
        video_devices = glob.glob('/dev/video*')
        for video_dev in sorted(video_devices):
            try:
                device_id = int(video_dev.replace('/dev/video', ''))
                # Try to open and check if it matches somehow
                # This is a basic fallback - not perfect but better than failing
                if device_id < 10:  # Reasonable range
                    print(f"⚠️ Using fallback mapping for '{device_name}' -> {video_dev} (ID: {device_id})")
                    return device_id
            except ValueError:
                continue
                
    except Exception:
        pass
    
    raise ValueError(f"Could not resolve device name '{device_name}' using any method")


class CameraHandle:
    """Individual camera wrapper with cross-platform backend optimization."""
    
    def __init__(self, camera_id: int, config: Dict):
        """Initialize camera handle with configuration."""
        self.camera_id = camera_id
        self.config = config
        self.cap = None
        self.status = CameraStatus()
        self.lock = threading.Lock()
        self.latest_frame = None
        
        # Performance tracking
        self.frame_times = []
        self.warmup_count = 0
        
        # Downsampling
        self.downsample_factor = config.get('downsample_factor', 1.0)
        
        # Device resolution
        self.resolved_device_id = None
        
        # Homography transformation
        self.homography_enabled = False
        self.homography_matrix = None
        self._setup_homography()
        
        # Threading for continuous capture
        self.capture_thread = None
        self.capture_running = False
        self.frame_lock = threading.Lock()  # Separate lock for frame access
        self.latest_raw_frame = None
        self.latest_processed_frame = None
        self.last_capture_time = 0.0
        
        device_display = config.get('device_id', 'unknown')
        print(f"📷 Initializing camera {camera_id} with device {device_display}")
    
    def _setup_homography(self):
        """Setup homography transformation if enabled in config."""
        homography_config = self.config.get('homography', {})
        self.homography_enabled = homography_config.get('enabled', False)
        
        if self.homography_enabled:
            try:
                # Check if angle-based configuration is used
                if 'correction_angle' in homography_config:
                    # Use angle-based correction
                    angle = homography_config.get('correction_angle', 0.0)
                    src_points, dst_points = self._calculate_homography_from_angle(angle)
                    print(f"✅ Homography enabled for camera {self.camera_id} with angle correction: {angle}°")
                else:
                    # Use traditional point-based configuration
                    src_points = np.array(homography_config.get('source_points', []), dtype=np.float32)
                    dst_points = np.array(homography_config.get('destination_points', []), dtype=np.float32)
                    
                    # Validate points
                    if src_points.shape != (4, 2) or dst_points.shape != (4, 2):
                        print(f"⚠️ Invalid homography points for camera {self.camera_id}, disabling homography")
                        self.homography_enabled = False
                        return
                    
                    print(f"✅ Homography enabled for camera {self.camera_id}")
                    print(f"   Source points: {src_points.tolist()}")
                    print(f"   Destination points: {dst_points.tolist()}")
                
                # Calculate homography matrix
                self.homography_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                
            except Exception as e:
                print(f"❌ Failed to setup homography for camera {self.camera_id}: {e}")
                self.homography_enabled = False
    
    def _calculate_homography_from_angle(self, angle_degrees: float):
        """
        Calculate homography points from camera tilt angle.
        Assumes camera is looking downward at an angle.
        
        Args:
            angle_degrees: Camera tilt angle (0 = perpendicular, positive = tilted forward)
            
        Returns:
            tuple: (source_points, destination_points) as numpy arrays
        """
        # Get camera resolution from config
        width, height = self.config.get('resolution', [640, 480])
        
        # Convert angle to radians
        angle_rad = np.radians(angle_degrees)
        
        # Define perspective correction based on camera angle
        # The further from camera, the more compressed the view becomes
        
        # Top edge appears closer to horizon due to tilt
        top_compression = np.tan(angle_rad) * 0.3  # Adjust factor as needed
        
        # Source points (what the tilted camera sees - trapezoid)
        margin_x = width * 0.1  # 10% margin on sides
        margin_y = height * 0.1  # 10% margin on top/bottom
        
        # Top edge is compressed (narrower) due to perspective
        top_width_reduction = width * top_compression * 0.5
        
        src_points = np.array([
            [margin_x + top_width_reduction, margin_y],                    # Top-left
            [width - margin_x - top_width_reduction, margin_y],            # Top-right  
            [width - margin_x, height - margin_y],                        # Bottom-right
            [margin_x, height - margin_y]                                 # Bottom-left
        ], dtype=np.float32)
        
        # Destination points (bird's eye view - rectangle)
        dst_points = np.array([
            [margin_x, margin_y],                    # Top-left
            [width - margin_x, margin_y],            # Top-right
            [width - margin_x, height - margin_y],   # Bottom-right
            [margin_x, height - margin_y]            # Bottom-left
        ], dtype=np.float32)
        
        return src_points, dst_points
    
    def _fourcc_from_string(self, fourcc_str: str) -> int:
        """Convert FOURCC string to OpenCV format."""
        if len(fourcc_str) != 4:
            return cv2.VideoWriter_fourcc(*'MJPG')
        return cv2.VideoWriter_fourcc(*fourcc_str)
    
    def connect(self) -> bool:
        """Connect to camera with platform-appropriate backend."""
        try:
            # Release existing connection
            if self.cap is not None:
                self.cap.release()
            
            # Resolve device identifier to OpenCV-compatible format
            device_identifier = self.config['device_id']
            try:
                self.resolved_device_id = resolve_device_identifier(device_identifier)
                print(f"📷 Resolved device '{device_identifier}' to ID: {self.resolved_device_id}")
            except ValueError as e:
                print(f"❌ Device resolution failed: {e}")
                self.status.error_count += 1
                self.status.last_error = str(e)
                return False
            
            # Create VideoCapture with platform-appropriate backend
            device_id = self.resolved_device_id
            
            # Choose backend based on platform
            system = platform.system().lower()
            if system == 'linux':
                # Use V4L2 on Linux
                self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
                print(f"📷 Using V4L2 backend for camera {self.camera_id} on Linux")
            elif system == 'windows':
                # Use DirectShow on Windows
                self.cap = cv2.VideoCapture(device_id, cv2.CAP_DSHOW)
                print(f"📷 Using DirectShow backend for camera {self.camera_id} on Windows")
            elif system == 'darwin':  # macOS
                # Use AVFoundation on macOS
                self.cap = cv2.VideoCapture(device_id, cv2.CAP_AVFOUNDATION)
                print(f"📷 Using AVFoundation backend for camera {self.camera_id} on macOS")
            else:
                # Fallback to default backend
                self.cap = cv2.VideoCapture(device_id)
                print(f"📷 Using default backend for camera {self.camera_id} on {system}")
            
            # If specific backend fails, try default
            if not self.cap.isOpened() and system != 'unknown':
                print(f"⚠️ Platform-specific backend failed, trying default...")
                self.cap.release()
                self.cap = cv2.VideoCapture(device_id)
            
            if not self.cap.isOpened():
                self.status.error_count += 1
                self.status.last_error = f"Failed to open camera device {device_identifier} (resolved to {device_id})"
                return False
            
            # IMPORTANT: Set FOURCC first for proper format negotiation
            # This must be done before setting resolution/FPS on many cameras
            fourcc = self._fourcc_from_string(self.config['fourcc'])
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            
            # Small delay to let format change take effect
            time.sleep(0.1)
            
            # Now configure resolution and FPS
            width, height = self.config['resolution']
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.config['buffer_size'])
            
            # Set FPS last - some drivers need format and resolution set first
            self.cap.set(cv2.CAP_PROP_FPS, self.config['target_fps'])
            
            # Set image adjustment properties
            if self.config['brightness'] != 0:
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.config['brightness'])
            if self.config['contrast'] != 0:
                self.cap.set(cv2.CAP_PROP_CONTRAST, self.config['contrast'])
            if self.config['saturation'] != 0:
                self.cap.set(cv2.CAP_PROP_SATURATION, self.config['saturation'])
            
            # Exposure settings
            if self.config['auto_exposure']:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
                if self.config['exposure_time'] > 0:
                    self.cap.set(cv2.CAP_PROP_EXPOSURE, self.config['exposure_time'])
            
            # White balance settings
            if self.config['auto_white_balance']:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
                self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.config['white_balance_temp'])
            
            # Gain setting
            if self.config['gain'] != 0:
                self.cap.set(cv2.CAP_PROP_GAIN, self.config['gain'])
            
            # Verify actual settings
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_fourcc = self.cap.get(cv2.CAP_PROP_FOURCC)
            
            # Convert FOURCC code back to string for display
            fourcc_str = "".join([chr((int(actual_fourcc) >> (8 * i)) & 0xFF) for i in range(4)])
            
            print(f"✅ Camera {self.camera_id} connected:")
            print(f"   📱 Device: {device_identifier} → ID {self.resolved_device_id}")
            print(f"   📐 Resolution: {actual_width}x{actual_height} (requested: {width}x{height})")
            print(f"   🎞️ FPS: {actual_fps} (requested: {self.config['target_fps']})")
            print(f"   🔧 FOURCC: {fourcc_str} (requested: {self.config['fourcc']})")
            print(f"   📦 Buffer size: {self.config['buffer_size']}")
            print(f"   🔍 Downsample: {self.downsample_factor}x")
            
            # FPS warning if significantly different
            if actual_fps > 0 and abs(actual_fps - self.config['target_fps']) > 5:
                print(f"   ⚠️ FPS mismatch! Check camera format support")
            
            self.status.connected = True
            self.status.error_count = 0
            self.status.last_error = ""
            self.warmup_count = 0
            
            # Start capture thread
            self._start_capture_thread()
            
            return True
            
        except Exception as e:
            self.status.error_count += 1
            self.status.last_error = str(e)
            print(f"❌ Camera {self.camera_id} connection failed: {e}")
            return False
    
    def _start_capture_thread(self):
        """Start the background capture thread."""
        if self.capture_thread is not None and self.capture_thread.is_alive():
            return
        
        self.capture_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        print(f"🎞️ Camera {self.camera_id} capture thread started")
    
    def _stop_capture_thread(self):
        """Stop the background capture thread."""
        self.capture_running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        print(f"🎞️ Camera {self.camera_id} capture thread stopped")
    
    def _capture_loop(self):
        """Continuous capture loop running in background thread."""
        consecutive_failures = 0
        max_failures = 10
        
        while self.capture_running and self.status.connected:
            try:
                with self.lock:
                    if self.cap is None or not self.cap.isOpened():
                        break
                    
                    ret, frame = self.cap.read()
                
                if not ret or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= max_failures:
                        self.status.error_count += 1
                        self.status.last_error = f"Too many consecutive read failures ({max_failures})"
                        break
                    time.sleep(0.01)  # Brief pause before retry
                    continue
                
                # Reset failure counter on success
                consecutive_failures = 0
                
                # Skip warmup frames
                if self.warmup_count < self.config.get('warmup_frames', 5):
                    self.warmup_count += 1
                    continue
                
                # Update frame statistics
                current_time = time.time()
                self.status.frame_count += 1
                self.status.last_frame_time = current_time
                self.last_capture_time = current_time
                
                # Calculate FPS (rolling average)
                self.frame_times.append(current_time)
                if len(self.frame_times) > 10:
                    self.frame_times.pop(0)
                
                if len(self.frame_times) > 1:
                    time_span = self.frame_times[-1] - self.frame_times[0]
                    if time_span > 0:
                        self.status.fps = (len(self.frame_times) - 1) / time_span
                
                # Store raw frame and process
                with self.frame_lock:
                    self.latest_raw_frame = frame.copy()
                    
                    # Apply transformations in order:
                    # 1. Homography (perspective correction)
                    if self.homography_enabled:
                        processed_frame = self._apply_homography(frame)
                    else:
                        processed_frame = frame
                    
                    # 2. Downsampling (size reduction)
                    if self.downsample_factor > 1.0:
                        processed_frame = self._downsample_frame(processed_frame)
                    
                    self.latest_processed_frame = processed_frame.copy()
                    self.latest_frame = processed_frame.copy()  # Backward compatibility
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    self.status.error_count += 1
                    self.status.last_error = str(e)
                    break
                time.sleep(0.01)
        
        print(f"📷 Camera {self.camera_id} capture loop ended")
    
    def get_frame(self) -> Optional[np.ndarray]:
        """Get latest frame from camera (non-blocking)."""
        if not self.status.connected:
            return None
        
        with self.frame_lock:
            if self.latest_processed_frame is not None:
                return self.latest_processed_frame.copy()
            return None
    
    def get_raw_frame(self) -> Optional[np.ndarray]:
        """Get latest raw frame (before downsampling) from camera."""
        if not self.status.connected:
            return None
        
        with self.frame_lock:
            if self.latest_raw_frame is not None:
                return self.latest_raw_frame.copy()
            return None
    
    def _apply_homography(self, frame: np.ndarray) -> np.ndarray:
        """Apply homography transformation to frame if enabled."""
        if not self.homography_enabled or self.homography_matrix is None:
            return frame
        
        height, width = frame.shape[:2]
        try:
            # Apply perspective transform
            warped = cv2.warpPerspective(frame, self.homography_matrix, (width, height))
            return warped
        except Exception as e:
            print(f"❌ Homography transform error: {e}")
            return frame
    
    def _downsample_frame(self, frame: np.ndarray) -> np.ndarray:
        """Downsample frame by the specified factor."""
        if self.downsample_factor <= 1.0:
            return frame
        
        height, width = frame.shape[:2]
        new_width = int(width / self.downsample_factor)
        new_height = int(height / self.downsample_factor)
        
        return cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
    
    def set_downsample_factor(self, factor: float):
        """
        Set downsample factor for this camera.
        
        Args:
            factor: Downsample factor (1.0 = no downsampling, 2.0 = half size, etc.)
        """
        with self.lock:
            self.downsample_factor = max(1.0, factor)
            self.config['downsample_factor'] = self.downsample_factor
            print(f"📷 Camera {self.camera_id} downsample factor set to {self.downsample_factor}x")
    
    def set_homography(self, enabled: bool, source_points: List[List[float]] = None, 
                      destination_points: List[List[float]] = None, 
                      correction_angle: float = None):
        """
        Update homography settings for this camera.
        
        Args:
            enabled: Whether to enable homography transformation
            source_points: 4 points in source image [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
            destination_points: 4 points in destination image
            correction_angle: Camera tilt angle in degrees (alternative to manual points)
        """
        with self.lock:
            self.config['homography']['enabled'] = enabled
            
            if enabled:
                if correction_angle is not None:
                    # Use angle-based configuration
                    self.config['homography']['correction_angle'] = correction_angle
                    # Remove manual points to avoid conflicts
                    if 'source_points' in self.config['homography']:
                        del self.config['homography']['source_points']
                    if 'destination_points' in self.config['homography']:
                        del self.config['homography']['destination_points']
                elif source_points and destination_points:
                    # Use manual point configuration
                    self.config['homography']['source_points'] = source_points
                    self.config['homography']['destination_points'] = destination_points
                    # Remove angle to avoid conflicts
                    if 'correction_angle' in self.config['homography']:
                        del self.config['homography']['correction_angle']
            
            # Recalculate homography matrix
            self._setup_homography()
            
            if self.homography_enabled:
                if 'correction_angle' in self.config['homography']:
                    angle = self.config['homography']['correction_angle']
                    print(f"📷 Camera {self.camera_id} homography enabled with {angle}° correction")
                else:
                    print(f"📷 Camera {self.camera_id} homography enabled with manual points")
            else:
                print(f"📷 Camera {self.camera_id} homography disabled")
    
    def disconnect(self):
        """Disconnect from camera."""
        # Stop capture thread first
        self._stop_capture_thread()
        
        with self.lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            
            self.status.connected = False
            print(f"📷 Camera {self.camera_id} disconnected")
    
    def get_status(self) -> CameraStatus:
        """Get current camera status."""
        return self.status
    
    def is_healthy(self) -> bool:
        """Check if camera is working properly."""
        if not self.status.connected:
            return False
        
        # Check if capture thread is running
        if not self.capture_running or not (self.capture_thread and self.capture_thread.is_alive()):
            return False
        
        # Check error count and recent frame capture
        return (self.status.error_count < 10 and
                (time.time() - self.status.last_frame_time) < 2.0)


class CameraManager:
    """
    Cross-platform dual camera manager with thread-safe access.
    Automatically selects appropriate backend for each platform.
    """
    
    def __init__(self, config_file: str = "camera_config.json"):
        """Initialize camera manager with configuration."""
        self.config_file = config_file
        self.config = self._load_config()
        
        # Initialize camera handles
        self.cameras = {}
        self.cameras[1] = CameraHandle(1, self.config['camera_1'])
        self.cameras[2] = CameraHandle(2, self.config['camera_2'])
        
        # System state
        self.running = False
        self.monitor_thread = None
        
        print(f"🎥 Camera Manager initialized on {platform.system()}")
        print(f"📄 Config loaded from: {config_file}")
    
    def _load_config(self) -> Dict:
        """Load camera configuration from JSON file."""
        default_config = {
            "camera_1": {
                "device_id": 0,  # Can be int, string number, or device name like "usb-xhci-hcd.0-1"
                "resolution": [640, 480],
                "target_fps": 30,
                "buffer_size": 1,
                "fourcc": "MJPG",
                "brightness": 0,
                "contrast": 0,
                "saturation": 0,
                "auto_exposure": True,
                "exposure_time": -1,
                "auto_white_balance": True,
                "white_balance_temp": 4000,
                "gain": 0,
                "enabled": True,
                "downsample_factor": 1.0,
                "homography": {
                    "enabled": False,
                    "source_points": [[0, 0], [640, 0], [640, 480], [0, 480]],
                    "destination_points": [[0, 0], [640, 0], [640, 480], [0, 480]]
                }
            },
            "camera_2": {
                "device_id": 2,  # Can be int, string number, or device name like "usb-xhci-hcd.0-1"
                "resolution": [640, 480],
                "target_fps": 30,
                "buffer_size": 1,
                "fourcc": "MJPG",
                "brightness": 0,
                "contrast": 0,
                "saturation": 0,
                "auto_exposure": True,
                "exposure_time": -1,
                "auto_white_balance": True,
                "white_balance_temp": 4000,
                "gain": 0,
                "enabled": True,
                "downsample_factor": 1.0,
                "homography": {
                    "enabled": False,
                    "source_points": [[0, 0], [640, 0], [640, 480], [0, 480]],
                    "destination_points": [[0, 0], [640, 0], [640, 480], [0, 480]]
                }
            },
            "system": {
                "retry_attempts": 3,
                "retry_delay": 1.0,
                "warmup_frames": 5,
                "frame_timeout": 1.0,
                "enable_threading": True
            }
        }
        
        try:
            with open(self.config_file, 'r') as f:
                user_config = json.load(f)
                # Merge with defaults
                for section in default_config:
                    if section in user_config:
                        default_config[section].update(user_config[section])
                print(f"📋 Configuration loaded from: {self.config_file}")
                return default_config
        except FileNotFoundError:
            print(f"⚠️ Config file {self.config_file} not found, creating with defaults")
            self._save_config(default_config)
            return default_config
        except Exception as e:
            print(f"❌ Error loading config: {e}, using defaults")
            return default_config
    
    def _save_config(self, config: Dict):
        """Save configuration to JSON file."""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"💾 Configuration saved to: {self.config_file}")
        except Exception as e:
            print(f"❌ Failed to save config: {e}")
    
    def start(self):
        """Start camera manager and connect cameras."""
        print("🚀 Starting camera manager...")
        
        # Connect enabled cameras
        for camera_id in [1, 2]:
            camera_key = f"camera_{camera_id}"
            if self.config[camera_key].get('enabled', True):
                success = self._connect_camera_with_retry(camera_id)
                if not success:
                    print(f"⚠️ Camera {camera_id} failed to connect after retries")
        
        # Start monitoring thread if enabled
        if self.config['system']['enable_threading']:
            self.running = True
            self.monitor_thread = threading.Thread(target=self._monitor_cameras, daemon=True)
            self.monitor_thread.start()
            print("🔄 Camera monitoring thread started")
        
        print("✅ Camera manager started")
    
    def _connect_camera_with_retry(self, camera_id: int) -> bool:
        """Connect camera with retry logic."""
        camera = self.cameras[camera_id]
        retry_attempts = self.config['system']['retry_attempts']
        retry_delay = self.config['system']['retry_delay']
        
        for attempt in range(retry_attempts):
            if camera.connect():
                return True
            
            if attempt < retry_attempts - 1:
                print(f"🔄 Camera {camera_id} retry {attempt + 1}/{retry_attempts} in {retry_delay}s...")
                time.sleep(retry_delay)
        
        return False
    
    def _monitor_cameras(self):
        """Background thread to monitor camera health."""
        while self.running:
            try:
                for camera_id, camera in self.cameras.items():
                    if not camera.is_healthy() and self.config[f'camera_{camera_id}'].get('enabled', True):
                        print(f"🔄 Camera {camera_id} unhealthy, attempting reconnection...")
                        camera.disconnect()
                        time.sleep(1.0)
                        self._connect_camera_with_retry(camera_id)
                
                time.sleep(5.0)  # Check every 5 seconds
                
            except Exception as e:
                print(f"❌ Monitor thread error: {e}")
                time.sleep(1.0)
    
    def stop(self):
        """Stop camera manager and disconnect cameras."""
        print("🛑 Stopping camera manager...")
        
        # Stop monitoring thread
        self.running = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        # Disconnect cameras
        for camera in self.cameras.values():
            camera.disconnect()
        
        print("✅ Camera manager stopped")
    
    def get_frame(self, camera_id: int) -> Optional[np.ndarray]:
        """
        Get latest frame from specified camera.
        
        Args:
            camera_id: Camera number (1 or 2)
            
        Returns:
            Latest frame as numpy array or None if unavailable
        """
        if camera_id not in self.cameras:
            return None
        
        return self.cameras[camera_id].get_frame()
    
    def get_both_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get frames from both cameras simultaneously.
        
        Returns:
            Tuple of (camera_1_frame, camera_2_frame)
        """
        frame1 = self.get_frame(1)
        frame2 = self.get_frame(2)
        return frame1, frame2
    
    def get_raw_frame(self, camera_id: int) -> Optional[np.ndarray]:
        """
        Get latest raw frame (before downsampling) from specified camera.
        
        Args:
            camera_id: Camera number (1 or 2)
            
        Returns:
            Latest raw frame as numpy array or None if unavailable
        """
        if camera_id not in self.cameras:
            return None
        
        return self.cameras[camera_id].get_raw_frame()
    
    def get_both_raw_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get raw frames from both cameras simultaneously.
        
        Returns:
            Tuple of (camera_1_raw_frame, camera_2_raw_frame)
        """
        frame1 = self.get_raw_frame(1)
        frame2 = self.get_raw_frame(2)
        return frame1, frame2
    
    def get_camera_status(self, camera_id: int) -> Optional[CameraStatus]:
        """
        Get status information for specified camera.
        
        Args:
            camera_id: Camera number (1 or 2)
            
        Returns:
            CameraStatus object or None if camera doesn't exist
        """
        if camera_id not in self.cameras:
            return None
        
        return self.cameras[camera_id].get_status()
    
    def is_camera_active(self, camera_id: int) -> bool:
        """
        Check if specified camera is active and working.
        
        Args:
            camera_id: Camera number (1 or 2)
            
        Returns:
            True if camera is connected and healthy
        """
        if camera_id not in self.cameras:
            return False
        
        return self.cameras[camera_id].is_healthy()
    
    def set_downsample_factor(self, camera_id: int, factor: float):
        """
        Set downsample factor for a specific camera.
        
        Args:
            camera_id: Camera number (1 or 2)
            factor: Downsample factor (1.0 = no downsampling, 2.0 = half size, etc.)
        """
        if camera_id in self.cameras:
            self.cameras[camera_id].set_downsample_factor(factor)
            # Update config
            self.config[f'camera_{camera_id}']['downsample_factor'] = factor
            self._save_config(self.config)
    
    def set_downsample_all(self, factor: float):
        """
        Set downsample factor for all cameras.
        
        Args:
            factor: Downsample factor (1.0 = no downsampling, 2.0 = half size, etc.)
        """
        for camera_id in self.cameras:
            self.set_downsample_factor(camera_id, factor)
    
    def get_downsample_factor(self, camera_id: int) -> float:
        """
        Get current downsample factor for a camera.
        
        Args:
            camera_id: Camera number (1 or 2)
            
        Returns:
            Current downsample factor
        """
        if camera_id in self.cameras:
            return self.cameras[camera_id].downsample_factor
        return 1.0
    
    def get_system_status(self) -> Dict:
        """
        Get comprehensive system status.
        
        Returns:
            Dictionary with status of all cameras and system info
        """
        status = {
            'running': self.running,
            'config_file': self.config_file,
            'cameras': {}
        }
        
        for camera_id, camera in self.cameras.items():
            cam_status = camera.get_status()
            status['cameras'][camera_id] = {
                'connected': cam_status.connected,
                'fps': cam_status.fps,
                'frame_count': cam_status.frame_count,
                'error_count': cam_status.error_count,
                'last_error': cam_status.last_error,
                'healthy': camera.is_healthy(),
                'device_id': self.config[f'camera_{camera_id}']['device_id'],
                'resolution': self.config[f'camera_{camera_id}']['resolution'],
                'target_fps': self.config[f'camera_{camera_id}']['target_fps'],
                'downsample_factor': camera.downsample_factor,
                'homography_enabled': camera.homography_enabled,
                'capture_thread_running': camera.capture_running,
                'capture_thread_alive': camera.capture_thread and camera.capture_thread.is_alive(),
                'last_capture_time': camera.last_capture_time
            }
        
        return status
    
    def set_homography(self, camera_id: int, enabled: bool, 
                      source_points: List[List[float]] = None,
                      destination_points: List[List[float]] = None,
                      correction_angle: float = None):
        """
        Set homography transformation for a specific camera.
        
        Args:
            camera_id: Camera number (1 or 2)
            enabled: Whether to enable homography
            source_points: 4 corner points in source image
            destination_points: 4 corner points in destination image
            correction_angle: Camera tilt angle in degrees (alternative to manual points)
        """
        if camera_id in self.cameras:
            self.cameras[camera_id].set_homography(enabled, source_points, destination_points, correction_angle)
            # Update config
            self.config[f'camera_{camera_id}']['homography']['enabled'] = enabled
            if correction_angle is not None:
                self.config[f'camera_{camera_id}']['homography']['correction_angle'] = correction_angle
                # Remove manual points to avoid conflicts
                if 'source_points' in self.config[f'camera_{camera_id}']['homography']:
                    del self.config[f'camera_{camera_id}']['homography']['source_points']
                if 'destination_points' in self.config[f'camera_{camera_id}']['homography']:
                    del self.config[f'camera_{camera_id}']['homography']['destination_points']
            elif source_points and destination_points:
                self.config[f'camera_{camera_id}']['homography']['source_points'] = source_points
                self.config[f'camera_{camera_id}']['homography']['destination_points'] = destination_points
                # Remove angle to avoid conflicts
                if 'correction_angle' in self.config[f'camera_{camera_id}']['homography']:
                    del self.config[f'camera_{camera_id}']['homography']['correction_angle']
            self._save_config(self.config)


# Signal handler for clean shutdown
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\n🛑 Shutting down camera manager...")
    sys.exit(0)


if __name__ == "__main__":
    """
    Test camera manager from command line.
    Usage: python camera_manager.py [--config CONFIG_FILE] [--duration SECONDS]
    """
    import argparse
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Camera Manager Test")
    parser.add_argument('--config', type=str, default='camera_config.json', help='Config file path')
    parser.add_argument('--duration', type=int, help='Test duration in seconds')
    args = parser.parse_args()
    
    print("🎥 Camera Manager Test")
    
    try:
        # Create and start camera manager
        manager = CameraManager(config_file=args.config)
        manager.start()
        
        print("📷 Testing camera access...")
        start_time = time.time()
        
        # Test camera access
        while True:
            frame1, frame2 = manager.get_both_frames()
            
            # Print status
            status = manager.get_system_status()
            cam1_status = "✅" if status['cameras'][1]['connected'] else "❌"
            cam2_status = "✅" if status['cameras'][2]['connected'] else "❌"
            fps1 = status['cameras'][1]['fps']
            fps2 = status['cameras'][2]['fps']
            
            print(f"\rCamera 1: {cam1_status} {fps1:.1f}fps | Camera 2: {cam2_status} {fps2:.1f}fps", 
                  end='', flush=True)
            
            # Check duration limit
            if args.duration and (time.time() - start_time) > args.duration:
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        pass
    finally:
        if 'manager' in locals():
            manager.stop()
    
    print("\n✅ Camera Manager test completed")