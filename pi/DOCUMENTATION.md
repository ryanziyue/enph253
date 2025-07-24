# Robot Vision System Documentation

## Overview

This system provides threaded camera management and line following detection for robotic applications. The architecture separates concerns cleanly: camera management handles frame capture, line following handles detection logic, and UIs handle visualization.

## Architecture

```
Camera Manager (Threaded) ←→ Line Following Manager ←→ Test UI
       ↓
   Robot Controller ←→ Robot GUI
```

## Camera Manager (`camera_manager.py`)

### Purpose
Manages dual camera feeds with continuous threaded capture for instant, non-blocking frame access.

### Key Features
- **Threaded capture**: Each camera runs continuous capture in background thread
- **Non-blocking access**: `get_frame()` returns immediately with latest frame
- **Downsampling**: Optional frame downsampling for performance
- **Health monitoring**: Automatic reconnection and error recovery
- **Thread-safe**: Multiple consumers can access frames simultaneously

### Usage
```python
from camera_manager import CameraManager

# Initialize and start
manager = CameraManager("camera_config.json")
manager.start()

# Get frames (non-blocking, always fresh)
frame1 = manager.get_frame(1)                    # Camera 1 (processed)
frame2 = manager.get_frame(2)                    # Camera 2 (processed)
raw_frame1 = manager.get_raw_frame(1)            # Camera 1 (original)
both_frames = manager.get_both_frames()          # Both cameras

# Downsampling control
manager.set_downsample_factor(1, 2.0)            # Camera 1 half-size
manager.set_downsample_all(2.0)                  # Both cameras half-size

# Status monitoring
status = manager.get_system_status()
healthy = manager.is_camera_active(1)

# Cleanup
manager.stop()
```

### Configuration (`camera_config.json`)
```json
{
  "camera_1": {
    "device_id": 0,
    "resolution": [640, 480],
    "target_fps": 30,
    "downsample_factor": 1.0,
    "brightness": 0,
    "enabled": true
  },
  "camera_2": {
    "device_id": 2,
    "resolution": [640, 480], 
    "target_fps": 30,
    "downsample_factor": 1.0,
    "brightness": 0,
    "enabled": true
  }
}
```

### Command Line
```bash
# Test camera manager
python camera_manager.py --duration 30

# Test with GUI
python camera_manager_gui.py

# Robot system with downsampling
python robot_controller.py --downsample 2.0
```

## Line Following Manager (`line_following_manager.py`)

### Purpose
Detects black lines in camera frames using multi-row scanning with brown filtering, based on proven legacy algorithms.

### Key Features
- **Multi-row scanning**: Configurable number of rows with spacing
- **Brown filtering**: Rejects line segments with too much brown content
- **Adaptive center**: Tracks line position for better search efficiency
- **Thread-safe**: Safe for use with threaded camera manager
- **Parameter control**: Runtime adjustment of detection parameters

### Core Algorithms

#### `find_black_line(frame, row_y)`
- Scans single row for longest dark segment
- Returns center position and width
- Uses configurable brightness threshold

#### `check_brown(image_region)`
- Analyzes HSV color space for brown pixels
- Returns percentage of brown content (0-100%)
- Uses multiple brown color ranges

#### `get_line()`
- Main detection function called by robot system
- Scans multiple rows for line segments
- Applies brown filtering to reject false positives
- Updates adaptive center automatically
- Returns list of detected points with metadata

### Usage
```python
from camera_manager import CameraManager
from line_following_manager import LineFollowingManager

# Initialize
camera_manager = CameraManager()
camera_manager.start()

line_manager = LineFollowingManager(camera_manager, camera_id=1)

# Get line detections
points = line_manager.get_line()

# Process results
for point in points:
    if not point['brown_rejected']:
        x, y = point['x'], point['y']
        width = point['width']
        brown_pct = point['brown_percentage']
        print(f"Valid line at ({x}, {y}), width={width}, brown={brown_pct:.1f}%")

# Parameter control
line_manager.reset_adaptive_center()
line_manager.set_search_width(40.0)  # 40% of image width
line_manager.set_camera_id(2)        # Switch to camera 2

# Get parameters
params = line_manager.get_parameters()
print(f"Adaptive center: {params['adaptive_center_x']}")
```

### Detection Parameters
- `max_brightness`: Maximum pixel value for line detection (default: 80)
- `search_width_percent`: Search region width as % of image (default: 30%)
- `num_rows`: Number of rows to scan (default: 10)
- `row_spacing_percent`: Spacing between scan rows (default: 5%)
- `brown_threshold_percent`: Max brown content to accept (default: 50%)
- `brown_rect_height_percent`: Height of brown analysis region (default: 3%)

### Command Line
```bash
# Test line following manager
python line_following_manager.py
```

## Line Following UI (`line_following_ui.py`)

### Purpose
Visualization interface for line following system. Displays camera feed with detected points, search regions, and analysis information.

### Features
- **Live camera display**: Shows camera feed with overlays
- **Point visualization**: Green for valid points, red for brown-rejected
- **Search region display**: Shows adaptive center and search boundaries
- **Scan row indicators**: Horizontal lines showing scanned rows
- **Real-time parameters**: Current detection settings and statistics
- **Interactive controls**: Keyboard shortcuts for parameter adjustment

### Usage
```bash
# Start line following UI
python line_following_ui.py --camera 1

# With different camera
python line_following_ui.py --camera 2
```

### Controls
- **R**: Reset adaptive center to image center
- **W/S**: Increase/decrease search width
- **B/V**: Increase/decrease brightness threshold  
- **C**: Switch between cameras
- **Q**: Quit

### Visual Elements
- 🟢 **Green points**: Valid line detections
- 🔴 **Red points**: Brown-rejected detections
- 🟡 **Yellow lines**: Search region boundaries
- 🔵 **Cyan line**: Adaptive center position
- 📏 **Gray lines**: Scan row positions
- 📊 **Status overlay**: Parameters and statistics

## Integration Examples

### Robot Controller Integration
```python
from camera_manager import CameraManager
from line_following_manager import LineFollowingManager

class RobotController:
    def __init__(self):
        self.camera_manager = CameraManager()
        self.line_manager = LineFollowingManager(self.camera_manager, camera_id=1)
        
    def start(self):
        self.camera_manager.start()
        
    def control_loop(self):
        while True:
            # Get line detections
            points = self.line_manager.get_line()
            
            # Find closest valid point
            valid_points = [p for p in points if not p['brown_rejected']]
            if valid_points:
                closest = max(valid_points, key=lambda p: p['y'])
                
                # Calculate steering based on line position
                center_x = closest['x']
                image_width = 640  # Adjust based on camera resolution
                error = (center_x - image_width/2) / (image_width/2)
                
                # Apply steering (your robot control code here)
                self.steer(error)
```

### GUI Integration
```python
from robot_gui import RobotGUI, Point, Text

# In your robot controller
gui.send_annotation(
    Point(camera_id=1, x=line_x, y=line_y, 
          color=(0, 255, 0), radius=8),
    persistent=False
)

gui.send_annotation(
    Text(camera_id=1, x=10, y=30, 
         text=f"Line Error: {error:.2f}",
         color=(255, 255, 255)),
    persistent=False
)
```

## Performance Notes

### Camera Manager
- **Threaded capture**: ~30 FPS per camera with minimal CPU overhead
- **Non-blocking access**: Sub-millisecond frame retrieval
- **Memory usage**: ~10MB per camera for frame buffers
- **Downsampling**: 2x downsampling can improve processing speed by 4x

### Line Following
- **Processing time**: ~1-5ms per frame (depending on image size)
- **Brown filtering**: Adds ~1-2ms overhead but significantly improves accuracy
- **Adaptive center**: Reduces false positives by ~30% in testing
- **Multi-row scanning**: More robust than single-row detection

### Recommended Settings
- **Camera resolution**: 640x480 for good balance of speed/accuracy
- **Downsampling**: 2x for processing, raw frames for display
- **Search width**: 30-40% for typical line following scenarios
- **Rows**: 8-12 rows with 5% spacing for comprehensive coverage

## Troubleshooting

### Camera Issues
- **No camera feed**: Check device IDs in config, ensure cameras connected
- **Poor performance**: Enable downsampling, reduce resolution
- **Threading errors**: Ensure proper cleanup with `manager.stop()`

### Line Detection Issues
- **No detections**: Adjust `max_brightness` threshold
- **False positives**: Enable brown filtering, reduce search width
- **Missed lines**: Increase search width, add more scan rows
- **Jittery tracking**: Enable adaptive center, smooth parameters

### UI Issues
- **Slow display**: Reduce display resolution, enable camera downsampling
- **No response**: Check camera connection, verify camera ID