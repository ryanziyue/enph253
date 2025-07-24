# Windows Setup Guide

This guide helps you set up the robot controller system on Windows.

## Camera Manager Windows Support ✅

The camera manager now supports **cross-platform operation**:

- **Linux**: V4L2 backend (original)
- **Windows**: DirectShow backend (new)
- **macOS**: AVFoundation backend (new)
- **Fallback**: Default OpenCV backend

## Prerequisites

### 1. Python Dependencies
```bash
pip install opencv-python numpy
```

### 2. Camera Drivers
- Ensure your USB cameras have proper Windows drivers installed
- Most USB cameras work with built-in Windows drivers
- For specialized cameras, install manufacturer drivers

## Quick Test

### 1. Test Camera Compatibility
```bash
python test_camera_windows.py
```

This script will:
- ✅ Test OpenCV backend availability
- 🔍 Enumerate available cameras
- 📝 Create Windows-compatible config file
- 🎥 Test Camera Manager functionality

### 2. Expected Output
```
🪟 Windows Camera Compatibility Test
==================================================
🧪 Testing OpenCV Backend Support
========================================
Platform: Windows 10
OpenCV Version: 4.8.0

✅ Default: Available
✅ DirectShow (Windows): Available
   📸 Frame capture: Success (640, 480)
❌ V4L2 (Linux): Not available in this OpenCV build

🔍 Camera Device Enumeration
==============================
📷 Camera 0: Available (640x480)
📷 Camera 1: Available (1280x720)
📷 Camera 2: Not available

✅ Found 2 working cameras

🎥 Camera Manager Test
=========================
✅ Camera 1: Frame captured (640x480)
✅ Camera 2: Frame captured (1280x720)

🎉 All tests passed! Camera Manager is Windows-compatible.
```

## Configuration

### Windows-Specific Settings

The camera configuration is automatically adjusted for Windows:

```json
{
  "cameras": {
    "1": {
      "device_id": 0,          // First camera (usually built-in)
      "resolution": [640, 480],
      "target_fps": 30,
      "buffer_size": 1,        // Smaller buffer for Windows
      "fourcc": "MJPG",        // MJPEG works well on Windows
      "brightness": 0,
      "contrast": 0,
      "saturation": 0,
      "enabled": true
    },
    "2": {
      "device_id": 1,          // Second camera (USB)
      "resolution": [640, 480],
      "target_fps": 30,
      "buffer_size": 1,
      "fourcc": "MJPG",
      "brightness": 0,
      "contrast": 0,
      "saturation": 0,
      "enabled": true
    }
  }
}
```

## Common Issues & Solutions

### Issue 1: "Camera not opening"
**Symptoms**: Camera detection fails
**Solutions**:
- Check if other applications are using the camera
- Try different device IDs (0, 1, 2...)
- Install camera manufacturer drivers
- Restart the application

### Issue 2: "DirectShow backend not working"
**Symptoms**: Falls back to default backend
**Solutions**:
- Update OpenCV: `pip install --upgrade opencv-python`
- Check Windows camera permissions
- Try MediaFoundation backend (CAP_MSMF)

### Issue 3: "Poor frame rate"
**Symptoms**: Low FPS, choppy video
**Solutions**:
- Reduce resolution in config
- Set `buffer_size: 1` 
- Use MJPEG codec (`fourcc: "MJPG"`)
- Close other camera applications

### Issue 4: "USB camera not detected"
**Symptoms**: Only built-in camera works
**Solutions**:
- Check USB connection
- Try different USB ports (preferably USB 3.0)
- Install camera-specific drivers
- Use USB camera software to verify functionality

## Running the Robot Controller

### 1. Basic Usage
```bash
python robot_controller.py --serial-port COM3
```

### 2. Without GUI (Console only)
```bash
python robot_controller.py --no-gui --console --serial-port COM3
```

### 3. Test Mode with Annotations
```bash
python robot_controller.py --test --serial-port COM3
```

## Windows-Specific Notes

1. **Serial Ports**: Use Windows format (`COM1`, `COM3`, etc.)
2. **Camera IDs**: Usually 0 (built-in) and 1+ (USB cameras)
3. **Permissions**: May need to allow camera access in Windows Privacy settings
4. **Performance**: DirectShow backend may have slight latency compared to V4L2

## Troubleshooting

### Enable Debug Mode
Add debug output to see backend selection:
```python
# In camera_manager.py, the system will print:
# 📷 Using DirectShow backend for camera 1 on Windows
```

### Check System Info
```python
import platform
import cv2
print(f"Platform: {platform.system()}")
print(f"OpenCV: {cv2.__version__}")
```

### Manual Camera Test
```python
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # DirectShow
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print(f"Success: {frame.shape}")
cap.release()
```

## Performance Tips

1. **Use MJPEG codec** for better compression
2. **Set buffer_size=1** to reduce latency  
3. **Lower resolution** if frame rate drops
4. **Close other camera apps** before running
5. **Use USB 3.0 ports** for multiple cameras

## Support

- Run `test_camera_windows.py` for comprehensive diagnostics
- Check OpenCV installation: `python -c "import cv2; print(cv2.__version__)"`
- Verify camera hardware with Windows Camera app first

✅ **The camera manager now fully supports Windows!**