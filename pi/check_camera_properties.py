#!/usr/bin/env python3
"""
Check actual camera properties vs requested properties.
This helps identify if the camera is cropping or using different settings than requested.
"""

import cv2
import platform

def check_camera_properties(device_id, backend=None):
    """Check all camera properties for a given device."""
    print(f"\n📷 Checking Camera Device {device_id}")
    print("=" * 60)
    
    # Open camera with specified backend
    if backend:
        cap = cv2.VideoCapture(device_id, backend)
        print(f"Using backend: {backend}")
    else:
        cap = cv2.VideoCapture(device_id)
        print("Using default backend")
    
    if not cap.isOpened():
        print(f"❌ Could not open camera device {device_id}")
        return
    
    # Properties to check
    properties = [
        (cv2.CAP_PROP_FRAME_WIDTH, "Frame Width"),
        (cv2.CAP_PROP_FRAME_HEIGHT, "Frame Height"),
        (cv2.CAP_PROP_FPS, "FPS"),
        (cv2.CAP_PROP_FOURCC, "FOURCC"),
        (cv2.CAP_PROP_BRIGHTNESS, "Brightness"),
        (cv2.CAP_PROP_CONTRAST, "Contrast"),
        (cv2.CAP_PROP_SATURATION, "Saturation"),
        (cv2.CAP_PROP_HUE, "Hue"),
        (cv2.CAP_PROP_GAIN, "Gain"),
        (cv2.CAP_PROP_EXPOSURE, "Exposure"),
        (cv2.CAP_PROP_AUTO_EXPOSURE, "Auto Exposure"),
        (cv2.CAP_PROP_BUFFERSIZE, "Buffer Size"),
        (cv2.CAP_PROP_ZOOM, "Zoom"),
        (cv2.CAP_PROP_FOCUS, "Focus"),
        (cv2.CAP_PROP_PAN, "Pan"),
        (cv2.CAP_PROP_TILT, "Tilt"),
        (cv2.CAP_PROP_ROLL, "Roll"),
    ]
    
    print("\n📊 Current Camera Properties:")
    for prop_id, prop_name in properties:
        value = cap.get(prop_id)
        if value != -1 and value != 0:  # -1 usually means unsupported
            if prop_id == cv2.CAP_PROP_FOURCC:
                # Convert FOURCC to string
                fourcc_int = int(value)
                fourcc_str = "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
                print(f"   {prop_name}: {fourcc_str} ({value})")
            else:
                print(f"   {prop_name}: {value}")
    
    # Test different resolutions
    print("\n🔧 Testing Resolution Changes:")
    test_resolutions = [
        (640, 480),
        (800, 600),
        (1280, 720),
        (1920, 1080),
        (320, 240),
    ]
    
    for width, height in test_resolutions:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
        if actual_width == width and actual_height == height:
            print(f"   ✅ {width}x{height} - Supported")
        else:
            print(f"   ❌ {width}x{height} - Not supported (got {int(actual_width)}x{int(actual_height)})")
    
    # Check for actual frame
    print("\n🎞️ Capturing test frame...")
    ret, frame = cap.read()
    if ret and frame is not None:
        actual_height, actual_width = frame.shape[:2]
        print(f"   Frame shape: {actual_width}x{actual_height}")
        print(f"   Frame type: {frame.dtype}")
        print(f"   Channels: {frame.shape[2] if len(frame.shape) > 2 else 1}")
        
        # Check if image appears flipped
        print("\n🔄 Checking for common issues:")
        
        # Create a simple test pattern
        test_frame = frame.copy()
        height, width = test_frame.shape[:2]
        
        # Draw markers in corners
        cv2.putText(test_frame, "TL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(test_frame, "TR", (width-40, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(test_frame, "BL", (10, height-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(test_frame, "BR", (width-40, height-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Save test frame
        cv2.imwrite(f"camera_test_device_{device_id}.jpg", test_frame)
        print(f"   📸 Saved test frame: camera_test_device_{device_id}.jpg")
        print("   Check if TL/TR/BL/BR markers appear in correct positions")
    
    cap.release()
    print()

def main():
    print("🔍 Camera Properties Diagnostic")
    print("This tool checks actual vs requested camera properties")
    
    # Platform info
    system = platform.system()
    print(f"\n💻 Platform: {system}")
    
    # Choose backend based on platform
    if system == "Linux":
        backend = cv2.CAP_V4L2
    elif system == "Windows":
        backend = cv2.CAP_DSHOW
    elif system == "Darwin":
        backend = cv2.CAP_AVFOUNDATION
    else:
        backend = None
    
    # Check cameras
    for device_id in [0, 1, 2]:
        check_camera_properties(device_id, backend)
    
    print("\n💡 Possible causes of centering issue:")
    print("1. Camera firmware auto-crops to achieve requested resolution")
    print("2. Camera sensor has different aspect ratio than output")
    print("3. Camera is horizontally flipped (check test images)")
    print("4. Display software is performing additional processing")
    print("\n🔧 Solutions:")
    print("1. Try different resolutions to find native sensor resolution")
    print("2. Check if camera has zoom/pan settings that are non-zero")
    print("3. Use cv2.flip() if image is mirrored")

if __name__ == "__main__":
    main()