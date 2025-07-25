#!/usr/bin/env python3
"""
Debug object detection differences between UI and robot controller.
"""

import sys
import time
from camera_manager import CameraManager
from object_detection_manager import ObjectDetectionManager

def test_ui_style():
    """Test object detection exactly like the UI does."""
    print("🧪 Testing UI-style initialization...")
    
    # Exactly like UI
    camera_manager = CameraManager()
    camera_manager.start()
    time.sleep(2)  # Give cameras time to initialize
    
    # Default camera ID 2
    camera_id = 2
    detection_manager = ObjectDetectionManager(camera_manager, camera_id, 
                                             config_file="object_detection_config_yolov11n.json")
    
    # Test frame access
    frame = camera_manager.get_frame(camera_id)
    if frame is not None:
        print(f"✅ UI-style: Camera {camera_id} frame: {frame.shape}")
    else:
        print(f"❌ UI-style: No frame from camera {camera_id}")
    
    # Test detection
    print("Testing UI-style detection...")
    result = detection_manager.run_detection_blocking()
    print(f"   Result: found={result.found}, conf={result.confidence:.3f}, class='{result.class_name}'")
    
    camera_manager.stop()
    return result.found

def test_robot_controller_style():
    """Test object detection exactly like robot controller does."""
    print("\n🧪 Testing Robot Controller-style initialization...")
    
    # Exactly like robot controller
    camera_manager = CameraManager("camera_config.json")  # Explicit config file
    camera_manager.start()
    time.sleep(1)  # Shorter wait time like robot controller
    
    # Exactly like robot controller: camera_id=2 
    camera_id = 2
    try:
        detection_manager = ObjectDetectionManager(camera_manager, camera_id=2, 
                                                 config_file="object_detection_config_yolov11n.json")
        
        # Test frame access like robot controller does
        frame = camera_manager.get_frame(2)
        if frame is not None:
            height, width = frame.shape[:2]
            print(f"✅ Robot-style: Camera 2 frame: {width}x{height}")
        else:
            print("❌ Robot-style: No frame from camera 2")
        
        # Test detection
        print("Testing robot-style detection...")
        result = detection_manager.run_detection_blocking()
        print(f"   Result: found={result.found}, conf={result.confidence:.3f}, class='{result.class_name}'")
        
        camera_manager.stop()
        return result.found
        
    except Exception as e:
        print(f"❌ Robot-style initialization failed: {e}")
        camera_manager.stop()
        return False

def check_camera_status():
    """Check camera availability and status."""
    print("\n📷 Checking camera status...")
    
    camera_manager = CameraManager("camera_config.json")
    camera_manager.start()
    time.sleep(2)
    
    status = camera_manager.get_system_status()
    
    for cam_id in [1, 2]:
        if cam_id in status['cameras']:
            cam_status = status['cameras'][cam_id]
            print(f"Camera {cam_id}:")
            print(f"   Connected: {cam_status['connected']}")
            print(f"   Healthy: {cam_status['healthy']}")
            print(f"   FPS: {cam_status['fps']:.1f}")
            print(f"   Device ID: {cam_status['device_id']}")
            print(f"   Resolution: {cam_status['resolution']}")
            print(f"   Error count: {cam_status['error_count']}")
            if cam_status['last_error']:
                print(f"   Last error: {cam_status['last_error']}")
    
    camera_manager.stop()

def main():
    print("🔍 Object Detection Debug Tool")
    print("="*50)
    
    # Check camera status first
    check_camera_status()
    
    # Test both approaches
    ui_works = test_ui_style()
    robot_works = test_robot_controller_style()
    
    print("\n📊 Summary:")
    print(f"UI-style detection works: {'✅' if ui_works else '❌'}")
    print(f"Robot Controller-style detection works: {'✅' if robot_works else '❌'}")
    
    if ui_works and not robot_works:
        print("\n🔍 Potential issues with Robot Controller approach:")
        print("1. Camera timing - Robot Controller waits less time")
        print("2. Configuration differences")
        print("3. Camera availability/health issues")
        print("4. Frame access patterns")
    elif robot_works and not ui_works:
        print("\n🔍 Potential issues with UI approach:")
        print("1. Default camera manager config vs explicit config")
        print("2. Camera initialization differences")
    elif not ui_works and not robot_works:
        print("\n❌ Both approaches fail - check:")
        print("1. Model file exists: yolov11_ft.pt")
        print("2. Camera connectivity")
        print("3. YOLO installation")
    else:
        print("\n✅ Both approaches work - issue might be elsewhere")

if __name__ == "__main__":
    main()