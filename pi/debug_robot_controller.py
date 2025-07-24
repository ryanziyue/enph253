#!/usr/bin/env python3
"""
Debug version of robot controller to isolate the 'Killed' issue.
"""

import time
import sys
import traceback
import psutil
import os

def check_memory_usage():
    """Check current memory usage."""
    process = psutil.Process(os.getpid())
    memory_info = process.memory_info()
    print(f"📊 Memory Usage: {memory_info.rss / 1024 / 1024:.1f} MB")
    return memory_info.rss / 1024 / 1024  # Return MB

def test_camera_manager():
    """Test camera manager initialization."""
    print("🔍 Testing Camera Manager...")
    try:
        from camera_manager import CameraManager
        print("✅ Camera manager imported successfully")
        
        manager = CameraManager()
        print("✅ Camera manager created")
        check_memory_usage()
        
        manager.start()
        print("✅ Camera manager started")
        check_memory_usage()
        
        time.sleep(2)
        
        # Test getting frames
        frame1 = manager.get_frame(1)
        frame2 = manager.get_frame(2)
        print(f"✅ Frame test: Cam1={frame1 is not None}, Cam2={frame2 is not None}")
        check_memory_usage()
        
        manager.stop()
        print("✅ Camera manager stopped")
        check_memory_usage()
        
        return True
        
    except Exception as e:
        print(f"❌ Camera manager test failed: {e}")
        traceback.print_exc()
        return False

def test_object_detection():
    """Test object detection manager."""
    print("\n🔍 Testing Object Detection Manager...")
    try:
        from camera_manager import CameraManager
        from object_detection_manager import ObjectDetectionManager
        
        print("✅ Object detection imported successfully")
        
        camera_manager = CameraManager()
        camera_manager.start()
        time.sleep(1)
        check_memory_usage()
        
        obj_detector = ObjectDetectionManager(camera_manager, camera_id=2)
        print("✅ Object detection manager created")
        check_memory_usage()
        
        # Test detection
        result = obj_detector.run_detection()
        print(f"✅ Detection test: Found={result.found}, Confidence={result.confidence:.3f}")
        check_memory_usage()
        
        obj_detector.cleanup()
        camera_manager.stop()
        print("✅ Object detection cleaned up")
        check_memory_usage()
        
        return True
        
    except Exception as e:
        print(f"❌ Object detection test failed: {e}")
        traceback.print_exc()
        return False

def test_robot_controller_minimal():
    """Test minimal robot controller."""
    print("\n🔍 Testing Minimal Robot Controller...")
    try:
        from robot_controller import RobotController
        
        print("✅ Robot controller imported successfully")
        check_memory_usage()
        
        # Create controller without serial port to avoid hardware dependencies
        controller = RobotController(
            config_file="camera_config.json",
            enable_gui=False,  # Disable GUI to reduce complexity
            serial_port=None   # No serial port
        )
        print("✅ Robot controller created")
        check_memory_usage()
        
        # Start systems
        controller.start()
        print("✅ Robot controller started")
        check_memory_usage()
        
        # Wait a bit
        time.sleep(3)
        print("✅ Robot controller running for 3 seconds")
        check_memory_usage()
        
        # Stop
        controller.stop()
        print("✅ Robot controller stopped")
        check_memory_usage()
        
        return True
        
    except Exception as e:
        print(f"❌ Robot controller test failed: {e}")
        traceback.print_exc()
        return False

def main():
    print("🔍 Robot Controller Debug Tool")
    print("=" * 50)
    
    # Initial memory check
    print(f"🏁 Starting debug at PID: {os.getpid()}")
    initial_memory = check_memory_usage()
    
    # Test individual components
    tests = [
        ("Camera Manager", test_camera_manager),
        ("Object Detection", test_object_detection),
        ("Robot Controller", test_robot_controller_minimal),
    ]
    
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        
        try:
            memory_before = check_memory_usage()
            success = test_func()
            memory_after = check_memory_usage()
            memory_diff = memory_after - memory_before
            
            status = "✅ PASSED" if success else "❌ FAILED"
            print(f"\n{status} {test_name}")
            print(f"📊 Memory change: {memory_diff:+.1f} MB")
            
            if not success:
                print("🛑 Stopping tests due to failure")
                break
                
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted by user")
            break
        except Exception as e:
            print(f"❌ CRITICAL ERROR in {test_name}: {e}")
            traceback.print_exc()
            break
        
        # Pause between tests
        time.sleep(1)
    
    final_memory = check_memory_usage()
    total_memory_change = final_memory - initial_memory
    print(f"\n📊 Total memory change: {total_memory_change:+.1f} MB")
    
    if total_memory_change > 100:
        print("⚠️ WARNING: Significant memory increase detected!")
    
    print("\n✅ Debug session complete")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Debug session interrupted")
    except Exception as e:
        print(f"\n❌ FATAL ERROR: {e}")
        traceback.print_exc()
    finally:
        print(f"\n🏁 Debug session ended at PID: {os.getpid()}")