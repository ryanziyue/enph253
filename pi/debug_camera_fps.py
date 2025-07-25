#!/usr/bin/env python3
"""
Debug camera FPS issues when using device names.
Tests different configurations to find what works.
"""

import cv2
import time
import sys

def test_camera_config(device, description):
    """Test a camera configuration and report FPS."""
    print(f"\n{'='*60}")
    print(f"Testing: {description}")
    print(f"Device: {device}")
    print(f"{'='*60}")
    
    # Try opening camera
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("❌ Failed to open camera")
        return
    
    # Test 1: Default settings
    print("\n📊 Test 1: Default settings")
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cap.get(cv2.CAP_PROP_FOURCC)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fourcc_str = "".join([chr((int(fourcc) >> (8 * i)) & 0xFF) for i in range(4)])
    print(f"   FPS: {fps}")
    print(f"   Resolution: {width}x{height}")
    print(f"   FOURCC: {fourcc_str}")
    
    # Test 2: Set MJPG first, then FPS
    print("\n📊 Test 2: MJPG first, then FPS")
    cap.release()
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    fourcc_mjpg = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
    time.sleep(0.1)  # Let it settle
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cap.get(cv2.CAP_PROP_FOURCC)
    fourcc_str = "".join([chr((int(fourcc) >> (8 * i)) & 0xFF) for i in range(4)])
    print(f"   FPS: {fps}")
    print(f"   FOURCC: {fourcc_str}")
    
    # Test 3: Set all properties in specific order
    print("\n📊 Test 3: MJPG → Resolution → FPS")
    cap.release()
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fourcc = cap.get(cv2.CAP_PROP_FOURCC)
    fourcc_str = "".join([chr((int(fourcc) >> (8 * i)) & 0xFF) for i in range(4)])
    print(f"   FPS: {fps}")
    print(f"   Resolution: {width}x{height}")
    print(f"   FOURCC: {fourcc_str}")
    
    # Test 4: Try lower resolution
    print("\n📊 Test 4: MJPG with lower resolution (320x240)")
    cap.release()
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"   FPS: {fps}")
    print(f"   Resolution: {width}x{height}")
    
    # Test 5: Measure actual FPS
    print("\n📊 Test 5: Measuring actual FPS (5 seconds)...")
    start_time = time.time()
    frame_count = 0
    
    while time.time() - start_time < 5:
        ret, frame = cap.read()
        if ret:
            frame_count += 1
    
    actual_fps = frame_count / 5.0
    print(f"   Measured FPS: {actual_fps:.1f}")
    print(f"   Reported FPS: {cap.get(cv2.CAP_PROP_FPS)}")
    
    cap.release()
    
    # Test with v4l2-ctl command
    print("\n📊 Test 6: Checking v4l2-ctl formats")
    import subprocess
    try:
        if isinstance(device, int):
            device_path = f"/dev/video{device}"
        else:
            device_path = device
            
        cmd = f"v4l2-ctl -d {device_path} --list-formats-ext | grep -A5 'MJPG\\|Motion-JPEG'"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print("   MJPG formats supported:")
            print(result.stdout)
        else:
            print("   Could not query v4l2-ctl")
    except:
        print("   v4l2-ctl not available")

def main():
    print("🔍 Camera FPS Debug Tool")
    print("This tool tests different configurations to diagnose FPS issues")
    
    # Test both device ID and device name
    devices_to_test = [
        (0, "Device ID: 0"),
        (1, "Device ID: 1"),
        ("usb-xhci-hcd.0-1", "Device name: usb-xhci-hcd.0-1"),
        ("usb-xhci-hcd.0-2", "Device name: usb-xhci-hcd.0-2"),
    ]
    
    for device, description in devices_to_test:
        try:
            test_camera_config(device, description)
        except Exception as e:
            print(f"❌ Error testing {device}: {e}")
    
    print("\n\n📋 Recommendations:")
    print("1. If device names show 10fps but IDs show 30fps, use device IDs")
    print("2. Ensure MJPG format is being used (not YUYV)")
    print("3. Try adding to your config:")
    print('   "fourcc": "MJPG"')
    print("4. Check USB bandwidth - use different USB ports")
    print("5. Use v4l2-ctl to set format before starting:")
    print("   v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG")
    print("   v4l2-ctl -d /dev/video0 --set-parm=30")

if __name__ == "__main__":
    main()