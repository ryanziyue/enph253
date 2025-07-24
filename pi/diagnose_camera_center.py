#!/usr/bin/env python3
"""
Diagnostic tool to check camera centering issue.
Shows raw camera feed with grid overlay to identify if objects are being centered unexpectedly.
"""

import cv2
import numpy as np
import time
from camera_manager import CameraManager

def draw_grid(frame, grid_divisions=10):
    """Draw a grid overlay on the frame to help visualize position."""
    height, width = frame.shape[:2]
    
    # Create a copy to draw on
    overlay = frame.copy()
    
    # Draw vertical lines
    for i in range(1, grid_divisions):
        x = int(width * i / grid_divisions)
        cv2.line(overlay, (x, 0), (x, height), (0, 255, 0), 1)
        # Add percentage label
        cv2.putText(overlay, f"{i*10}%", (x-15, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    
    # Draw horizontal lines
    for i in range(1, grid_divisions):
        y = int(height * i / grid_divisions)
        cv2.line(overlay, (0, y), (width, y), (0, 255, 0), 1)
        # Add percentage label
        cv2.putText(overlay, f"{i*10}%", (5, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    
    # Draw center crosshair
    center_x, center_y = width // 2, height // 2
    cv2.line(overlay, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
    cv2.line(overlay, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)
    cv2.circle(overlay, (center_x, center_y), 30, (0, 0, 255), 2)
    
    # Add center label
    cv2.putText(overlay, "CENTER", (center_x - 30, center_y - 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Draw frame edges
    cv2.rectangle(overlay, (1, 1), (width-2, height-2), (255, 255, 0), 2)
    
    return overlay

def main():
    print("🔍 Camera Centering Diagnostic Tool")
    print("=" * 50)
    
    # Initialize camera manager
    manager = CameraManager()
    manager.start()
    
    print("\nWaiting for cameras to initialize...")
    time.sleep(2)
    
    # Get system status
    status = manager.get_system_status()
    
    # Check which cameras are active
    active_cameras = []
    for cam_id in [1, 2]:
        if status['cameras'][cam_id]['connected']:
            active_cameras.append(cam_id)
            cam_info = status['cameras'][cam_id]
            print(f"\n📷 Camera {cam_id} Info:")
            print(f"   Device ID: {cam_info['device_id']}")
            print(f"   Resolution: {cam_info['resolution'][0]}x{cam_info['resolution'][1]}")
            print(f"   FPS: {cam_info['fps']:.1f}")
    
    if not active_cameras:
        print("\n❌ No cameras connected!")
        manager.stop()
        return
    
    print("\n🎮 Controls:")
    print("   1/2 - Switch between cameras")
    print("   R - Show raw frame (no downsample)")
    print("   D - Show downsampled frame")
    print("   G - Toggle grid overlay")
    print("   S - Save current frame")
    print("   Q - Quit")
    print("\n👁️ Place an object to the LEFT of the camera and observe where it appears")
    
    # Start with first active camera
    current_camera = active_cameras[0]
    show_raw = True
    show_grid = True
    
    try:
        while True:
            # Get frame based on mode
            if show_raw:
                frame = manager.get_raw_frame(current_camera)
                mode_text = "RAW"
            else:
                frame = manager.get_frame(current_camera)
                mode_text = "PROCESSED"
            
            if frame is None:
                print(f"\r⚠️ No frame from camera {current_camera}", end='', flush=True)
                time.sleep(0.1)
                continue
            
            # Create display frame
            display_frame = frame.copy()
            
            # Add grid if enabled
            if show_grid:
                display_frame = draw_grid(display_frame)
            
            # Add info text
            height, width = display_frame.shape[:2]
            info_text = f"Camera {current_camera} | Mode: {mode_text} | Size: {width}x{height}"
            cv2.putText(display_frame, info_text, (10, height - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show instructions
            cv2.putText(display_frame, "Object LEFT of camera should appear LEFT in frame", 
                       (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Display
            cv2.imshow(f"Camera Diagnostic - Camera {current_camera}", display_frame)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('1') and 1 in active_cameras:
                current_camera = 1
                print(f"\n📷 Switched to Camera 1")
            elif key == ord('2') and 2 in active_cameras:
                current_camera = 2
                print(f"\n📷 Switched to Camera 2")
            elif key == ord('r') or key == ord('R'):
                show_raw = True
                print(f"\n🎞️ Showing RAW frames")
            elif key == ord('d') or key == ord('D'):
                show_raw = False
                print(f"\n🎞️ Showing PROCESSED frames")
            elif key == ord('g') or key == ord('G'):
                show_grid = not show_grid
                print(f"\n📐 Grid {'ON' if show_grid else 'OFF'}")
            elif key == ord('s') or key == ord('S'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"camera_diagnostic_{current_camera}_{timestamp}.jpg"
                cv2.imwrite(filename, display_frame)
                print(f"\n📸 Saved: {filename}")
    
    except KeyboardInterrupt:
        print("\n\n🛑 Diagnostic interrupted")
    finally:
        cv2.destroyAllWindows()
        manager.stop()
        print("✅ Diagnostic completed")

if __name__ == "__main__":
    main()