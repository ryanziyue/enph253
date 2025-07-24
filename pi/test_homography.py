#!/usr/bin/env python3
"""
Test homography transformation for downward-facing line following camera.
This helps calibrate the perspective correction for better line detection.
"""

import cv2
import numpy as np
import time
from camera_manager import CameraManager

def draw_grid_overlay(frame, grid_size=10):
    """Draw a grid on the frame to visualize perspective distortion."""
    height, width = frame.shape[:2]
    
    # Draw vertical lines
    for i in range(0, width, width // grid_size):
        cv2.line(frame, (i, 0), (i, height), (0, 255, 0), 1)
    
    # Draw horizontal lines
    for i in range(0, height, height // grid_size):
        cv2.line(frame, (0, i), (width, i), (0, 255, 0), 1)
    
    return frame

def main():
    print("🔍 Homography Calibration Tool")
    print("=" * 50)
    
    # Initialize camera manager
    manager = CameraManager()
    manager.start()
    
    # Wait for cameras to initialize
    time.sleep(2)
    
    # Camera to calibrate (1 = line following camera)
    camera_id = 1
    
    # Get initial frame to determine size
    frame = manager.get_raw_frame(camera_id)
    if frame is None:
        print("❌ Could not get frame from camera")
        manager.stop()
        return
    
    height, width = frame.shape[:2]
    print(f"📷 Camera {camera_id} resolution: {width}x{height}")
    
    # Example homography points for a downward-facing camera
    # These points define the perspective transform
    # Adjust based on your camera's mounting angle and height
    
    # Source points (trapezoid shape typical for angled camera)
    # Top of image appears further away, bottom appears closer
    src_points = [
        [width * 0.2, height * 0.3],   # Top-left
        [width * 0.8, height * 0.3],   # Top-right
        [width * 0.95, height * 0.95], # Bottom-right
        [width * 0.05, height * 0.95]  # Bottom-left
    ]
    
    # Destination points (rectangle for bird's eye view)
    dst_points = [
        [width * 0.2, height * 0.1],   # Top-left
        [width * 0.8, height * 0.1],   # Top-right
        [width * 0.8, height * 0.9],   # Bottom-right
        [width * 0.2, height * 0.9]    # Bottom-left
    ]
    
    # Interactive mode flag
    interactive = True
    homography_enabled = False
    
    print("\n🎮 Controls:")
    print("   H - Toggle homography on/off")
    print("   S - Save current homography settings")
    print("   R - Reset to default points")
    print("   G - Toggle grid overlay")
    print("   Q - Quit")
    print("\n💡 Tip: Place a rectangular pattern (like a checkerboard) under the camera")
    print("   Adjust the source points so it appears rectangular after transform")
    
    show_grid = True
    
    try:
        while True:
            # Get current frame
            if homography_enabled:
                # Get processed frame (with homography applied)
                frame = manager.get_frame(camera_id)
                mode = "HOMOGRAPHY ON"
            else:
                # Get raw frame
                frame = manager.get_raw_frame(camera_id)
                mode = "RAW"
            
            if frame is None:
                continue
            
            # Create display frame
            display = frame.copy()
            
            # Draw grid if enabled
            if show_grid:
                display = draw_grid_overlay(display)
            
            # Draw source/destination points
            if not homography_enabled:
                # Draw source points on raw image
                for i, pt in enumerate(src_points):
                    cv2.circle(display, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
                    cv2.putText(display, f"S{i+1}", (int(pt[0])+10, int(pt[1])), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Add status text
            cv2.putText(display, f"Mode: {mode}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show frame
            cv2.imshow(f"Homography Calibration - Camera {camera_id}", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('h') or key == ord('H'):
                homography_enabled = not homography_enabled
                if homography_enabled:
                    # Apply homography
                    manager.set_homography(camera_id, True, src_points, dst_points)
                    print("✅ Homography enabled")
                else:
                    # Disable homography
                    manager.set_homography(camera_id, False)
                    print("❌ Homography disabled")
            elif key == ord('g') or key == ord('G'):
                show_grid = not show_grid
                print(f"📐 Grid {'ON' if show_grid else 'OFF'}")
            elif key == ord('r') or key == ord('R'):
                # Reset to default points
                src_points = [
                    [width * 0.2, height * 0.3],
                    [width * 0.8, height * 0.3],
                    [width * 0.95, height * 0.95],
                    [width * 0.05, height * 0.95]
                ]
                print("🔄 Reset to default points")
                if homography_enabled:
                    manager.set_homography(camera_id, True, src_points, dst_points)
            elif key == ord('s') or key == ord('S'):
                # Save current settings
                print("\n💾 Current homography settings:")
                print(f"Source points: {src_points}")
                print(f"Destination points: {dst_points}")
                print("\nAdd these to your camera_config.json:")
                print(f'"homography": {{')
                print(f'    "enabled": true,')
                print(f'    "source_points": {src_points},')
                print(f'    "destination_points": {dst_points}')
                print(f'}}')
    
    except KeyboardInterrupt:
        print("\n🛑 Calibration interrupted")
    finally:
        cv2.destroyAllWindows()
        manager.stop()
        print("✅ Calibration complete")

if __name__ == "__main__":
    main()