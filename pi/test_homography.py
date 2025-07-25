#!/usr/bin/env python3
"""
Test homography transformation for downward-facing line following camera.
This helps calibrate the perspective correction angle without modifying config.
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

def apply_angle_homography(frame, angle_degrees):
    """Apply homography transformation based on angle (without modifying camera config)."""
    if angle_degrees == 0:
        return frame
    
    height, width = frame.shape[:2]
    
    # Convert angle to radians
    angle_rad = np.radians(angle_degrees)
    
    # Calculate perspective correction based on camera angle
    top_compression = np.tan(angle_rad) * 0.3
    
    # Source points (what the tilted camera sees - trapezoid)
    margin_x = width * 0.1
    margin_y = height * 0.1
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
    
    try:
        # Calculate and apply homography matrix
        homography_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(frame, homography_matrix, (width, height))
        return warped
    except Exception as e:
        print(f"❌ Homography error: {e}")
        return frame

def main():
    print("🔍 Angle-Based Homography Calibration Tool")
    print("=" * 50)
    print("This tool helps you find the correct angle without modifying camera config.")
    
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
    
    # Get current homography settings from config
    camera_config = manager.config[f'camera_{camera_id}']
    current_config_angle = camera_config.get('homography', {}).get('correction_angle', 0.0)
    config_enabled = camera_config.get('homography', {}).get('enabled', False)
    
    print(f"📋 Current config: angle={current_config_angle}°, enabled={config_enabled}")
    
    # Start with current config angle or 0
    test_angle = current_config_angle if config_enabled else 0.0
    show_grid = True
    preview_mode = False  # False = raw, True = with test angle applied
    
    print("\n🎮 Controls:")
    print("   SPACE - Toggle preview (raw vs angle-corrected)")
    print("   W/X - Adjust angle by ±1° (W=up, X=down)")
    print("   A/D - Adjust angle by ±5° (A=left/down, D=right/up)")
    print("   Arrow keys - Also work for angle adjustment (if supported)")
    print("   R - Reset to current config angle")
    print("   G - Toggle grid overlay")
    print("   S - Show current angle (for manual config update)")
    print("   Q - Quit")
    print(f"\n📐 Starting angle: {test_angle:.1f}°")
    print("💡 Tip: Toggle preview and adjust angle until vertical/horizontal lines look straight")
    
    try:
        while True:
            # Always get raw frame to avoid interference with config
            frame = manager.get_raw_frame(camera_id)
            
            if frame is None:
                continue
            
            # Apply test angle transformation if in preview mode
            if preview_mode:
                display = apply_angle_homography(frame, test_angle)
                mode = f"PREVIEW: {test_angle:.1f}°"
            else:
                display = frame.copy()
                mode = "RAW"
            
            # Draw grid if enabled
            if show_grid:
                display = draw_grid_overlay(display)
            
            # Add status text
            cv2.putText(display, f"Mode: {mode}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display, f"Test Angle: {test_angle:.1f}°", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(display, f"Config: {current_config_angle:.1f}° ({'ON' if config_enabled else 'OFF'})", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Show frame
            cv2.imshow(f"Angle Homography Calibration - Camera {camera_id}", display)
            
            # Handle keyboard input - use full key code for better Linux compatibility
            key = cv2.waitKey(1) & 0xFFFF  # Use full 16-bit key code
            
            if key == ord('q'):
                break
            elif key == ord(' '):  # Spacebar
                preview_mode = not preview_mode
                mode_text = "PREVIEW" if preview_mode else "RAW"
                print(f"🔄 Switched to {mode_text} mode")
            elif key == ord('g') or key == ord('G'):
                show_grid = not show_grid
                print(f"📐 Grid {'ON' if show_grid else 'OFF'}")
            elif key == ord('r') or key == ord('R'):
                test_angle = current_config_angle
                print(f"🔄 Reset to config angle: {test_angle:.1f}°")
            elif key == ord('s') or key == ord('S'):
                print(f"\n💾 Current test angle: {test_angle:.1f}°")
                print(f"To update your camera_config.json, change the correction_angle to:")
                print(f'"homography": {{')
                print(f'    "enabled": true,')
                print(f'    "correction_angle": {test_angle:.1f}')
                print(f'}}')
                print()
            # Letter-based controls (more reliable than arrow keys)
            elif key == ord('w') or key == ord('W'):  # W = increase angle by 1
                test_angle += 1.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (+1)")
            elif key == ord('x') or key == ord('X'):  # X = decrease angle by 1
                test_angle -= 1.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (-1)")
            elif key == ord('d') or key == ord('D'):  # D = increase angle by 5
                test_angle += 5.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (+5)")
            elif key == ord('a') or key == ord('A'):  # A = decrease angle by 5
                test_angle -= 5.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (-5)")
            # Try common Linux arrow key codes
            elif key == 65362:  # Up arrow on some Linux systems
                test_angle += 1.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (↑)")
            elif key == 65364:  # Down arrow on some Linux systems
                test_angle -= 1.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (↓)")
            elif key == 65363:  # Right arrow on some Linux systems
                test_angle += 5.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (→)")
            elif key == 65361:  # Left arrow on some Linux systems
                test_angle -= 5.0
                test_angle = max(-90, min(90, test_angle))
                print(f"📐 Angle: {test_angle:.1f}° (←)")
            # Debug: show unknown key codes (remove this after testing)
            elif key != 65535 and key != 0:  # Ignore no-key-pressed values
                print(f"🔍 Debug: Unknown key code {key} (0x{key:x})")
    
    except KeyboardInterrupt:
        print("\n🛑 Calibration interrupted")
    finally:
        cv2.destroyAllWindows()
        manager.stop()
        print(f"✅ Calibration complete")
        print(f"📐 Final test angle: {test_angle:.1f}°")

if __name__ == "__main__":
    main()