#!/usr/bin/env python3
"""
Real-time camera test for angle correction function.
Shows live camera feed with object detection and calculated angles.
"""

import cv2
import time
import numpy as np
from camera_manager import CameraManager
from object_detection_manager import ObjectDetectionManager

def calculate_angle_correction(bbox, camera_fov_degrees, frame_width):
    """
    Calculate angle correction needed to center object in frame.
    """
    # Calculate pet center X position
    pet_center_x = (bbox[0] + bbox[2]) / 2
    
    # Calculate frame center
    frame_center_x = frame_width / 2
    
    # Calculate pixel offset from center
    pixel_offset = pet_center_x - frame_center_x
    
    # Convert pixel offset to angle
    pixels_per_degree = frame_width / camera_fov_degrees
    angle_correction = pixel_offset / pixels_per_degree
    
    # Return negative angle (same as original function)
    return -angle_correction

def draw_detection_overlay(frame, detection_result, angle_correction, frame_center_x):
    """
    Draw detection overlay on frame with angle information.
    """
    height, width = frame.shape[:2]
    
    # Draw center line
    cv2.line(frame, (int(frame_center_x), 0), (int(frame_center_x), height), (0, 255, 0), 2)
    cv2.putText(frame, "CENTER", (int(frame_center_x) + 5, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    if detection_result.found:
        # Draw bounding box
        x1, y1, x2, y2 = detection_result.bbox
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        
        # Draw pet center
        pet_center_x = (x1 + x2) / 2
        pet_center_y = (y1 + y2) / 2
        cv2.circle(frame, (int(pet_center_x), int(pet_center_y)), 5, (255, 0, 0), -1)
        
        # Draw line from pet center to frame center
        cv2.line(frame, (int(pet_center_x), int(pet_center_y)), 
                (int(frame_center_x), int(pet_center_y)), (255, 255, 0), 2)
        
        # Text information
        info_y = 30
        line_height = 25
        
        # Class and confidence
        cv2.putText(frame, f"Class: {detection_result.class_name}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += line_height
        
        cv2.putText(frame, f"Confidence: {detection_result.confidence:.3f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += line_height
        
        # Pixel positions
        cv2.putText(frame, f"Pet Center: {pet_center_x:.1f}px", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += line_height
        
        cv2.putText(frame, f"Frame Center: {frame_center_x:.1f}px", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += line_height
        
        # Pixel offset
        pixel_offset = pet_center_x - frame_center_x
        cv2.putText(frame, f"Pixel Offset: {pixel_offset:.1f}px", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += line_height
        
        # Angle correction
        angle_color = (0, 255, 0) if abs(angle_correction) < 1.0 else (0, 165, 255) if abs(angle_correction) < 5.0 else (0, 0, 255)
        cv2.putText(frame, f"Angle: {angle_correction:.2f}°", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, angle_color, 2)
        info_y += line_height
        
        # Direction
        direction = "CENTERED" if abs(angle_correction) < 1.0 else ("TURN RIGHT" if angle_correction > 0 else "TURN LEFT")
        cv2.putText(frame, f"Direction: {direction}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, angle_color, 2)
        
        # Show bbox coordinates at bottom
        bbox_text = f"BBox: [{int(x1)}, {int(y1)}, {int(x2)}, {int(y2)}]"
        cv2.putText(frame, bbox_text, 
                   (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    else:
        # No detection
        cv2.putText(frame, "NO DETECTION", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    return frame

def main():
    print("🎥 Real-time Angle Correction Test")
    print("=" * 50)
    print("Controls:")
    print("  ESC/Q - Quit")
    print("  SPACE - Pause/Resume")
    print("  F - Toggle FOV (70° / 120°)")
    print("  C - Print current detection to console")
    print("  S - Save current frame")
    print("=" * 50)
    
    # Configuration
    camera_id = 2  # Object detection camera
    fov_options = [70, 120]  # Different FOV values to test
    current_fov_idx = 0
    camera_fov = fov_options[current_fov_idx]
    
    # Initialize systems
    try:
        print("🎥 Initializing camera manager...")
        camera_manager = CameraManager()
        camera_manager.start()
        time.sleep(1)  # Let camera initialize
        
        print("🎯 Initializing object detection...")
        object_detection_manager = ObjectDetectionManager(camera_manager, camera_id=camera_id)
        
        # Get initial frame to determine dimensions
        frame = camera_manager.get_frame(camera_id)
        if frame is None:
            print("❌ Could not get frame from camera")
            return
        
        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2
        
        print(f"📐 Camera settings:")
        print(f"   Frame size: {frame_width}x{frame_height}")
        print(f"   Frame center: {frame_center_x}")
        print(f"   Current FOV: {camera_fov}°")
        print(f"   Pixels per degree: {frame_width/camera_fov:.2f}")
        
        paused = False
        frame_count = 0
        
        while True:
            if not paused:
                # Get current frame
                frame = camera_manager.get_frame(camera_id)
                if frame is None:
                    print("❌ Lost camera connection")
                    break
                
                # Run object detection
                detection_result = object_detection_manager.run_detection()
                
                # Calculate angle correction if detection found
                angle_correction = 0.0
                if detection_result.found:
                    angle_correction = calculate_angle_correction(
                        detection_result.bbox, camera_fov, frame_width
                    )
                
                # Draw overlay
                display_frame = draw_detection_overlay(
                    frame.copy(), detection_result, angle_correction, frame_center_x
                )
                
                frame_count += 1
            
            # Add FOV info to top right
            fov_text = f"FOV: {camera_fov}° (F to change)"
            cv2.putText(display_frame, fov_text, 
                       (frame_width - 200, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Add status info
            status_text = "PAUSED" if paused else f"Frame: {frame_count}"
            cv2.putText(display_frame, status_text, 
                       (frame_width - 120, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Display frame
            cv2.imshow("Real-time Angle Correction Test", display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):  # ESC or Q
                break
            elif key == ord(' '):  # SPACE
                paused = not paused
                print(f"{'⏸️  Paused' if paused else '▶️  Resumed'}")
            elif key == ord('f'):  # F - toggle FOV
                current_fov_idx = (current_fov_idx + 1) % len(fov_options)
                camera_fov = fov_options[current_fov_idx]
                print(f"📐 FOV changed to {camera_fov}°")
            elif key == ord('c'):  # C - print current detection
                if detection_result.found:
                    pet_center_x = (detection_result.bbox[0] + detection_result.bbox[2]) / 2
                    pixel_offset = pet_center_x - frame_center_x
                    pixels_per_degree = frame_width / camera_fov
                    
                    print(f"\n📊 Current Detection Details:")
                    print(f"   Class: {detection_result.class_name}")
                    print(f"   Confidence: {detection_result.confidence:.3f}")
                    print(f"   BBox: {detection_result.bbox}")
                    print(f"   Pet center: {pet_center_x:.1f}px")
                    print(f"   Frame center: {frame_center_x:.1f}px")
                    print(f"   Pixel offset: {pixel_offset:.1f}px")
                    print(f"   Pixels per degree: {pixels_per_degree:.2f}")
                    print(f"   Angle correction: {angle_correction:.2f}°")
                    print(f"   Turret action: {'Turn RIGHT' if angle_correction > 0 else 'Turn LEFT' if angle_correction < 0 else 'Already centered'}")
                else:
                    print("\n📊 No detection currently")
            elif key == ord('s'):  # S - save frame
                filename = f"angle_test_frame_{int(time.time())}.jpg"
                cv2.imwrite(filename, display_frame)
                print(f"💾 Saved frame: {filename}")
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("🧹 Cleaning up...")
        cv2.destroyAllWindows()
        if 'object_detection_manager' in locals():
            object_detection_manager.cleanup()
        if 'camera_manager' in locals():
            camera_manager.stop()
        print("✅ Cleanup complete")

if __name__ == "__main__":
    main()