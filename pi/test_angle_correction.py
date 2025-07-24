#!/usr/bin/env python3
"""
Standalone test script for angle correction function.
Run this to test the _calculate_angle_correction function without starting the full robot system.
"""

def calculate_angle_correction(bbox, camera_fov_degrees):
    """
    Standalone version of the angle correction function for testing.
    """
    # Frame width (hardcoded in original function)
    frame_width = 640
    
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

def main():
    print("🧪 Standalone Angle Correction Test")
    print("=" * 50)
    
    # Default test parameters
    default_fov = 70  # degrees
    frame_width = 320
    frame_center = frame_width / 2
    
    print(f"📐 Settings:")
    print(f"   Frame width: {frame_width} pixels")
    print(f"   Frame center: {frame_center} pixels")
    print(f"   Default FOV: {default_fov}°")
    
    # Test cases: [x1, y1, x2, y2, description]
    test_cases = [
        [140, 100, 180, 150, "Centered object (should be ~0°)"],
        [200, 100, 240, 150, "Object to the right"],
        [80, 100, 120, 150, "Object to the left"],
        [0, 100, 40, 150, "Object far left"],
        [280, 100, 320, 150, "Object far right"],
        [100, 100, 220, 150, "Wide object centered"],
        [240, 100, 280, 150, "Small object right side"],
        [40, 100, 80, 150, "Small object left side"],
    ]
    
    print(f"\n📊 Test Results:")
    print(f"{'Bbox':<25} {'Pet Center':<12} {'Frame Center':<13} {'Offset':<8} {'Angle':<8} {'Description'}")
    print("-" * 90)
    
    for bbox_coords in test_cases:
        x1, y1, x2, y2, description = bbox_coords
        bbox = [x1, y1, x2, y2]
        
        # Calculate angle correction
        angle_correction = calculate_angle_correction(bbox, default_fov)
        
        # Calculate components for display
        pet_center_x = (x1 + x2) / 2
        pixel_offset = pet_center_x - frame_center
        
        print(f"{str(bbox):<25} {pet_center_x:<12.1f} {frame_center:<13.1f} {pixel_offset:<8.1f} {angle_correction:<8.1f} {description}")
    
    print(f"\n🔍 Interactive Test:")
    while True:
        try:
            print(f"\nEnter bounding box coordinates (or 'q' to quit):")
            user_input = input("Format: x1,y1,x2,y2 (e.g., 140,100,180,150): ").strip()
            
            if user_input.lower() == 'q':
                break
            
            # Parse input
            coords = [float(x.strip()) for x in user_input.split(',')]
            if len(coords) != 4:
                print("❌ Please provide exactly 4 coordinates")
                continue
            
            x1, y1, x2, y2 = coords
            bbox = [x1, y1, x2, y2]
            
            # Optional: allow custom FOV
            fov_input = input(f"Camera FOV in degrees (default {default_fov}): ").strip()
            fov = float(fov_input) if fov_input else default_fov
            
            # Calculate angle correction
            angle_correction = calculate_angle_correction(bbox, fov)
            
            # Show detailed calculation
            pet_center_x = (x1 + x2) / 2
            pixel_offset = pet_center_x - frame_center
            pixels_per_degree = frame_width / fov
            
            print(f"\n📊 Calculation Details:")
            print(f"   Pet center X: {pet_center_x:.1f} pixels")
            print(f"   Frame center X: {frame_center:.1f} pixels")
            print(f"   Pixel offset: {pixel_offset:.1f} pixels")
            print(f"   Pixels per degree: {pixels_per_degree:.2f}")
            print(f"   Raw angle: {pixel_offset / pixels_per_degree:.2f}°")
            print(f"   Final angle correction: {angle_correction:.2f}°")
            print(f"   Direction: {'Turn RIGHT' if angle_correction > 0 else 'Turn LEFT' if angle_correction < 0 else 'Already centered'}")
            
            # Show what this means for turret movement
            print(f"\n🔄 Turret Movement:")
            if abs(angle_correction) < 1.0:
                print(f"   ✅ Pet is well centered (±1°)")
            elif abs(angle_correction) < 5.0:
                print(f"   ⚠️ Small adjustment needed: {abs(angle_correction):.1f}°")
            else:
                print(f"   🔄 Large adjustment needed: {abs(angle_correction):.1f}°")
            
        except ValueError:
            print("❌ Invalid input format. Use: x1,y1,x2,y2")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    print(f"\n✅ Test complete")

if __name__ == "__main__":
    main()