#!/usr/bin/env python3
"""
Quick robot command tester - Interactive mode
"""

import serial
import time
import threading

def main():
    port = input("Serial port [/dev/ttyUSB0]: ").strip() or "/dev/ttyUSB0"
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        print(f"Connected to {port}")
        print("Type commands to send (or 'quit' to exit)")
        print("Examples:")
        print("  PI:MC,100,100    - Move forward")
        print("  PI:SP,90,90,90   - Home arm position") 
        print("  PI:GP,25,20      - Move arm to position")
        print("  PI:STATUS        - Get status")
        print()
        
        # Listen for responses
        def listen():
            while True:
                if ser.in_waiting:
                    response = ser.readline().decode().strip()
                    if response:
                        print(f"← {response}")
        
        listener = threading.Thread(target=listen, daemon=True)
        listener.start()
        
        # Send commands
        while True:
            cmd = input("→ ").strip()
            if cmd.lower() in ['quit', 'exit', 'q']:
                break
            if cmd:
                ser.write((cmd + '\n').encode())
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()