# ESP32-Pi Communication Protocol

## Pi to ESP

### Motor Control
- `PI:MC,left_speed,right_speed` - Set motor speeds (-255 to 255)
- `PI:LF,toggle` - Toggle line following (1=on, 0=off)
- `PI:LFS,speed` - Set line following base speed (0-255)

### Arm Control
- `PI:SP,base,shoulder,elbow` - Set servo positions (0-180, "-" to skip)
- `PI:WP,angle` - Set wrist position (0-180°, temp disables lock for 5s)
- `PI:CP,angle` - Set claw position (0-180°)
- `PI:GP,x,y` - Set position of wrist joint using inverse kinematics
- `PI:GV,vx,vy` - Set end-effector velocity for continuous motion

### Wrist Lock Control
- `PI:WLT,toggle` - Toggle wrist lock (1=on, 0=off) [default: ON]
- `PI:WLA,angle` - Set wrist lock angle (-45° to 45°, 0=level)
- `PI:WLTD,duration_ms` - Temporarily disable wrist lock (limited to 100-30000ms)

### Status
- `PI:STATUS` - Request system status

## ESP to Pi

### Command Responses
- `OK: [message]` - Command successful
- `ERROR: [message]` - Command failed

### Status Data
- `ESP:Pos:(x,y) LF:ON/OFF Motors:L=x R=y` - System status response
- `ESP:x,y` - Position updates (if enabled)

### Automatic Notifications
- `ESP:LS` - Limit switch pressed
- `ESP:EMERGENCY_STOP` - Emergency stop activated
