# ENPH 253 Competition Robot

Autonomous competition robot integrating dual sensing (vision + reflectance), YOLO object detection, articulated arm with inverse kinematics, and a split architecture (Raspberry Pi high‑level + ESP32 real‑time control).

<div align="center" markdown="1">
<img height="300" alt="image" src="https://github.com/user-attachments/assets/3ed423ad-08a1-4e6c-8404-085384f53286" />
<img height="300" alt="image" src="https://github.com/user-attachments/assets/eb1df924-2a02-4151-b028-1515a0551caf" />

<img height="400" alt="image" src="https://github.com/user-attachments/assets/271ba6ce-5686-4b20-8d25-800dc26e751f" />
<img height="400" alt="image" src="https://github.com/user-attachments/assets/880732a5-d1d4-409f-a941-ce01c639884a" />
<img height="400" alt="image" src="https://github.com/user-attachments/assets/9a0cc977-9c89-452c-bf60-5ee6a6419dc4" />

</div>

---
### Demonstration Videos
| Scenario | Link |
|----------|------|
| Full Mission | [YT link](https://youtu.be/eCi2i0b2N54) |
| Line Following | [YT link](https://youtu.be/UnH2nF_ivgA) |
| Pet Tracking Example | [YT link](https://youtu.be/3Meb8kBMPbk) |

---
### Quick Start
1. Flash ESP32 firmware (PlatformIO project in `esp32/`).
2. Connect ESP32 over USB; verify serial at 921600 baud.
3. On the Pi / dev machine:
```bash
python3 -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r pi/requirements.txt
python pi/robot_controller.py --serial-port /dev/ttyUSB0 --enable-gui
```
4. Confirm cameras enumerate (GUI or log output).
5. Begin mission (line following, detection, pickup sequence).

---
### Repository Layout
| Path | Purpose |
|------|---------|
| `cad/` | Laser-cut DXF plates, 3D printed STL mounts, arm + camera hardware revisions. |
| `esp32/` | Low-level real‑time firmware: motors, servos, reflectance sensors, OLED UI, command parser. |
| `pi/` | High‑level Python subsystems: camera manager, line following (vision), object detection, arm + motor coordination, GUIs & diagnostics. |
| `pi/*.json` | Runtime configuration (cameras, serial, object detection, chunk configs). |
| `pi/YOLO_MODELS_GUIDE.md` | Model selection & notes. |
| `pi/WINDOWS_SETUP.md` | Windows-specific environment setup. |
| `pi/DOCUMENTATION.md` | Detailed vision + line following internals (imported into this README summary). |
| `main/` | (Reserved) |

---
### System Architecture
```
	Raspberry Pi (Python)                               ESP32 (Firmware)
┌──────────────────────────────┐                  ┌──────────────────────────┐
│ camera_manager.py (threads)  │  serial cmds     │ motor.cpp                │
│ line_following_manager.py    │────────────────► │ linefollower.cpp (PID)   │
│ object_detection_manager.py  │                  │ arm.cpp (IK, servos)     │
│ motor_controller.py          │  serial cmds     │ pi.cpp (command parser)  │
│ arm_controller.py            │◄─────────────────│ input_display.cpp (UI)   │
│ robot_controller.py (or GUI) │  status / ACK    │ custom_servo.cpp         │
└──────────────────────────────┘                  └──────────────────────────┘
```
Closed-loop layering:
* High-level (Pi): vision line extraction, object detection, mission state, arm trajectory requests.
* Low-level (ESP32): deterministic PWM, reflectance PID, servo motion profiles, safety handling.

---
### Firmware (ESP32)
Location: `esp32/`

Build & flash:
```bash
pio run -t upload
```
Key classes:
* `MotorController` – speed limiting, direction control, min/max enforcement.
* `LineFollower` – FreeRTOS task computing line position from 4 analog sensors; PID (Kp,Ki,Kd,Ko) -> differential motor commands.
* `ServoController` – Multi-servo abstraction: target angle, speed limiting, mirrored shoulder mapping, optional IK velocity mode, wrist lock.
* `PiComm` – Parses ASCII commands starting with `PI:`; returns `ESP:...` responses.
* `InputDisplay` – Debounced inputs + user feedback states (INIT, READY, RUNNING, ERROR).

Timing model:
* Main loop (~100 Hz) services serial + updates servos.
* Line following runs in its own task (consistent dt for PID deltaTime calculation).
* Servo motion increments based on elapsed millis (no blocking delays).

---
### High-Level Software (Raspberry Pi / PC)
Location: `pi/`

Install:
```bash
pip install -r pi/requirements.txt
```
Run main controller:
```bash
python pi/robot_controller.py --serial-port /dev/ttyUSB0 --enable-gui
```
Key modules:
* `camera_manager.py` – Thread-per-camera continuous capture, non-blocking latest frame API, optional downsample.
* `line_following_manager.py` – Vision line detector: multi-row scan, brightness threshold, brown rejection, adaptive search center.
* `object_detection_manager.py` – YOLO (see `object_detection_config_*.json`), optional crop to ROI for speed.
* `motor_controller.py` – Maps vision-derived line error to controller speed, then to raw motor speeds via serial (`PI:MC`), supports reflectance mode switching.
* `arm_controller.py` – High-level turret & IK commands packaging to serial (`PI:GP`, `PI:SP`, etc.).
* `robot_gui.py` / `object_detection_ui.py` / `line_following_ui.py` – Visualization & tuning.

---
### Command Protocol (Pi <-> ESP32)
ASCII lines terminated by `\n`. Requests start with `PI:`. Selected handlers (see `pi.h`):

| Command | Format | Purpose |
|---------|--------|---------|
| Motors | `PI:MC,left,right` | Direct raw motor speeds (-255..255). |
| Motor base speed | `PI:LBS,value` | Set base speed used by reflectance PID. |
| Motor min speed | `PI:LMS,value` | Minimum PWM for movement. |
| Line follow toggle | `PI:LF,1|0` | Enable/disable reflectance line follower task. |
| PID gains | `PI:PID,kp,ki,kd,ko` | Set Kp,Ki,Kd,Ko. |
| Target position | `PI:TP,x` | Set desired line position (sensor space). |
| Sensor thresholds | `PI:ST,r1,l1,r2,l2` | Set analog voltage thresholds. |
| Reflectance sample | `PI:REF` | Request current sensor voltages. |
| Servo positions | `PI:SP,base,shoulder,elbow` | Direct joint targets (use '-' to skip). |
| Wrist position (unlock) | `PI:WP,angle` | Set wrist without lock. |
| Wrist lock toggle | `PI:WLT,1|0` | Enable/disable wrist lock mode. |
| Wrist lock angle | `PI:WLA,angle` | Set locked angle. |
| Claw angle | `PI:CP,angle` | Open/close claw. |
| Global IK pos | `PI:GP,x,y` | Cartesian target (inverse kinematics). |
| Global IK vel | `PI:GV,vx,vy` | Cartesian velocity mode. |
| Servo speeds | `PI:SS,base,shoulder,elbow,wrist` | Per-joint speed commands. |
| Servo max speeds | `PI:SMS,base,shoulder,elbow,wrist` | Per-joint speed caps. |
| Base angle query | `PI:BASE` | Return base joint angle. |
| Mission complete | `PI:COMPLETE` | Signal mission termination. |

Responses typically: `ESP:OK:msg`, `ESP:ERROR:reason`, status updates (e.g. `ESP:EMERGENCY_STOP`, `ESP:FINISH`).

Example exchange:
```
> PI:PID,30,0,0.5,2
< ESP:OK:PID Updated
> PI:LF,1
< ESP:OK:Line Following Started
```

---
### Core Algorithms (Summary)
#### Reflectance Line Following (Firmware)
4 analog sensors -> position estimate -> PID (Kp,Ki,Kd,Ko) -> differential motor PWM (clamped).

#### Vision Line Detection (Pi)
Row sampling + brightness threshold + brown rejection + weighted lateral error -> optional camera-mode PID.

#### Curve Detection
Angle difference lower vs upper line segments > threshold => phase transition cue.

#### Object Detection
YOLO model (config JSON) with optional ROI cropping triggers pickup routine.

#### Arm IK
Planar 2‑link solve with mechanical offsets; mirrored shoulder; wrist lock for end-effector orientation.

---
### Mission Flow (Reference)
1. System init & servo home.
2. Enable chosen line following mode.
3. Track line; detect curve events for state changes.
4. Object zone: detect & localize target.
5. Execute pickup (IK + claw) and deposit.
6. Resume or terminate mission.

---
### Safety
* Emergency stop halts motors, stops tasks, reports `ESP:EMERGENCY_STOP`.
* Reset button (long hold) returns to READY state.

---
### Competition Rules
Official competition rules for the robot can be found [here](https://docs.google.com/document/d/1uTwa8DMtBvLUz8N1geSQqHyW6fn-HD22/edit?rtpof=true&sd=true&tab=t.0).
Apologies in advance if this link is no longer functional in the future! Send me a message and I may be able to email you a PDF or something...


---
### Development
| Task | Example |
|------|---------|
| Flash firmware | `pio run -t upload` |
| Serial monitor | `pio device monitor -b 921600` |
| Run controller | `python pi/robot_controller.py --serial-port /dev/ttyUSB0 --enable-gui` |
| Vision line UI | `python pi/line_following_ui.py --camera 1` |
| Object detection test | `python pi/debug_object_detection.py` |

---
### Contributors
Made with love and big way by Ryan Cheng, David Oh, Zachary Xie, Bowen Yuan

<div align="center">
<img width="275" height="200" alt="image" src="https://github.com/user-attachments/assets/941ef203-157e-46ee-8f5e-a331419b92bb" />
<img width="275" height="200" alt="image" src="https://github.com/user-attachments/assets/eba62d67-5784-4a90-9bf9-058d834cf812" />
<br />
<img width="200" height="275" alt="image" src="https://github.com/user-attachments/assets/5fd0345f-b9b7-4322-8fc0-4be0fbd295a9" />
<img width="200" height="275" alt="image" src="https://github.com/user-attachments/assets/4df722c5-208c-4023-949b-51c94ad9994a" />
<img width="200" height="275" alt="image" src="https://github.com/user-attachments/assets/daf5efb1-d9fa-491a-96ce-1b56e44ad43a" />
</div>










