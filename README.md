## ENPH 253 Competition Robot

Highly modular autonomous robot for the ENPH 253 competition featuring dual sensing (camera + reflectance), YOLO-based object detection, articulated arm with IK, and a hybrid ESP32 + Raspberry Pi control stack.

---
## Quick Start (TL;DR)
1. Flash ESP32 firmware (PlatformIO) in `esp32/`.
2. Wire hardware per pin map (see Hardware > Electronics).
3. On Pi (or dev PC with cameras attached):
```bash
python3 -m venv .venv
source .venv/bin/activate  # (Windows: .venv\Scripts\activate)
pip install -r pi/requirements.txt
python pi/robot_controller.py --serial-port /dev/ttyUSB0 --enable-gui
```
4. Wait for cameras to initialize; send `PI:READY` from Pi to ESP32 (script does this automatically after handshake).
5. Start line following / mission sequence via GUI or custom control code.

---
## Repository Layout
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
## System Architecture
```
	Raspberry Pi (Python)                               ESP32 (Firmware)
┌──────────────────────────────┐                  ┌──────────────────────────┐
│ camera_manager.py (threads)  │  frames          │ motor.cpp                │
│ line_following_manager.py    │──────────────►   │ linefollower.cpp (PID)   │
│ object_detection_manager.py  │                  │ arm.cpp (IK, servos)     │
│ motor_controller.py          │  serial cmds     │ pi.cpp (command parser)  │
│ arm_controller.py            │◄──────────────┐  │ input_display.cpp (UI)   │
│ robot_controller.py (or GUI) │  status / ACK  │  │ custom_servo.cpp        │
└──────────────────────────────┘                  └──────────────────────────┘
```
Dual closed loops:
1. High-level (Pi): vision-based line geometry, mission logic, object detection, strategic arm placement.
2. Low-level (ESP32): deterministic timing for motor PWM, reflectance line PID, servo motion profiles, safety / emergency stop.

---
## Hardware
### Mechanical
Modules (see `cad/`):
* Chassis plates (rev folders) – width & mounting holes consistent across revisions.
* Arm assembly – multi-link with mirrored shoulder servos (left/right) and elbow + wrist + claw.
* Camera mounts: front (line following) + elevated arm camera (if used) `ArmCamMount/` & `Front[Cam,Sensor]Mount/`.
* Basket / payload handling components.

Recommended fabrication:
* DXF: 3 mm acrylic or plywood (confirm thickness in your BOM).
* STL: PETG or PLA+ at ≥30% infill for structural parts (turret base, camera stands).

### Electronics
| Component | Notes |
|----------|-------|
| ESP32 dev board | PWM for motors & 5+ servo channels (uses LEDC). |
| 2x DC gear motors | Driven via H-bridge (channels FWD/REV). |
| Reflectance sensors (4) | Analog inputs (R1,L1,R2,L2). Threshold calibrated in firmware. |
| Servos (Base, Shoulder L/R, Elbow, Wrist, Claw) | 5 logical joints + mirrored shoulder compensation. |
| OLED (SSD1306) | I2C status & mode display. |
| Start + Reset buttons, 2x mode switches | Debounced + long-hold reset logic. |
| 1–2 USB cameras | UVC 640x480 @ 30 FPS typical. |

Pin mapping (see `esp32/include/main.h` authoritative): motors, servos, analog channels, buttons.

Power guidelines:
* Separate regulated 5–6V servo rail; common ground with logic.
* Brown-out avoidance: large electrolytic (≥470 µF) near servo supply.
* USB camera(s) may need powered hub if Pi ports undervolt.

### Kinematics Constants (from `main.h`)
| Symbol | Value | Meaning |
|--------|-------|---------|
| ARM_L1 | 14.5 | First arm segment length (units consistent with IK input). |
| ARM_L2 | 12.547 | Second segment length. |
| ELBOW_OFFSET | 40.0 | Mechanical offset compensation (deg). |

Wrist mechanical limits: -75° .. +105° logical; servo aligned offset defined by `WRIST_SERVO_ALIGNED`.

---
## Firmware (ESP32)
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
## High-Level Software (Raspberry Pi / PC)
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

Performance targets:
| Subsystem | Typical Latency |
|-----------|-----------------|
| Frame capture | <5 ms (threaded) |
| Vision line scan (640x480, 10 rows) | 1–5 ms |
| Brown filter | +1–2 ms |
| YOLO11n (cropped) | ~30–70 ms (device dependent) |

---
## Command Protocol (Pi <-> ESP32)
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
## Algorithms
### Reflectance Line Follower (Firmware)
* 4 sensors -> positional estimate (weighted or geometric – see implementation).
* PID error = targetPosition - measuredPosition.
* Ko term (feed-forward / offset) widens high-speed stability band.
* Output -> left/right motor differentials with clamping to min/max.

Tuning order:
1. Set thresholds (`PI:ST`) so white > threshold, line < threshold.
2. Increase base speed until oscillation then back off.
3. Tune Kp for responsiveness; add small Kd to damp; keep Ki ~0 unless drift.
4. Adjust Ko to bias correction strength.

### Vision Line Detection (Pi)
Steps per frame:
1. Select N horizontal scan rows (configurable). 
2. For each row: threshold to find longest dark segment near adaptive center. 
3. Reject segments with brown% > threshold (color-space HSV ranges). 
4. Collect candidate points; weight by vertical position (exponential) to compute lateral error. 
5. PID in `motor_controller.py` converts error to speed correction (camera mode).

### Curve Detection
* Partition lower vs upper subsets of line points. 
* Compute angle each subset (linear regression vs vertical). 
* If |Δθ| > curvature_threshold => curve event (used for mission phase transitions).

### Object Detection (YOLO)
* Config file selects model weight + confidence threshold + class filters. 
* Optional ROI cropping (bottom half) reduces inference time. 
* Detection result triggers pet pickup sequence (arm positioning heuristics configurable in controller script).

### Arm Inverse Kinematics
* Given (x,y): solve planar 2‑link IK with offsets → (shoulder, elbow). 
* Shoulder mirroring: right shoulder servo angle derived from left via `convertToRightShoulderAngle`. 
* Wrist lock maintains orientation angle independent of upstream joint motion by applying compensatory rotation.

---
## Typical Mission Sequence (Example)
1. Initialize & zero servos (`arm.resetPosition()`).
2. Enable reflectance line follower: `PI:LF,1` (or camera mode PID if enabled).
3. Follow line until curve detection triggers phase change.
4. Slow down / switch to vision-based approach for object zone.
5. Object detection finds target -> arm IK position to staged pickup.
6. Wrist/claw sequence to grasp; retract to basket drop pose.
7. Resume navigation or signal mission complete.

---
## Calibration & Tuning Procedures
### Reflectance Sensors
```text
1. Stream `PI:REF` while moving sensor over line.
2. Note white vs black voltages; set midpoints using `PI:ST`.
3. Verify `offLine()` transitions filter correctly (command `sensors` via local serial).
```
### Vision Thresholds
* Adjust `max_brightness` in `LineFollowingManager` if ambient lighting changes.
* Brown rejection: tune `brown_threshold_percent` until false positives (floor artifacts) drop.

### Servo Motion
* `PI:SP,-,angle,-` to isolate single joint.
* Use `PI:SS` low speed values during mechanical testing.
* Verify wrist lock response: enable `PI:WLT,1`; move shoulder; wrist angle should hold.

### Motor Dead Zone
* Use `motor_controller.map_speed_to_motor` output logs (enable debug) to ensure first non-zero command overcomes static friction; adjust `min_motor_speed` accordingly.

---
## Safety / Fault Handling
* Emergency stop (`emergencyStop()`) halts motors, stops line follower task, freezes servos, displays ERROR, emits `ESP:EMERGENCY_STOP`.
* Long-hold Reset button (>1s) triggers system state reset (OLED progress bar).
* Always test new PID or arm parameters with wheels elevated or motors disabled first.

---
## Competition Rules (Summary Placeholder)
Fill in exact numeric limits from official PDF.
* Course: Line-following track with curves and object retrieval zone.
* Objectives: Navigate autonomously, identify & collect target objects ("pets"), deposit in goal/basket area.
* Scoring: Points for successful pickups, delivery, speed; penalties for boundary violations or manual intervention.
* Constraints: Max footprint WxLxH, weight limit, autonomous requirement after start button, safety (no exposed hazards).
* Run Limit: N attempts; best score counts (insert specifics).

(Add concrete values: size, time limits, scoring table once confirmed.)

---
## Demonstration Videos (Add Links)
| Topic | Link |
|-------|------|
| Full Mission Run | (YouTube URL) |
| Reflectance Line Following Tuning | (YouTube URL) |
| Vision Line Detection Overlay | (YouTube URL) |
| Object Detection + Pickup | (YouTube URL) |
| Arm IK & Wrist Lock Demo | (YouTube URL) |

---
## Development Workflow
| Task | Command / Action |
|------|------------------|
| Format / lint Python | (Add tooling if desired: black / flake8) |
| Flash firmware | `pio run -t upload` |
| Monitor serial | `pio device monitor -b 921600` |
| Test cameras | `python pi/camera_manager.py --duration 15` |
| Line UI | `python pi/line_following_ui.py --camera 1` |
| Object detection test | `python pi/debug_object_detection.py` |
| Arm diagnostics | Serial: `PI:SP,...` / use controller helper methods |

---
## Extending
* Add new Pi -> ESP32 command: implement handler in `pi.cpp` (PiComm) + public API in Python controller.
* Swap YOLO model: place weights, update `object_detection_config_*.json` path & class list.
* Add sensor: extend command protocol + status print in `printSystemStatus()`.

---
## Troubleshooting Cheat Sheet
| Symptom | Likely Cause | Action |
|---------|-------------|--------|
| Motors twitch, no motion | min speed too low | Raise `min_motor_speed` / check supply voltage. |
| Line oscillation | Kp too high / Kd zero | Lower Kp, introduce Kd. |
| Missed line (vision) | brightness threshold off | Adjust `max_brightness` or increase rows. |
| False brown rejections | HSV ranges too wide | Narrow brown mask or raise threshold. |
| Arm drift / overshoot | Speed caps too high | Lower with `PI:SMS`. |
| Wrist angle changes on move | Wrist lock disabled | `PI:WLT,1` + set lock angle. |
| YOLO slow | Full-frame inference | Enable crop / downsample. |
| Camera feed freezes | USB bandwidth / power | Reduce FPS or use powered hub. |

---
## License / Attribution
(Add license here if open-sourcing; otherwise state proprietary / educational use.)

---
## Contributors
Add team member names & roles.

---
## TODO (Future Improvements)
* Formal logging & telemetry stream.
* Automated calibration script for reflectance thresholds.
* Adaptive speed controller tied to curvature estimation.
* Add unit tests for IK & speed mapping.

---
Feel free to submit PRs or issues with improvements, tuning data, or integration scripts.

