# ESP32 Wheel-Legged Robot - Development Progress

---

## 2025/11/1 ~ 11/7 - Project Setup & Hardware Foundation

- Set up ESP32 + PlatformIO + Arduino framework project
- Implemented CAN bus communication module (1Mbps, GPIO33/32) for motor controller data exchange
- Initialized 6 motors: 4 joint motors (4310 type) + 2 wheel motors (2805 type), with back-EMF compensation
- Integrated MPU6050 IMU via DMP for yaw/pitch/roll attitude data and angular velocities
- Created motor send task (2ms cycle) and CAN receive task for real-time feedback

## 2025/11/8 ~ 11/15 - PID Controllers & Yaw Control

- Implemented cascade PID controller framework (inner + outer loop)
- Completed Roll PID: controls body tilt via left/right leg force differential
- Completed Leg Length PID: controls virtual leg length with integral and low-pass filtering
- Completed Leg Angle Difference PID: ensures left/right leg angle consistency
- Added gravity feedforward compensation
- Completed Yaw PID controller for turning functionality

## 2025/11/16 ~ 11/22 - Ground Detection & Landing

- Implemented ground support force calculation based on leg force, gravity, and acceleration
- Touch detection logic: both legs > 3N threshold = grounded
- Added 1-second anti-bounce mechanism to prevent false liftoff detection
- Disabled wheel motors when airborne, keeping legs vertical only
- Implemented landing cushioning state machine: marks cushioning on ground contact, exits when leg compresses to target length
- Reset position target on landing to prevent accumulated position error

## 2025/11/23 ~ 11/30 - Angle Protection & Standup

- Added leg angle protection: shuts down all motors when leg angle exceeds safe range or pitch is too large
- 2-second recovery delay after protection triggers to prevent oscillation
- Implemented split-leg standup (`Ctrl_StandupPrepareTask`): left leg swings back, right leg swings forward
- Standup state machine: None -> Prepare -> Standup -> None
- Known issue: split standup is too aggressive for an already-standing robot

## 2025/12/1 ~ 12/7 - Jump Functionality

- Implemented `Ctrl_JumpPrepareTask` as independent FreeRTOS task
- Jump sequence: crouch (lower leg length) -> max torque push -> quick leg retraction
- Task self-deletes after completion

## 2025/12/8 ~ 12/14 - Xbox BLE Controller Integration

- Integrated NimBLE-Arduino library for Xbox Series X wireless controller support
- Auto-scan and connect to "Xbox Wireless Controller"
- Button mappings:
  - Left Stick Y: forward/backward speed
  - Right Stick X: yaw turning speed
  - Left Stick X: roll offset
  - Right Stick Y: leg length adjustment
  - A: standup / B: jump / Y: balance toggle / X: cross-step toggle
- BLE processing task running at 50Hz

## 2025/12/15 ~ 12/22 - Motor Calibration, Persistence & Voltage Monitoring

- Implemented automatic motor direction detection with user confirmation
- Zero-offset calibration: place legs horizontal, compute offsetAngle
- Parameter save/load via ESP32 Preferences (NVS Flash)
- Serial debug commands: `clearmotor`, `imu`, `clearimu`, `log`, `nolog`
- Integrated ADS1115 ADC for battery voltage monitoring
- Dynamic motor output ratio compensation based on battery voltage

## 2025/12/23 ~ 12/31 - Serial Command Queue System

- Implemented UART2 (GPIO16/17) command receiving task
- Command queue system: max 20 commands, sequential execution
- Movement commands: `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `STOP`
- Action commands: `JUMP`, `STANDUP`, `CROSSLEG`
- Leg adjustment: `INCREASELEGLENGTH`, `DECREASELEGLENGTH`
- Delay command: `DELAY`
- Queue control: `QUEUE_START`, `QUEUE_PAUSE`, `QUEUE_RESUME`, `QUEUE_STOP`, `QUEUE_STATUS`
- Composite commands via semicolon separation (e.g. `FORWARD,50,3;LEFT,90,2;STOP`)

---

## 2026/1/1 ~ 1/30 - Exam Break (No Development)

---

## 2026/2/1 ~ 2/2 - Cross-Step Walking & Input Arbitration

- Implemented cross-step walking: alternating left/right leg angle oscillation
- X button toggle with 300ms debounce, auto leg length increase when enabled
- Added BLE input enable/disable interface (`BLE_SetInputEnabled` / `BLE_GetInputEnabled`)
- Serial queue automatically overrides BLE input when active
- Added serial commands: `BALANCE_ON`, `BALANCE_OFF`, `BLE_DISABLE`, `BLE_ENABLE`, `BLE_STATUS`

## 2026/2/2 ~ 2/10 - MATLAB Code Generation & Leg Kinematics

- Generated forward kinematics code (`leg_position`) in MATLAB: joint angles to leg length & angle
- Generated velocity kinematics code (`leg_speed`): joint speeds to leg linear/angular velocity
- Generated VMC conversion code (`leg_vmc_conv`): virtual force/torque to joint motor torques
- Generated LQR feedback gain matrix (`lqr_k`) in MATLAB, dynamically computed based on leg length
- Built `LegPos_UpdateTask` for real-time leg pose updates at 4ms intervals
- Added low-pass filtered leg length acceleration calculation
- Defined 6-dimensional state vector: leg angle, position, pitch angle, and their derivatives
- Implemented `CtrlBasic_Task` main control loop at 4ms cycle with LQR balance control
- Wheel motor output = LQR output + Yaw PID output

---

## 2026/2/15 - Trigger Functions & Gentle Standup

- **Problem**: Pressing Y button caused motors to spin chaotically, robot could not balance
  - **Root cause**: Y button triggered split standup (`Ctrl_StandupPrepareTask`), which enters `StandupState_Prepare` and pauses balance control; the split pose conflicts with the robot's current standing posture
- **Solution**: Added trigger button functions to avoid the split standup action
  - Left Trigger (LT): deep press toggles balance on/off
  - Right Trigger (RT): triggers gentle standup (`Ctrl_GentleStandupTask`)
- **Gentle standup implementation**:
  - Does NOT enter `StandupState_Prepare`, balance control keeps running
  - Only sets target values (leg length, roll, speed) and lets PID/LQR smoothly adjust
  - Waits 800ms then resets position and yaw targets to current actual values

## 2026/2/16 - Leg Motor Test Mode

- **Problem**: Pressing left trigger caused robot to twitch; need to rule out motor hardware issues
  - **Analysis**: LT toggles `balanceEnabled` to false, causing `CtrlBasic_Task` to immediately zero all 6 motor torques, robot instantly loses support
- **Solution**: Changed right trigger to leg test mode for isolated motor diagnosis
  - Added `legTestMode` global flag
  - RT held down (trigRT > 500) enters test mode, release exits immediately
  - In test mode (balance must be off):
    - Left leg: +3N downward force (extend)
    - Right leg: -3N upward force (retract)
    - Forces converted to joint torques via VMC
    - Wheel motors remain off
  - Small force value (3N) ensures slow, safe movement
- **Debug procedure**:
  1. Ensure balance is off (`balanceEnabled = false`)
  2. Hold RT and observe if left leg slowly extends, right leg slowly retracts
  3. If motors don't respond or move in wrong direction, indicates motor or CAN issue
  4. Release RT to immediately zero all motors
  5. Use serial `log` command to monitor joint angles in real time

## 2026/2/17 - IMU Serial Debug Output & Roll Diagnosis

- **Problem**: Robot drifts to one side during balancing, suspected Roll axis issue; need to check if IMU is functioning correctly
- **Solution**: Added `imulog` / `noimulog` serial commands to continuously output IMU data via USB serial
  - Output format: `R:0.12 P:-0.35 Y:2.10 | dR:0.01 dP:-0.02 dY:0.00` (degrees and deg/s)
  - Updates every 100ms in `Log_Task`
- **Diagnosis steps**:
  1. Connect USB serial at 115200 baud, send `imulog`
  2. Place robot on level surface, check if Roll and Pitch are near 0 degrees (within +/-2 deg is normal)
  3. Slowly tilt robot left/right, verify Roll changes smoothly and symmetrically
  4. Return to level, confirm Roll returns to near 0
  5. If Roll shows large static offset, send `imu` to recalibrate (keep robot still and level)
  6. Send `noimulog` to stop output
