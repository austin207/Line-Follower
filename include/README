## Complete System Architecture - Logical Flow

### **Project Structure Overview**

System is organized into **7 modular headers** + **1 main control file**:

```
include/
├── BLE/BLEConfig.h          → Wireless communication
├── Calibration/Calibration.h → Sensor calibration
├── IR/IRSensor.h            → Hardware sensor reading
├── LinePos/Linepos.h        → Signal processing
├── Motor/MotorControl.h     → Motor driver interface
├── PID/PID.h                → Control algorithm
└── PIDController/PIDController.h → PID orchestration
src/
└── main.cpp                 → System orchestrator
```

***

## **Detailed Logical Flow**

### **Layer 1: Hardware Interface Layer**

**IRSensor.h** - Raw sensor data acquisition
- **Input**: GPIO pins (13, 12, 14, 27, 26, 25, 33, 32)
- **Process**: `analogRead()` from 8 IR sensors
- **Output**: `sensorValues[8]` array (0-4095 analog values)
- **Purpose**: Interface with physical sensors

**MotorControl.h** - Motor actuation
- **Input**: Speed commands (-255 to +255)
- **Process**: PWM generation via LEDC, TB6612FNG control logic
- **Output**: Motor movements (forward/backward/turn)
- **Pins**: D5(PWMA), D18(AIN2), D19(AIN1), D21(BIN1), D22(BIN2), D23(PWMB), D17(STBY)

***

### **Layer 2: Signal Processing Layer**

**Calibration.h** - Sensor normalization
- **Input**: `sensorValues[8]` from IRSensor.h
- **Process**: 
  - Collect min/max values over 5 seconds
  - Calculate thresholds: `(min + max) / 2`
  - Normalize to 0-1000 range
  - Save to ESP32 flash via Preferences library[8]
- **Output**: Calibrated sensor readings, thresholds
- **Purpose**: Adapt to different lighting/surface conditions

**Linepos.h** - Position calculation
- **Input**: `sensorValues[8]` from IRSensor.h
- **Process**: Weighted average algorithm
  ```
  Position = Σ(sensorValue[i] × weight[i]) / Σ(active_sensors)
  Weights: [-4, -3, -2, -1, 1, 2, 3, 4]
  ```
- **Output**: 
  - Line position (negative=left, positive=right, 0=center)
  - -999 if line lost
- **Purpose**: Convert sensor array into single position value

***

### **Layer 3: Control Algorithm Layer**

**PID.h** - Core PID mathematics
- **Input**: 
  - Setpoint (target position = 0)
  - Current position from Linepos.h
  - Kp, Ki, Kd gains
- **Process**: 
  - P term = Kp × error
  - I term = Ki × ∫error dt (with anti-windup)
  - D term = Kd × (Δerror/Δt)
  - Output = P + I + D
- **Output**: Correction value (steering command)
- **Purpose**: Calculate precise steering correction

**PIDController.h** - PID lifecycle manager
- **Input**: 
  - Line position
  - BLE parameters (Kp, Ki, Kd, min/max output)
- **Process**:
  - Check if output limits changed → recreate PID
  - Check if gains changed → update tunings
  - Call `PID.compute()`
  - Handle line-lost cases
- **Output**: Correction value for motors
- **Purpose**: Manage dynamic PID parameter updates

***

### **Layer 4: Communication Layer**

**BLEConfig.h** - Wireless parameter control
- **Input**: BLE write commands from Flutter app
- **Process**: 
  - 9 BLE characteristics (Kp, Ki, Kd, MinOut, MaxOut, Position, Calibrate, BaseSpeed, EStop)
  - Characteristic callbacks update global variables
  - Notify Flutter app with live position data
- **Output**: 
  - Updated PID parameters
  - Trigger calibration
  - Emergency stop flag
- **Purpose**: Real-time wireless tuning without reflashing

***

### **Layer 5: System Orchestration Layer**

**main.cpp** - Control loop coordinator
- **Frequency**: 50Hz (20ms cycle time)
- **Process Flow**:
  1. Check emergency stop → halt if triggered
  2. Check calibration request → run calibration routine
  3. Read sensors → `readIRSensors()`
  4. Calculate position → `readLinePosition()`
  5. Update PID → `pidController.update(position)`
  6. Apply to motors → `applyPIDCorrection(correction)`
  7. Send BLE updates → `updatePositionValue(position)`
  8. Repeat
- **Purpose**: Orchestrate all subsystems into coherent behavior

***

## **Data Flow Diagram**

### **Complete System Flow**

```
HARDWARE LAYER:
[8 IR Sensors] → GPIO Pins → [IRSensor.h]
                                    ↓
                            sensorValues[8] (0-4095)
                                    ↓
                            [Calibration.h] ← Flash Memory (Preferences)
                                    ↓
                            Normalized values (0-1000)
                                    ↓
SIGNAL PROCESSING:          [Linepos.h]
                                    ↓
                            Line Position (-∞ to +∞)
                                    ↓
CONTROL ALGORITHM:          [PIDController.h] ← BLE Parameters
                                    ↓                ↑
                            [PID.h (P+I+D)]          |
                                    ↓                |
                            Correction (-255 to +255)|
                                    ↓                |
MOTOR ACTUATION:            [MotorControl.h]         |
                                    ↓                |
                            Left/Right Motor Speeds  |
                                    ↓                |
                            [TB6612FNG Driver]       |
                                    ↓                |
                            [DC Motors]              |
                                                     |
COMMUNICATION:              [BLEConfig.h] ←→ [Flutter App]
                                    ↑
                            User adjusts Kp, Ki, Kd, Speed
```

### **Control Loop Sequence**

```
1. Read Sensors      → IRSensor.h
2. Calibrate (opt)   → Calibration.h  
3. Calculate Pos     → Linepos.h
4. Update PID        → PIDController.h → PID.h
5. Apply Correction  → MotorControl.h
6. Drive Motors      → TB6612FNG → DC Motors
7. Notify BLE        → BLEConfig.h → Flutter App
8. Loop (50Hz)       → Repeat
```