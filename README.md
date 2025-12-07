# ESP32 Line Follower Robot

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Ready-orange.svg)](https://platformio.org) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE) [![ESP32](https://img.shields.io/badge/ESP32-WROOM-green.svg)](https://www.espressif.com/en/products/socs/esp32)

A high-performance autonomous line following robot built with ESP32, featuring **wireless PID tuning via Bluetooth Low Energy (BLE)** and advanced sensor calibration. Perfect for robotics competitions, learning control systems, or building your own autonomous vehicle.

![Line Follower Demo](docs/images/robot-demo.gif)

---

## Features

### Core Functionality
- **PID Control Algorithm** - Smooth, accurate line following with minimal oscillation
- **8-Channel IR Sensor Array** - High-resolution line detection
- **Wireless Tuning** - Adjust PID parameters in real-time via BLE (no code upload needed!)
- **Auto-Calibration** - Adapts to different lighting conditions and track surfaces
- **Differential Steering** - Precise motor control using TB6612FNG dual motor driver

### Wireless Control (BLE)
- **Real-time PID Tuning** - Adjust Kp, Ki, Kd without reflashing
- **Speed Control** - Change base speed on-the-fly
- **Motor Limits** - Dynamic min/max output adjustment
- **Live Monitoring** - Stream line position data to your phone
- **Emergency Stop** - Instant wireless shutdown
- **Calibration Trigger** - Start sensor calibration remotely

### Advanced Algorithms
- **Weighted Average Position Calculation** - Accurate line position from sensor array
- **PID with Anti-Windup** - Prevents integral saturation
- **Persistent Calibration** - Saves to ESP32 flash memory (survives power-off)
- **Line Lost Detection** - Automatic safety stop when track is lost

### Developer-Friendly
- **Modular Architecture** - Clean separation of concerns (7 independent modules)
- **Extensive Documentation** - Beginner-friendly READMEs for every component
- **PlatformIO Project** - Modern build system with dependency management
- **Debug Utilities** - Serial monitor output for tuning and troubleshooting

---

## Demo

| Feature | Demo |
|---------|------|
| Line Following | ![Line Following](docs/images/line-follow.gif) |
| BLE Tuning | ![BLE Tuning](docs/images/ble-tuning.gif) |
| Calibration | ![Calibration](docs/images/calibration.gif) |

---

## Hardware Requirements

### Core Components

| Component | Specification | Quantity | Notes |
|-----------|---------------|----------|-------|
| **ESP32-WROOM** | Dual-core, WiFi/BLE | 1 | Development board (DevKit V1 or similar) |
| **IR Sensor Array** | QTR-8A/QTR-8RC | 1 | 8-channel analog reflectance sensor |
| **Motor Driver** | TB6612FNG | 1 | Dual H-bridge, 1.2A continuous per channel |
| **DC Motors** | 6V, N20/TT motors | 2 | Geared motors with wheels |
| **Battery** | 7.4V LiPo / 6xAA | 1 | 1000-2000mAh recommended |
| **Chassis** | - | 1 | Custom or commercial robot base |

### Pin Connections

**IR Sensor Array → ESP32:**
```
A1 → GPIO13 | A2 → GPIO12 | A3 → GPIO14 | A4 → GPIO27
A5 → GPIO26 | A6 → GPIO25 | A7 → GPIO33 | A8 → GPIO32
VCC → 3.3V/5V | GND → GND
```

**TB6612FNG Motor Driver → ESP32:**
```
PWMA → GPIO5  | AIN1 → GPIO19 | AIN2 → GPIO18
PWMB → GPIO23 | BIN1 → GPIO21 | BIN2 → GPIO22
STBY → GPIO17 | VCC → 3.3V    | GND → GND
VM → Battery+ (6-12V)
```

**Power:**
```
Battery → TB6612FNG VM & ESP32 VIN
Common Ground: Battery GND ↔ TB6612FNG GND ↔ ESP32 GND
```

> ⚠️ **Important:** Ensure common ground between ESP32 and motor driver!

---

## Software Requirements

### Development Environment
- [PlatformIO IDE](https://platformio.org/install) (VS Code extension recommended)
- OR [Arduino IDE](https://www.arduino.cc/en/software) 1.8.13+

### Libraries (Auto-installed via PlatformIO)
```
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
lib_deps =
    esp32@^2.0.0
```

### Optional: Flutter BLE Control App
- [Flutter SDK](https://flutter.dev/docs/get-started/install) 3.0+
- `flutter_blue_plus` package (for BLE communication)
- Coming soon: Pre-built APK for Android

---

## Quick Start

### **1.** Clone the Repository
```
git clone https://github.com/yourusername/esp32-line-follower.git
cd esp32-line-follower
```

### **2.** Open in PlatformIO
```
# Using VS Code with PlatformIO extension
code .

# Or use PlatformIO CLI
pio run
```

### **3.** Upload to ESP32
```
pio run --target upload
pio device monitor  # Open serial monitor
```

### **4.** Calibrate Sensors
- Power on the robot
- Connect via BLE app or send `'1'` to **CALIBRATE** characteristic
- Move robot over black line and white surface for 5 seconds
- Calibration auto-saves to flash memory

### **4.** Start Line Following
- Place robot on track (line centered under sensors)
- Robot automatically follows the line!
- Adjust PID via BLE if needed

---

## Project Structure

```
esp32-line-follower/
├── include/                    # Header files (modular architecture)
│   ├── BLE/
│   │   ├── BLEConfig.h        # Bluetooth Low Energy server & characteristics
│   │   └── README.md          # BLE documentation
│   ├── Calibration/
│   │   ├── Calibration.h      # Auto-calibration & persistent storage
│   │   └── README.md
│   ├── IR/
│   │   ├── IRSensor.h         # 8-channel IR sensor interface
│   │   └── README.md
│   ├── LinePos/
│   │   ├── Linepos.h          # Weighted average position calculation
│   │   └── README.md
│   ├── Motor/
│   │   ├── MotorControl.h     # TB6612FNG motor driver control
│   │   └── README.md
│   ├── PID/
│   │   ├── PID.h              # Core PID algorithm
│   │   └── README.md
│   └── PIDController/
│       ├── PIDController.h    # PID manager with BLE integration
│       └── README.md
├── src/
│   └── main.cpp               # Main control loop (50Hz)
├── docs/
│   ├── USAGE.md               # Detailed usage guide
│   ├── TROUBLESHOOTING.md     # Common issues & solutions
│   └── images/                # Demo GIFs and photos
├── platformio.ini             # PlatformIO configuration
├── LICENSE                    # MIT License
└── README.md                  # This file
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     BLE Control App (Flutter)               │
│              Real-time Parameter Tuning & Monitoring        │
└───────────────────┬─────────────────────────────────────────┘
                    │ Bluetooth Low Energy
                    ▼
┌────────────────────────────────────────────────────────────┐
│                        ESP32-WROOM                         │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  BLEConfig.h - Wireless Parameter Updates           │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │                                  │
│  ┌──────────────────────▼──────────────────────────────┐   │
│  │  PIDController.h - Dynamic PID Management           │   │
│  │  ├─ Fetches BLE parameters                          │   │
│  │  ├─ Manages PID lifecycle                           │   │
│  │  └─ Calls PID.h for calculations                    │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │                                  │
│  ┌──────────────────────▼──────────────────────────────┐   │
│  │  PID.h - Control Algorithm                          │   │
│  │  P + I + D → Correction Value                       │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │                                  │
│  ┌──────────────────────▼──────────────────────────────┐   │
│  │  MotorControl.h - Differential Steering             │   │
│  │  Left Motor = Base - Correction                     │   │
│  │  Right Motor = Base + Correction                    │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │                                  │
│  ┌──────────────────────▲──────────────────────────────┐   │
│  │  Linepos.h - Weighted Average Position              │   │
│  │  Σ(value × weight) / Σ(values)                      │   │
│  └──────────────────────▲──────────────────────────────┘   │
│                         │                                  │
│  ┌──────────────────────┴──────────────────────────────┐   │
│  │  Calibration.h - Sensor Normalization               │   │
│  │  Min/Max Tracking → Threshold Calculation           │   │
│  └──────────────────────▲──────────────────────────────┘   │
│                         │                                  │
│  ┌──────────────────────┴──────────────────────────────┐   │
│  │  IRSensor.h - 8-Channel ADC Interface               │   │
│  │  Reads GPIO pins → Analog values (0-4095)           │   │
│  └─────────────────────────────────────────────────────┘   │
└───────────────────┬────────────────────────────────────────┘
                    │
        ┌───────────┴───────────┐
        ▼                       ▼
┌──────────────┐        ┌──────────────┐
│ IR Sensors   │        │ DC Motors    │
│ (8 channels) │        │ (Left+Right) │
└──────────────┘        └──────────────┘
```

---

## Key Features Explained

### 1. Modular Architecture
Each component is **self-contained** with its own header file and documentation:
- Easy to understand (one file = one responsibility)
- Easy to modify (change motor driver? Update MotorControl.h only)
- Easy to debug (test modules individually)
- Easy to reuse (port to different projects)

### 2. Wireless PID Tuning
**Traditional approach:**
1. Change PID values in code
2. Upload to ESP32 (30+ seconds)
3. Test
4. Repeat 20+ times 

**Our approach:**
1. Adjust sliders in phone app
2. Changes apply instantly 
3. Test
4. Find optimal values in minutes! 

### 3. Persistent Calibration
- Calibrate once per environment
- Saves to ESP32 flash memory
- Auto-loads on every boot
- No need to recalibrate daily

### 4. Weighted Average Algorithm
```
Position = Σ(sensorValue × weight) / Σ(values)
Weights: [-4, -3, -2, -1, 1, 2, 3, 4]
```
- Accurate position even with multiple sensors active
- Smooth, continuous position values (not just left/center/right)
- Works with wide lines, narrow lines, curves

---

##  Performance Metrics

| Metric | Value |
|--------|-------|
| **Control Loop Frequency** | 50Hz (20ms per cycle) |
| **Sensor Resolution** | 12-bit ADC (4096 levels) |
| **Position Update Rate** | 50 updates/second |
| **BLE Latency** | <50ms parameter update |
| **Calibration Time** | 5 seconds (configurable) |
| **Max Speed** | 2.5 m/s (depends on motors/track) |
| **Line Width Tolerance** | 10-30mm (optimal: 19mm) |

---

## Testing & Validation

### Unit Tests
Each module includes test code in its README:
- `IRSensor.h` → Test raw sensor readings
- `Calibration.h` → Test min/max tracking
- `Linepos.h` → Test position calculation
- `PID.h` → Test algorithm output
- `MotorControl.h` → Test individual motor control

### Integration Testing
Run the full system and verify:
- [ ] Sensors read correctly
- [ ] Calibration saves/loads
- [ ] PID calculates corrections
- [ ] Motors respond to corrections
- [ ] BLE updates apply in real-time

---

## Documentation

### Quick Links
- **[USAGE.md](docs/USAGE.md)** - Step-by-step usage guide
- **[TUNING.md](docs/TUNING.md)** - PID tuning tutorial for beginners
- **[TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)** - Common problems & solutions
- **[API Reference](docs/API.md)** - Function documentation

### Module Documentation
Each header file has its own detailed README:
- [BLEConfig.h Documentation](include/BLE/README.md)
- [Calibration.h Documentation](include/Calibration/README.md)
- [IRSensor.h Documentation](include/IR/README.md)
- [Linepos.h Documentation](include/LinePos/README.md)
- [MotorControl.h Documentation](include/Motor/README.md)
- [PID.h Documentation](include/PID/README.md)
- [PIDController.h Documentation](include/PIDController/README.md)

---

## Roadmap

### Completed
- [x] Core line following algorithm
- [x] PID control with anti-windup
- [x] BLE parameter updates
- [x] Sensor calibration & persistence
- [x] Modular architecture
- [x] Comprehensive documentation

### In Progress
- [ ] Flutter BLE control app
- [ ] Web-based BLE interface
- [ ] Data logging to SD card

### Future Plans
- [ ] Speed optimization mode
- [ ] Multi-robot coordination
- [ ] Machine learning path prediction


---

## License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

##  Contact & Support

- **Author:** Antony Austin
- **Email:** austinantony06@gmail.com
- **GitHub:** [@austin207](https://github.com/austin207)

---

**Made with ❤️ and lots of ☕**

If this project helped you, consider giving it a ⭐!

[⬆ Back to Top](#esp32-line-follower-robot)

</div>
