# USAGE.md - Complete User Guide

A step-by-step guide for using the ESP32 Line Follower Robot with BLE control and auto-calibration.

---

## Table of Contents

1. [First-Time Setup](#first-time-setup)
2. [Hardware Assembly](#hardware-assembly)
3. [Software Installation](#software-installation)
4. [Initial Configuration](#initial-configuration)
5. [Sensor Calibration](#sensor-calibration)
6. [BLE Connection & Control](#ble-connection--control)
7. [PID Tuning Guide](#pid-tuning-guide)
8. [Operating the Robot](#operating-the-robot)
9. [Troubleshooting](#troubleshooting)
10. [Advanced Usage](#advanced-usage)

---

## First-Time Setup

### Prerequisites

**Hardware:**
- ESP32-WROOM development board
- QTR-8A/QTR-8RC IR sensor array
- TB6612FNG motor driver
- 2× DC motors with wheels
- Battery (11.1V LiPo or 6×AA)
- Robot chassis
- Jumper wires

**Software:**
- PlatformIO IDE (VS Code extension) or Arduino IDE
- USB cable for ESP32 programming
- Serial terminal (built into VS Code/Arduino IDE)

**Mobile Device:**
- Smartphone with Bluetooth 4.0+ (BLE)
- BLE terminal app (nRF Connect, LightBlue, Serial Bluetooth Terminal)

---

## Hardware Assembly

### Step 1: Wire the IR Sensor Array

Connect the 8-channel IR sensor to ESP32:

| IR Sensor Pin | ESP32 GPIO | Wire Color (typical) |
|---------------|------------|---------------------|
| A1 | GPIO13 | White |
| A2 | GPIO12 | Gray |
| A3 | GPIO14 | Purple |
| A4 | GPIO27 | Blue |
| A5 | GPIO26 | Green |
| A6 | GPIO25 | Yellow |
| A7 | GPIO33 | Orange |
| A8 | GPIO32 | Red |
| VCC | 3.3V or 5V | Red |
| GND | GND | Black |

**Mounting Position:**
- Mount sensor array 3-5mm above the ground
- Center it under the robot chassis
- Sensors should face downward perpendicular to surface

---

### Step 2: Wire the Motor Driver (TB6612FNG)

**Control Signals (ESP32 → TB6612FNG):**

| TB6612FNG Pin | ESP32 GPIO | Function |
|---------------|------------|----------|
| PWMA | GPIO5 | Left motor speed |
| AIN1 | GPIO19 | Left motor direction 1 |
| AIN2 | GPIO18 | Left motor direction 2 |
| PWMB | GPIO23 | Right motor speed |
| BIN1 | GPIO21 | Right motor direction 1 |
| BIN2 | GPIO22 | Right motor direction 2 |
| STBY | GPIO17 (optional) | Standby (tie to VCC if unused) |

**Power Connections:**

| TB6612FNG Pin | Connection | Notes |
|---------------|------------|-------|
| VCC | ESP32 3.3V | Logic power |
| GND | Common GND | **CRITICAL: All grounds must connect** |
| VM | Battery + | Motor power (6-12V) |
| AO1 | Left Motor + | Left motor terminal 1 |
| AO2 | Left Motor - | Left motor terminal 2 |
| BO1 | Right Motor + | Right motor terminal 1 |
| BO2 | Right Motor - | Right motor terminal 2 |

> ⚠️ **IMPORTANT:** Connect all grounds together (ESP32 GND, TB6612FNG GND, Battery GND)

---

### Step 3: Power Supply

**Option 1: Separate Power (Recommended)**
```
Battery → TB6612FNG VM
ESP32 → USB power (during testing) OR Battery via regulator (autonomous)
```

**Option 2: Shared Power**
```
Battery + → ESP32 VIN (if 7-12V) AND TB6612FNG VM
Battery - → Common GND
```

**Power On Sequence:**
1. Connect all wires
2. Double-check polarity (+ and -)
3. Turn on battery switch
4. ESP32 should power up (onboard LED blinks)

---

## Software Installation

### Method 1: PlatformIO (Recommended)

**Step 1: Install VS Code**
```
# Download from https://code.visualstudio.com/
```

**Step 2: Install PlatformIO Extension**
1. Open VS Code
2. Go to Extensions (Ctrl+Shift+X)
3. Search "PlatformIO IDE"
4. Click Install
5. Restart VS Code

**Step 3: Clone & Open Project**
```
git clone https://github.com/austin207/Line-Follower.git
cd Line-Follower
code .
```

**Step 4: Build & Upload**
1. Connect ESP32 via USB
2. Click **PlatformIO: Upload** (→ icon) in bottom toolbar
3. Wait for compilation and upload (~1 minute)
4. Open Serial Monitor (plug icon, 115200 baud)

---

### Method 2: Arduino IDE

**Step 1: Install ESP32 Board Support**
1. Open Arduino IDE
2. File → Preferences
3. Additional Board URLs: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
4. Tools → Board → Boards Manager
5. Search "ESP32" → Install

**Step 2: Copy Files**
1. Download repository ZIP
2. Extract to Arduino sketchbook folder
3. Rename folder to `Line_Follower` (no hyphens)

**Step 3: Configure**
1. Tools → Board → ESP32 Dev Module
2. Tools → Upload Speed → 115200
3. Tools → Port → Select your ESP32 port

**Step 4: Upload**
1. Click Upload button (→)
2. Wait for compilation
3. Open Serial Monitor (115200 baud)

---

## Initial Configuration

### First Boot Checklist

**Step 1: Verify Serial Output**

After uploading, open Serial Monitor (115200 baud). You should see:

```
=== ESP32 Line Follower Initialized ===
IR Sensor Array Initialized (Analog Mode)
Motor control initialized!
BLE Server started
Waiting for BLE client connection...
```

If you see this → Software installed correctly

---

**Step 2: Check Default Settings**

Default parameters (defined in `BLEConfig.h`):

| Parameter | Default Value | Description |
|-----------|--------------|-------------|
| baseSpeed | 150 | Cruising speed (0-255) |
| Kp | 8.0 | Proportional gain |
| Ki | 0.0 | Integral gain (disabled) |
| Kd | 80.0 | Derivative gain |
| MinOutput | -255 | Minimum correction |
| MaxOutput | +255 | Maximum correction |

These work well for most setups. You can adjust via BLE later.

---

**Step 3: Test Sensors (Optional)**

Uncomment `printSensorValues()` in `main.cpp`:

```
void loop() {
    readIRSensors();
    printSensorValues();  // Uncomment this line
    delay(100);
}
```

**Expected Output:**
```
Sensors: 456  478  501  523  465  485  490  495
```

**Test:**
- Move hand over sensors → values should change
- White surface → LOW values (200-700)
- Black surface → HIGH values (2500-3800)

If sensors respond → Hardware wired correctly

---

## Sensor Calibration

Calibration adapts sensors to your specific track and lighting conditions.

### When to Calibrate

**Required:**
- First time using the robot
- Changed track surface (matte paper → glossy paper)
- Different lighting (indoor → outdoor)
- Moved to new location

**Optional:**
- Battery voltage drops significantly
- Robot behavior changes unexpectedly

---

### Calibration Methods

#### Method 1: BLE Trigger (Easiest)

**Step 1: Connect via BLE**
1. Open BLE app (nRF Connect or similar)
2. Scan for "ESP32 Line Follower"
3. Connect to device

**Step 2: Trigger Calibration**
1. Find "CALIBRATE" characteristic (UUID: `beb5483e-...`)
2. Write value: `1` (or any non-zero value)
3. Robot starts calibration

**Step 3: Move Robot**
```
=== CALIBRATION STARTED ===
Move sensor array over BLACK and WHITE surfaces...
..............................
=== CALIBRATION COMPLETE ===
```

- **Slowly** move robot over BLACK line
- **Slowly** move robot over WHITE surface
- Cover all 8 sensors (both colors)
- Duration: 5 seconds (shows progress dots)

**Step 4: Verify**

Serial Monitor shows calibration data:
```
--- Calibration Data ---
Min Values:  450   480   470   460   455   465   475   490
Max Values:  2600  2550  2580  2590  2610  2570  2560  2540
Threshold:   1525  1515  1525  1525  1532  1517  1517  1515

Calibration saved to flash memory!
```

✅ **Good Calibration:**
- Min values: 200-800 (white surface)
- Max values: 2200-3800 (black line)
- Difference (Max - Min): >1500

❌ **Bad Calibration:**
- Min ≈ Max (didn't see both colors)
- Difference < 500 (poor contrast)

---

#### Method 2: Code Trigger

**In `main.cpp` setup():**

```
void setup() {
    Serial.begin(115200);
    initIRSensors();
    initMotors();
    initBLE();
    initCalibration();
    
    // Add this:
    delay(3000);  // 3 second delay before calibration
    calibrate(5000);  // 5 second calibration
}
```

Upload code, then immediately move robot over black/white surfaces.

---

#### Method 3: Button Trigger (Hardware)

**Add calibration button:**

```
#define CALIBRATE_BUTTON 15  // GPIO15

void setup() {
    pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
    // ... other setup
}

void loop() {
    if (digitalRead(CALIBRATE_BUTTON) == LOW) {
        calibrate(5000);
        delay(1000);  // Debounce
    }
    // ... rest of loop
}
```

Press button to start calibration.

---

### Calibration Tips

**✅ DO:**
- Move slowly and smoothly
- Ensure all sensors see both black and white
- Calibrate in actual competition environment
- Recalibrate if track/lighting changes

**❌ DON'T:**
- Move too fast (sensors miss surfaces)
- Calibrate in different lighting than competition
- Skip calibration (robot won't work well)
- Cover sensors with hands (blocks light)

---

## BLE Connection & Control

### Connecting via Smartphone

**Recommended Apps:**
- **Android:** nRF Connect, Serial Bluetooth Terminal
- **iOS:** LightBlue, BLE Scanner

---

### Connection Steps (nRF Connect Example)

**Step 1: Power On Robot**
- ESP32 starts BLE server automatically
- Serial Monitor shows: `BLE Server started`

**Step 2: Open nRF Connect**
1. Launch app
2. Tap "SCAN" button
3. Look for "ESP32 Line Follower"

**Step 3: Connect**
1. Tap "CONNECT" next to device name
2. Wait for connection (LED may blink)
3. Serial Monitor shows: `BLE Client connected`

**Step 4: Discover Services**
1. App shows "Unknown Service" (UUID: 4fafc201-...)
2. Tap to expand service
3. See all characteristics (SPEED, Kp, Ki, Kd, etc.)

---

### BLE Characteristics Reference

| Characteristic | UUID (last 4 digits) | Type | Range | Description |
|----------------|---------------------|------|-------|-------------|
| SPEED | 483e-01 | Write/Read | 0-255 | Base speed |
| Kp | 483e-02 | Write/Read | 0-50.0 | Proportional gain |
| Ki | 483e-03 | Write/Read | 0-10.0 | Integral gain |
| Kd | 483e-04 | Write/Read | 0-300.0 | Derivative gain |
| MIN_OUTPUT | 483e-05 | Write/Read | -255 to 0 | Min correction |
| MAX_OUTPUT | 483e-06 | Write/Read | 0 to 255 | Max correction |
| CALIBRATE | 483e-07 | Write | 0/1 | Trigger calibration |
| POSITION | 483e-08 | Read/Notify | -999 to +100 | Line position |
| CORRECTION | 483e-09 | Read/Notify | -255 to +255 | PID output |
| EMERGENCY_STOP | 483e-10 | Write | 0/1 | Stop motors |

---

### Adjusting Parameters via BLE

#### Example: Change Base Speed

**Step 1: Find SPEED Characteristic**
- UUID ends in `483e-01`
- Current value shown (e.g., `150`)

**Step 2: Write New Value**
1. Tap "↑" (Write) button
2. Select data type: **UINT8** or **UINT16**
3. Enter new value: `100` (slower) or `200` (faster)
4. Tap "SEND"

**Step 3: Verify**
- Serial Monitor shows: `Base speed set to: 100`
- Robot slows down immediately

---

#### Example: Tune PID (Increase Kp)

**Step 1: Find Kp Characteristic**
- UUID ends in `483e-02`
- Current value: `8.00`

**Step 2: Write New Value**
1. Tap "↑" (Write) button
2. Select: **FLOAT32**
3. Enter: `12.0`
4. Tap "SEND"

**Step 3: Verify**
- Serial Monitor shows: `Kp updated to: 12.00`
- Robot responds more aggressively

---

#### Example: Monitor Line Position (Real-Time)

**Step 1: Find POSITION Characteristic**
- UUID ends in `483e-08`

**Step 2: Enable Notifications**
1. Tap "↓" (Subscribe/Notify) button
2. Toggle notifications ON

**Step 3: Watch Live Data**
- App shows continuous updates:
  ```
  Value: -30  (line is left)
  Value: -15
  Value: 0    (centered!)
  Value: 20   (line is right)
  ```

---

## PID Tuning Guide

### Understanding PID Parameters

**Kp (Proportional):**
- **Effect:** Immediate response to error
- **Too Low:** Slow response, large error
- **Too High:** Oscillation, overshoot
- **Start:** 5-10

**Ki (Integral):**
- **Effect:** Eliminates steady-state error
- **Too Low:** Persistent drift
- **Too High:** Windup, overshoot
- **Start:** 0 (disable initially)

**Kd (Derivative):**
- **Effect:** Dampens oscillation, smooths response
- **Too Low:** Overshoot, oscillation
- **Too High:** Jittery, noise amplification
- **Start:** 10× Kp (e.g., Kd = 80 if Kp = 8)

---

### Step-by-Step Tuning Process

#### Phase 1: Tune Kp Only

**Step 1: Set Ki=0, Kd=0**
```
Via BLE:
Ki → 0.0
Kd → 0.0
Kp → 5.0
```

**Step 2: Increase Kp Gradually**
1. Place robot on track
2. Increase Kp by 1-2
3. Observe behavior:
   - **Kp = 5:** Slow, large error ❌
   - **Kp = 8:** Better, slight oscillation ⚠️
   - **Kp = 12:** Constant oscillation ❌

**Step 3: Find Sweet Spot**
- Stop when oscillation starts
- Back off 20-30%
- **Example:** Oscillates at Kp=12 → Use Kp=8

✅ **Good Kp:** Robot follows line with slight wobble

---

#### Phase 2: Add Derivative (Kd)

**Step 1: Set Kd = 10× Kp**
```
If Kp = 8:
Kd → 80.0
```

**Step 2: Test**
- Robot should follow smoother
- Less oscillation
- No overshoot on curves

**Step 3: Adjust Kd**
- **Still oscillating?** Increase Kd (+20)
- **Too slow/sluggish?** Decrease Kd (-20)
- **Jittery?** Decrease Kd (-10)

✅ **Good Kd:** Smooth following, minimal overshoot

---

#### Phase 3: Add Integral (Ki) - Optional

**When Needed:**
- Robot consistently drifts to one side
- Persistent offset error
- One motor weaker than other

**Step 1: Start Small**
```
Ki → 0.01
```

**Step 2: Increase Slowly**
- Test on straight section
- Increase by 0.01-0.05 increments
- **Stop when drift eliminated**

**Step 3: Watch for Windup**
- Robot overcorrects after curves?
- Reduce Ki or leave at 0

✅ **Good Ki:** Eliminates drift without overshoot

---

### Typical Values

| Robot Speed | Kp | Ki | Kd | Notes |
|-------------|----|----|-------|-------|
| Slow (100) | 5.0 | 0.0 | 50.0 | Learning/testing |
| Medium (150) | 8.0 | 0.0 | 80.0 | Default (balanced) |
| Fast (200) | 12.0 | 0.1 | 120.0 | Competition mode |
| Very Fast (255) | 15.0 | 0.2 | 150.0 | Speed runs (risky) |

---

## Operating the Robot

### Pre-Flight Checklist

**Before Each Run:**
- [ ] Battery charged (>7V for 7.4V LiPo)
- [ ] All wires secure (no loose connections)
- [ ] Sensors clean (no dust/debris)
- [ ] Wheels spin freely
- [ ] Track clean (no obstructions)
- [ ] Calibration loaded (check Serial Monitor)

---

### Starting a Run

**Step 1: Position Robot**
- Place on track with line **centered** under sensor array
- Robot should be straight (not angled)
- All sensors should be over track (not hanging off edge)

**Step 2: Verify Settings**
- Check BLE app or Serial Monitor:
  ```
  Base Speed: 150
  Kp: 8.00, Ki: 0.00, Kd: 80.00
  ```

**Step 3: Start**

**Method A: Auto-Start (Default)**
- Robot starts following immediately after power-on
- No action needed

**Method B: BLE Start (if emergency stop was used)**
1. Connect via BLE
2. EMERGENCY_STOP → Write `0` (resume)

**Step 4: Monitor**
- Watch robot follow the line
- Check Serial Monitor (if connected):
  ```
  Pos: -20 | Correction: 85
  Pos: -10 | Correction: 45
  Pos: 0 | Correction: 2
  ```

---

### During Operation

**Normal Behavior:**
- ✅ Smooth following with slight wobble
- ✅ Corrects quickly on curves
- ✅ Stays centered on straights

**Warning Signs:**
- ⚠️ Wild oscillation → Reduce Kp or increase Kd
- ⚠️ Drifts to one side → Add small Ki or check motor balance
- ⚠️ Slow response → Increase Kp
- ⚠️ Overshoots curves → Increase Kd

**Emergency Situations:**
- 🚨 Line lost → Robot stops automatically
- 🚨 Veers off track → Press emergency stop button or BLE
- 🚨 Erratic behavior → Power off immediately

---

### Stopping the Robot

**Method 1: Emergency Stop (BLE)**
1. Open BLE app
2. EMERGENCY_STOP characteristic → Write `1`
3. Motors stop immediately

**Method 2: Remove from Track**
- Pick up robot
- Line position becomes -999
- Motors stop automatically

**Method 3: Power Off**
- Flip battery switch OFF

---

## Troubleshooting

### Problem: Robot Doesn't Move

**Check:**
- [ ] Battery connected and charged?
- [ ] Motor driver powered (check VM pin)?
- [ ] Common ground connected?
- [ ] STBY pin HIGH (if used)?

**Test:**
```
void loop() {
    setMotorA(100);   // Left motor forward
    setMotorB(100);   // Right motor forward
    delay(2000);
    stopMotors();
    delay(2000);
}
```

If motors spin → Wiring OK, issue is in control logic

---

### Problem: Robot Goes Straight Off Track

**Causes:**
- Sensors not calibrated
- Threshold too high/low
- PID gains wrong

**Solution:**
1. **Recalibrate sensors**
2. **Check sensor readings:**
   ```
   White: 400-800 (LOW)
   Black: 2500-3800 (HIGH)
   ```
3. **Verify position calculation:**
   ```
   Line left → Position negative
   Line right → Position positive
   ```

---

### Problem: Oscillates Wildly

**Cause:** Kp too high

**Solution:**
1. Via BLE: Reduce Kp by 30%
   ```
   Kp: 12.0 → 8.0
   ```
2. Increase Kd for damping:
   ```
   Kd: 80.0 → 120.0
   ```

---

### Problem: BLE Won't Connect

**Check:**
- [ ] ESP32 powered on?
- [ ] Serial Monitor shows "BLE Server started"?
- [ ] Bluetooth enabled on phone?
- [ ] Not connected to another device?

**Solution:**
1. Restart ESP32
2. Restart BLE app
3. Toggle phone Bluetooth OFF/ON
4. Check device name: "ESP32 Line Follower"

---

### Problem: Calibration Doesn't Save

**Check Serial Monitor:**
```
Calibration saved to flash memory!  ← Should see this
```

**If not saving:**
```
// In Calibration.h, check:
preferences.begin("calibration", false);  // false = read/write mode
```

**Clear and retry:**
1. BLE → CALIBRATE → Write `2` (clears calibration)
2. BLE → CALIBRATE → Write `1` (recalibrate)

---

## Advanced Usage

### Custom Speed Profiles

**Adaptive Speed (Slow on Curves):**

```
int baseSpeed = 150;

void loop() {
    int position = readLinePosition();
    
    // Slow down if far from center
    if (abs(position) > 50) {
        baseSpeed = 100;  // Slow for sharp turns
    } else {
        baseSpeed = 180;  // Fast on straights
    }
    
    int correction = pidController.update(position);
    applyPIDCorrection(correction);
}
```

---

### Data Logging

**Log to Serial (CSV format):**

```
void loop() {
    readIRSensors();
    int position = readLinePosition();
    int correction = pidController.update(position);
    
    // CSV: timestamp, position, correction, leftSpeed, rightSpeed
    Serial.print(millis());
    Serial.print(",");
    Serial.print(position);
    Serial.print(",");
    Serial.print(correction);
    Serial.print(",");
    int left, right;
    getMotorSpeeds(&left, &right);
    Serial.print(left);
    Serial.print(",");
    Serial.println(right);
    
    applyPIDCorrection(correction);
}
```

**Capture to file:**
```
pio device monitor > logfile.csv
```

---

### Multi-Track Support

**Save multiple calibrations:**

```
// Modify Calibration.h to support profiles
void saveCalibration(String profileName) {
    preferences.begin(profileName.c_str(), false);
    // ... save calibration
}

void loadCalibration(String profileName) {
    preferences.begin(profileName.c_str(), true);
    // ... load calibration
}
```

**Usage:**
```
saveCalibration("track_indoor");   // Indoor track calibration
saveCalibration("track_outdoor");  // Outdoor track calibration

// Switch profiles via BLE or button
loadCalibration("track_outdoor");
```

---

### Performance Optimization

**Increase Control Loop Frequency:**

```
void loop() {
    unsigned long loopStart = micros();
    
    readIRSensors();
    int position = readLinePosition();
    int correction = pidController.update(position);
    applyPIDCorrection(correction);
    
    // Maintain 100Hz (10ms loop)
    while (micros() - loopStart < 10000);
}
```

---

## Quick Reference

### Serial Commands (via Serial Monitor)

| Command | Action |
|---------|--------|
| `c` | Start calibration |
| `p` | Print calibration data |
| `r` | Reset PID controller |
| `d` | Toggle debug output |

*(Implement in main.cpp if needed)*

---

### BLE Quick Actions

| Task | Characteristic | Value |
|------|---------------|-------|
| Speed up | SPEED | Current + 20 |
| Speed down | SPEED | Current - 20 |
| More aggressive | Kp | Current × 1.2 |
| Smoother | Kd | Current × 1.2 |
| Calibrate | CALIBRATE | 1 |
| Emergency stop | EMERGENCY_STOP | 1 |
| Resume | EMERGENCY_STOP | 0 |

---

### Pin Reference (Quick Lookup)

**IR Sensors:**
```
13, 12, 14, 27, 26, 25, 33, 32
```

**Motors:**
```
PWM: 5, 23
DIR: 19, 18, 21, 22
```

---

## Tips for Competition

1. **Practice on actual competition track** (lighting/surface matters!)
2. **Calibrate multiple times** (use best result)
3. **Start conservative** (slower speed, lower Kp)
4. **Gradually increase speed** after stable following
5. **Have backup battery** (voltage drop affects performance)
6. **Clean sensors** before each run (dust reduces contrast)
7. **Check wire connections** (vibration can loosen wires)
8. **Log successful PID values** (write them down!)

---

## Support

**Issues?**
- Check [Troubleshooting](#troubleshooting) section (Coming Soon)
- Review module READMEs in `/include/` folders
- Open GitHub issue: [austin207/Line-Follower/issues](https://github.com/austin207/Line-Follower/issues)

**Need Help?**
- Post in GitHub Discussions
- Check Serial Monitor output for error messages
- Verify hardware connections against pin tables

---

<div align="center">

**Happy Line Following! 🤖**

Made with ❤️ by austin207

[⬆ Back to Top](#usagemd---complete-user-guide)

</div>