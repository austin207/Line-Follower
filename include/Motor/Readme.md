# MotorControl.h - TB6612FNG Motor Driver Interface

## What Does This File Do?

This file controls your robot's **two DC motors** using the **TB6612FNG dual motor driver**. It converts PID correction values into actual motor movements, making your robot steer left or right to follow the line.

Think of it as the **steering wheel and gas pedal** of your robot - it translates decisions into motion!

---

## Why Do We Need This?

### **The Problem:**

The ESP32 can't directly drive motors because:
- Motors need high current (1-2A each)
- ESP32 GPIO pins provide only 40mA
- Direct connection would **burn the GPIO pins**

### **The Solution:**

The **TB6612FNG motor driver** acts as a powerful switch:
- ESP32 sends low-power signals (direction + PWM)
- TB6612FNG controls high-power motor current
- Motors get the power they need safely

---

## Hardware Setup

### **TB6612FNG Motor Driver**

**Specifications:**
- Voltage: 4.5V - 13.5V
- Current: 1.2A continuous, 3.2A peak per motor
- Logic voltage: 3.3V or 5V compatible
- Dual H-Bridge (controls 2 motors independently)

### **Pin Connections**

**ESP32 → TB6612FNG:**

| ESP32 Pin | TB6612 Pin | Function | Description |
|-----------|------------|----------|-------------|
| GPIO5 | PWMA | Speed A | Motor A speed (0-255 PWM) |
| GPIO19 | AIN1 | Direction A1 | Motor A direction bit 1 |
| GPIO18 | AIN2 | Direction A2 | Motor A direction bit 2 |
| GPIO21 | BIN1 | Direction B1 | Motor B direction bit 1 |
| GPIO22 | BIN2 | Direction B2 | Motor B direction bit 2 |
| GPIO23 | PWMB | Speed B | Motor B speed (0-255 PWM) |
| GPIO17 | STBY | Standby | Enable/disable driver (HIGH=on) |

**TB6612FNG → Motors & Power:**

| TB6612 Pin | Connection | Description |
|------------|------------|-------------|
| VM | Battery + | Motor power supply (6-12V) |
| VCC | 3.3V/5V | Logic power supply |
| GND | Ground | Common ground (ESP32 + Battery) |
| AO1 | Motor A + | Motor A terminal 1 |
| AO2 | Motor A - | Motor A terminal 2 |
| BO1 | Motor B + | Motor B terminal 1 |
| BO2 | Motor B - | Motor B terminal 2 |

### **Wiring Diagram**

```
Battery (6-12V)
├── VM (TB6612)
└── GND (TB6612)

ESP32                TB6612FNG              Motors
┌──────────┐        ┌──────────┐         ┌────────┐
│ GPIO5    │───────→│ PWMA  A1 │────────→│        │
│ GPIO19   │───────→│ AIN1     │         │ Motor A│
│ GPIO18   │───────→│ AIN2  A2 │────────→│ (Left) │
│ GPIO21   │───────→│ BIN1     │         └────────┘
│ GPIO22   │───────→│ BIN2     │         ┌────────┐
│ GPIO23   │───────→│ PWMB  B1 │────────→│        │
│ 3.3V     │───────→│ VCC      │         │ Motor B│
│ GND      │───────→│ GND   B2 │────────→│ (Right)│
└──────────┘        └──────────┘         └────────┘
```

** IMPORTANT: Common Ground**
- Connect ESP32 GND and Battery GND together
- Without common ground, motor driver won't work
- Use thick wires for motor power (16-20 AWG)

---

##  Code Breakdown

### **1. Pin Definitions - Hardware Mapping**

```
#define PWMA_PIN    5   // Motor A speed control
#define AIN2_PIN    18  // Motor A direction control 2
#define AIN1_PIN    19  // Motor A direction control 1
#define BIN1_PIN    21  // Motor B direction control 1
#define BIN2_PIN    22  // Motor B direction control 2
#define PWMB_PIN    23  // Motor B speed control
```

**What These Do:**

**PWM Pins (5, 23):**
- Generate variable-speed signals (0-255)
- 0 = stopped, 255 = full speed
- Frequency: 20kHz (silent operation)

**Direction Pins (18, 19, 21, 22):**
- Control motor rotation direction
- Two pins per motor (4 total)
- See truth table below

**How to Change Pins:**
```
#define PWMA_PIN    16   // Change to GPIO16
#define AIN1_PIN    17   // Change to GPIO17
// Update all 6 pins as needed
```

---

### **2. PWM Configuration - Speed Control**

```
#define PWM_FREQ      20000  // 20kHz - above human hearing range
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_A  0     // LEDC channel for Motor A
#define PWM_CHANNEL_B  1     // LEDC channel for Motor B
```

**Understanding PWM:**

**PWM (Pulse Width Modulation):**
- Rapidly switches power ON/OFF
- Duty cycle = percentage of time ON
- Motor sees this as variable speed

**Visual Example:**
```
Speed 64 (25% duty):  ▄▁▁▁▄▁▁▁▄▁▁▁▄▁▁▁  → Slow
Speed 128 (50% duty): ▄▄▁▁▄▄▁▁▄▄▁▁▄▄▁▁  → Medium
Speed 192 (75% duty): ▄▄▄▁▄▄▄▁▄▄▄▁▄▄▄▁  → Fast
Speed 255 (100% duty):▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄  → Max
```

**Frequency (20kHz):**
- Above human hearing (we hear 20Hz-20kHz)
- Reduces motor whine/buzzing
- Lower frequency (1-5kHz) = audible noise
- Higher frequency (50kHz+) = less efficient

**Resolution (8-bit):**
- 2^8 = 256 speed levels (0-255)
- Enough precision for smooth control
- Could use 10-bit (0-1023) for finer control

**LEDC Channels:**
- ESP32 has 16 PWM channels (0-15)
- We use channels 0 and 1 (one per motor)
- Each channel is independent

**Customization:**
```
// Quieter operation (higher frequency)
#define PWM_FREQ 30000  // 30kHz

// More speed levels
#define PWM_RESOLUTION 10  // 10-bit (0-1023)

// Different channels
#define PWM_CHANNEL_A 4
#define PWM_CHANNEL_B 5
```

---

### **3. Global Variables - State Tracking**

```
int baseSpeed = 150;         // Default base speed (0-255)
int motorASpeed = 0;         // Current speed of Motor A
int motorBSpeed = 0;         // Current speed of Motor B
```

**baseSpeed:**
- The "cruising speed" of your robot
- Both motors run at this speed when going straight
- Adjustable via BLE (50-255 typical range)

**motorASpeed / motorBSpeed:**
- Current actual speed of each motor
- Includes PID corrections
- Negative values = reverse

**Example:**
```
baseSpeed = 150;           // Robot's normal speed
correction = +30;          // PID says turn right

leftMotorSpeed = 150 - 30 = 120   (slower)
rightMotorSpeed = 150 + 30 = 180  (faster)
→ Robot turns RIGHT ✓
```

---

### **4. initMotors() - Setup Function**

```
void initMotors() {
    // Configure direction control pins as outputs
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);
    
    // Configure PWM channels
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    
    // Attach PWM channels to pins
    ledcAttachPin(PWMA_PIN, PWM_CHANNEL_A);
    ledcAttachPin(PWMB_PIN, PWM_CHANNEL_B);
    
    // Stop motors initially
    stopMotors();
}
```

**Step-by-Step:**

**Step 1: Configure Direction Pins**
```
pinMode(AIN1_PIN, OUTPUT);
```
- Sets GPIO19 as output (can send HIGH/LOW signals)
- Same for all 4 direction pins

**Step 2: Setup PWM Channels**
```
ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
```
- Configures channel 0 for 20kHz, 8-bit PWM
- ESP32's LEDC (LED Controller) hardware does the work
- Same for channel 1 (Motor B)

**Step 3: Attach Channels to Pins**
```
ledcAttachPin(PWMA_PIN, PWM_CHANNEL_A);
```
- Links channel 0 to GPIO5
- Now `ledcWrite(0, speed)` controls GPIO5
- Same for channel 1 → GPIO23

**Step 4: Safety - Stop Motors**
```
stopMotors();
```
- Ensures motors are OFF during startup
- Prevents unexpected movement

**When to Call:**
```
void setup() {
  Serial.begin(115200);
  initMotors();  // Call once during setup
}
```

---

### **5. Motor Direction Control - Truth Table**

**TB6612FNG Logic:**

| IN1 | IN2 | Motor State |
|-----|-----|-------------|
| LOW | LOW | Brake (short brake) |
| HIGH | LOW | Forward (clockwise) |
| LOW | HIGH | Reverse (counter-clockwise) |
| HIGH | HIGH | Brake (short brake) |

**In Code:**

**Forward:**
```
digitalWrite(AIN1_PIN, HIGH);  // IN1 = HIGH
digitalWrite(AIN2_PIN, LOW);   // IN2 = LOW
→ Motor spins forward
```

**Reverse:**
```
digitalWrite(AIN1_PIN, LOW);   // IN1 = LOW
digitalWrite(AIN2_PIN, HIGH);  // IN2 = HIGH
→ Motor spins backward
```

**Brake:**
```
digitalWrite(AIN1_PIN, LOW);   // IN1 = LOW
digitalWrite(AIN2_PIN, LOW);   // IN2 = LOW
→ Motor stops (actively braked)
```

**Why Two Pins?**
- H-Bridge needs to know which way to drive current
- 2 pins = 4 states (forward, reverse, brake, brake)
- More control than simple ON/OFF

---

### **6. setMotorA() / setMotorB() - Individual Motor Control**

```
void setMotorA(int speed) {
    motorASpeed = constrain(speed, -255, 255);
    
    if (motorASpeed > 0) {
        // Forward
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_A, motorASpeed);
    } else if (motorASpeed < 0) {
        // Backward
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
        ledcWrite(PWM_CHANNEL_A, -motorASpeed);
    } else {
        // Stop (brake)
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_A, 0);
    }
}
```

**What This Does:**

**Step 1: Constrain Input**
```
motorASpeed = constrain(speed, -255, 255);
```
- Ensures speed is within valid range
- Negative = reverse, positive = forward

**Step 2: Determine Direction & Speed**

**Forward (speed > 0):**
```
digitalWrite(AIN1_PIN, HIGH);
digitalWrite(AIN2_PIN, LOW);
ledcWrite(PWM_CHANNEL_A, motorASpeed);
```
- Set direction pins for forward
- Write speed to PWM channel
- Example: `setMotorA(150)` → forward at speed 150

**Reverse (speed < 0):**
```
digitalWrite(AIN1_PIN, LOW);
digitalWrite(AIN2_PIN, HIGH);
ledcWrite(PWM_CHANNEL_A, -motorASpeed);
```
- Set direction pins for reverse
- Write **absolute value** to PWM (can't be negative!)
- Example: `setMotorA(-100)` → reverse at speed 100

**Stop (speed = 0):**
```
digitalWrite(AIN1_PIN, LOW);
digitalWrite(AIN2_PIN, LOW);
ledcWrite(PWM_CHANNEL_A, 0);
```
- Both direction pins LOW = brake
- PWM = 0 (no power)
- Motor actively braked (not coasting)

**setMotorB() is Identical** - just uses BIN1/BIN2 pins

---

### **7. applyPIDCorrection() - Differential Steering**

```
void applyPIDCorrection(int correction) {
    int leftMotorSpeed = baseSpeed - correction;
    int rightMotorSpeed = baseSpeed + correction;
    
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    
    setMotorA(leftMotorSpeed);   // Left motor
    setMotorB(rightMotorSpeed);  // Right motor
}
```

**Differential Steering Explained:**

**Concept:** Make one wheel faster, one slower → robot turns

**Math:**
```
Left Motor  = Base Speed - Correction
Right Motor = Base Speed + Correction
```

**Example Scenarios:**

**Scenario 1: Line Centered (correction = 0)**
```
Left  = 150 - 0 = 150
Right = 150 + 0 = 150
→ Both motors same speed → GO STRAIGHT ✓
```

**Scenario 2: Line to Right (correction = +50)**
```
Left  = 150 - 50 = 100   (slower)
Right = 150 + 50 = 200   (faster)
→ Left slower, right faster → TURN RIGHT ✓
```

**Scenario 3: Line to Left (correction = -50)**
```
Left  = 150 - (-50) = 200   (faster)
Right = 150 + (-50) = 100   (slower)
→ Left faster, right slower → TURN LEFT ✓
```

**Scenario 4: Sharp Turn (correction = +180)**
```
Left  = 150 - 180 = -30   (reverse!)
Right = 150 + 180 = 255   (max forward)
→ One reverses, one maxes → SHARP RIGHT TURN ✓
```

**Why Constrain?**
- Prevents values outside -255 to +255
- Protects motor driver from invalid commands

---

### **8. Helper Functions - Common Movements**

**stopMotors() - Full Stop**
```
void stopMotors() {
    setMotorA(0);
    setMotorB(0);
}
```
- Both motors brake
- Used for emergency stop, line lost

**moveForward() - Straight Ahead**
```
void moveForward() {
    setMotorA(baseSpeed);
    setMotorB(baseSpeed);
}
```
- Both motors at base speed
- No turning correction

**moveBackward() - Reverse**
```
void moveBackward() {
    setMotorA(-baseSpeed);
    setMotorB(-baseSpeed);
}
```
- Both motors reverse at base speed
- Useful for backing up

**turnLeft() - Spin in Place**
```
void turnLeft(int speed = 100) {
    setMotorA(-speed);   // Left reverse
    setMotorB(speed);    // Right forward
}
```
- Left motor reverse, right forward
- Robot pivots on center axis
- Default speed = 100 (adjustable)

**turnRight() - Spin in Place**
```
void turnRight(int speed = 100) {
    setMotorA(speed);    // Left forward
    setMotorB(-speed);   // Right reverse
}
```
- Left motor forward, right reverse
- Robot pivots on center axis

---

### **9. setBaseSpeed() - Speed Adjustment**

```
void setBaseSpeed(int speed) {
    baseSpeed = constrain(speed, 0, 255);
    Serial.print("Base speed set to: ");
    Serial.println(baseSpeed);
}
```

**What This Does:**
- Updates global `baseSpeed` variable
- Used by `applyPIDCorrection()`
- Constrains to 0-255 (no negative base speed)

**Usage:**
```
setBaseSpeed(100);  // Slow speed (testing)
setBaseSpeed(150);  // Medium speed (default)
setBaseSpeed(200);  // Fast speed (competition)
```

**Called By:**
- BLE callback (wireless speed adjustment)
- Manual tuning in code

---

### **10. getMotorSpeeds() - Read Current Speeds**

```
void getMotorSpeeds(int* leftSpeed, int* rightSpeed) {
    *leftSpeed = motorASpeed;
    *rightSpeed = motorBSpeed;
}
```

**What This Does:**
- Returns current motor speeds via pointers
- Used for debugging/monitoring

**Usage:**
```
int left, right;
getMotorSpeeds(&left, &right);
Serial.print("Left: ");
Serial.print(left);
Serial.print(" Right: ");
Serial.println(right);
```

**Output Example:**
```
Left: 120 Right: 180  (turning right)
Left: 150 Right: 150  (going straight)
Left: -50 Right: 200  (sharp right)
```

---

### **11. Emergency Stop & Resume**

**emergencyStop():**
```
void emergencyStop() {
    stopMotors();
    Serial.println("EMERGENCY STOP ACTIVATED!");
}
```
- Immediately stops both motors
- Called when BLE sends stop command
- Called when line is lost

**resume():**
```
void resume() {
    Serial.println("Motors resumed");
}
```
- Currently just prints message
- Motors resume automatically when PID runs again
- Could add initialization logic if needed

---

## How to Customize

### **Change Motor Pins**

```
// Use different GPIOs
#define PWMA_PIN    16
#define AIN1_PIN    17
#define AIN2_PIN    25
// Update all 6 pins
```

### **Change PWM Frequency**

```
// Quieter (higher frequency)
#define PWM_FREQ 30000  // 30kHz

// More torque (lower frequency)
#define PWM_FREQ 10000  // 10kHz
```

### **Change Base Speed**

```
int baseSpeed = 100;  // Slower default
int baseSpeed = 200;  // Faster default
```

### **Add Acceleration Ramping**

```
void setMotorSmooth(int target, int& current) {
    if (current < target) {
        current += 5;  // Ramp up slowly
    } else if (current > target) {
        current -= 5;  // Ramp down slowly
    }
}
```

### **Add Motor Trim (Correction for Uneven Motors)**

```
int leftTrim = 0;   // Adjust if left motor faster/slower
int rightTrim = 5;  // Add 5 to right motor

void applyPIDCorrection(int correction) {
    int left = baseSpeed - correction + leftTrim;
    int right = baseSpeed + correction + rightTrim;
    
    setMotorA(left);
    setMotorB(right);
}
```

### **Reverse Motor Direction** (If Wired Backwards)

```
// Swap HIGH/LOW in setMotorA
if (motorASpeed > 0) {
    digitalWrite(AIN1_PIN, LOW);   // Swapped
    digitalWrite(AIN2_PIN, HIGH);  // Swapped
    ledcWrite(PWM_CHANNEL_A, motorASpeed);
}
```

---

## Troubleshooting

### **Problem: Motors don't move**
**Cause:** No power, or STBY pin LOW
**Solution:**
- Check battery voltage (should be 6-12V)
- Ensure VM connected to battery +
- **Add STBY pin control:**
  ```
  #define STBY_PIN 17
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);  // Enable driver
  ```

### **Problem: One motor doesn't work**
**Cause:** Wiring issue or broken channel
**Solution:**
- Swap motor wires (A ↔ B)
- Check GPIO connections
- Test with `setMotorA(100)` directly

### **Problem: Motors spin wrong direction**
**Cause:** Motor wires reversed
**Solution:**
- Swap motor wires physically, OR
- Reverse in code (see customization above)

### **Problem: Motors jitter/vibrate**
**Cause:** PWM frequency too low
**Solution:**
```
#define PWM_FREQ 25000  // Increase frequency
```

### **Problem: Motors weak at low speeds**
**Cause:** Motor stall, or insufficient voltage
**Solution:**
- Increase minimum speed:
  ```
  if (speed > 0 && speed < 50) speed = 50;  // Min speed 50
  ```
- Check battery voltage (should be > 6V)

### **Problem: Robot doesn't go straight**
**Cause:** Motors have different speeds
**Solution:**
- Add motor trim (see customization)
- Calibrate mechanically (adjust wheel alignment)

---

## Speed Reference Table

| PWM Value | Percentage | Speed | Use Case |
|-----------|-----------|-------|----------|
| 0 | 0% | Stop | Brake, line lost |
| 50 | 20% | Very Slow | Initial testing |
| 100 | 39% | Slow | Tight turns, debugging |
| 150 | 59% | Medium | Default cruising speed |
| 200 | 78% | Fast | Competition mode |
| 255 | 100% | Max | Emergency corrections |

**Typical Values:**
- **Testing:** 80-120
- **Line Following:** 120-180
- **Competition:** 180-220
- **Speed Runs:** 220-255

---

## Key Concepts

### **H-Bridge**
- Circuit that allows bidirectional motor control
- Four switches arranged in "H" shape
- TB6612FNG has dual H-bridges (2 motors)

### **PWM (Pulse Width Modulation)**
- Rapid ON/OFF switching to control average power
- Duty cycle = % of time ON
- Frequency = how fast it switches

### **Differential Steering**
- Control direction by varying wheel speeds
- One faster, one slower = turn
- Both same speed = straight

### **Constrain**
- Limits value to min/max range
- Prevents invalid motor commands
- Formula: `constrain(value, min, max)`

### **LEDC (LED Controller)**
- ESP32 hardware PWM peripheral
- 16 independent channels
- Originally for LEDs, perfect for motors

---

## Testing Checklist

**Hardware:**
- [ ] All 6 GPIO pins connected correctly
- [ ] Battery connected to VM and GND
- [ ] Common ground (ESP32 + Battery)
- [ ] STBY pin HIGH (if used)
- [ ] Motors connected to AO1/AO2, BO1/BO2

**Software:**
- [ ] `initMotors()` called in setup()
- [ ] Serial Monitor shows initialization
- [ ] `setMotorA(100)` makes left motor spin
- [ ] `setMotorB(100)` makes right motor spin
- [ ] `moveForward()` moves robot forward
- [ ] `stopMotors()` stops robot

**Direction:**
- [ ] Both motors spin forward with positive speed
- [ ] Motors reverse with negative speed
- [ ] Robot goes straight with equal speeds
- [ ] Robot turns correctly with PID

---

## Pro Tips

1. **Test motors individually** - `setMotorA(100)` before full system
2. **Start slow** - Use baseSpeed=80 for initial testing
3. **Check polarity** - Swap wires if direction wrong
4. **Add STBY control** - Some boards require explicit enable
5. **Monitor current** - High current = mechanical binding
6. **Use thick wires** - 18-20 AWG for motor power
7. **Add capacitors** - 100µF across VM/GND reduces noise

---

## Integration with Other Modules

### **Uses:**
- ESP32 GPIO and LEDC hardware
- Arduino `digitalWrite()` and `ledcWrite()` functions

### **Used By:**
- `main.cpp` - Calls `applyPIDCorrection()`
- `BLEConfig.h` - Calls `setBaseSpeed()`
- `PIDController.h` - Provides correction values

### **Data Flow:**
```
PIDController.h
  ↓
correction (-255 to +255)
  ↓
applyPIDCorrection(correction)
  ↓
leftSpeed = baseSpeed - correction
rightSpeed = baseSpeed + correction
  ↓
setMotorA(leftSpeed)
setMotorB(rightSpeed)
  ↓
TB6612FNG Motor Driver
  ↓
DC Motors (Physical Movement)
```

---

## Quick Test Code

```
#include "Motor/MotorControl.h"

void setup() {
  Serial.begin(115200);
  initMotors();
}

void loop() {
  // Test 1: Forward
  Serial.println("Forward");
  moveForward();
  delay(2000);
  
  // Test 2: Stop
  Serial.println("Stop");
  stopMotors();
  delay(1000);
  
  // Test 3: Turn Right
  Serial.println("Turn Right");
  turnRight(100);
  delay(1000);
  
  // Test 4: Turn Left
  Serial.println("Turn Left");
  turnLeft(100);
  delay(1000);
  
  // Test 5: Backward
  Serial.println("Backward");
  moveBackward();
  delay(2000);
  
  // Stop
  stopMotors();
  delay(5000);  // Wait 5 seconds before repeat
}
```

**Expected Behavior:**
1. Robot moves forward 2 seconds
2. Stops for 1 second
3. Spins right 1 second
4. Spins left 1 second
5. Moves backward 2 seconds
6. Waits 5 seconds
7. Repeats

---