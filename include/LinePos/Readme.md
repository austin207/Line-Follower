# Linepos.h - Line Position Calculation

## What Does This File Do?

This file converts **8 individual sensor readings** into a **single position value** that tells you where the black line is relative to the center of your robot.

Think of it as **combining 8 eyes into one brain** - instead of processing 8 numbers, you get one answer: "the line is 2cm to the left."

---

## Why Do We Need This?

### **The Problem:**

You have 8 sensors giving you 8 different numbers:
```
Sensors: [450, 478, 2345, 2567, 2601, 523, 465]
```
- Is it left or right of center?
- How far off-center?

Looking at 8 numbers every loop is confusing and hard to use for motor control!

### **The Solution:**

This module calculates a **weighted average** that gives you ONE number:
```
Position: +15  (line is slightly right of center)
```

Now you can easily:
- Feed this to PID controller
- Make steering decisions
- Know exactly where the line is

---

## How It Works

### **The Weighted Average Algorithm**

**Core Concept:** Sensors closer to the line contribute MORE to the final position.

**Formula:**
```
Position = Σ(sensorValue[i] × weight[i]) / Σ(active sensor values)
```

**In English:**
1. Multiply each sensor reading by its weight
2. Add up all the weighted values
3. Divide by sum of all active sensor values
4. Result = position of line

---

##  Understanding Sensor Weights

### **Weight Array**

```
const int SENSOR_WEIGHTS = {-4, -3, -2, -1, 1, 2, 3, 4};
```

**Visual Representation:**
```
        LEFT               CENTER               RIGHT
         ↓                   ↓                    ↓
Sensor:  0    1    2    3    |    4    5    6    7
Weight: -4   -3   -2   -1    |    1    2    3    4
        ←────────────────────┼────────────────────→
     Negative values         0         Positive values
```

**What This Means:**

| Sensor | Weight | Position | Meaning |
|--------|--------|----------|---------|
| 0 | -4 | Far Left | Most negative influence |
| 1 | -3 | Left | Negative influence |
| 2 | -2 | Left-Center | Slightly negative |
| 3 | -1 | Left-Center | Just left of center |
| 4 | +1 | Right-Center | Just right of center |
| 5 | +2 | Right-Center | Slightly positive |
| 6 | +3 | Right | Positive influence |
| 7 | +4 | Far Right | Most positive influence |

**Why No Zero Weight?**
- Zero weight would mean "ignore this sensor"
- We skip from -1 to +1 to avoid dead zone
- Center reference is BETWEEN sensors 3 and 4

---

## Code Breakdown

### **1. Sensor Weights - Position Mapping**

```
const int SENSOR_WEIGHTS = {-4, -3, -2, -1, 1, 2, 3, 4};
```

**Why These Specific Numbers?**

**Linear Spacing:**
- Creates evenly-spaced position values
- Each sensor is 1 "unit" apart
- Simple and intuitive

**Symmetric:**
- Left side: negative (-4 to -1)
- Right side: positive (+1 to +4)
- Makes math easier for PID

**Customization Options:**

**Option 1: Non-linear (emphasize edges)**
```
const int SENSOR_WEIGHTS = {-8, -4, -2, -1, 1, 2, 4, 8};
// Edge sensors have MORE influence
```

**Option 2: Wider spacing**
```
const int SENSOR_WEIGHTS = {-40, -30, -20, -10, 10, 20, 30, 40};
// Same ratios, bigger numbers
```

**Option 3: Include zero (6 sensors in use)**
```
const int SENSOR_WEIGHTS = {-3, -2, -1, 0, 0, 1, 2, 3};
// Center 2 sensors ignored
```

---

### **2. readLinePosition() - Main Position Calculator**

```
int readLinePosition() {
    int weightedSum = 0;      // Σ(value × weight)
    int activeSensors = 0;    // Σ(values)
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int value = sensorValues[i];
        
        // Only use sensors above threshold (seeing black line)
        if (value > 1500) {
            weightedSum += SENSOR_WEIGHTS[i] * value;
            activeSensors += value;
        }
    }
    
    // Line lost - no sensors active
    if (activeSensors == 0) {
        return -999;
    }
    
    // Calculate weighted average
    return weightedSum / activeSensors;
}
```

**Step-by-Step Example:**

**Scenario:** Line is centered between sensors 3 and 4

**Raw Sensor Values:**
```
Sensor:  0     1     2     3     4     5     6     7
Value:   450   478   501   2567  2601  523   465   485
Weight: -4    -3    -2    -1     1     2     3     4
```

**Step 1: Filter by Threshold** (value > 1500)
```
Active sensors: 3 (value=2567) and 4 (value=2601)
Inactive sensors: 0,1,2,5,6,7 (ignored)
```

**Step 2: Calculate Weighted Sum**
```
weightedSum = (WEIGHT × VALUE) + (WEIGHT × VALUE)[2][3]
            = (-1 × 2567) + (1 × 2601)
            = -2567 + 2601
            = 34
```

**Step 3: Calculate Sum of Active Sensors**
```
activeSensors = VALUE + VALUE[3][2]
              = 2567 + 2601
              = 5168
```

**Step 4: Calculate Position**
```
position = weightedSum / activeSensors
         = 34 / 5168
         = 0.0065
         ≈ 0 (centered!)
```

**Result:** Position ≈ 0 = Line is centered ✓

---

### **3. Threshold - Line Detection Cutoff**

```
if (value > 1500) { // Adjust this threshold
```

**What This Does:**
- Only sensors reading **above 1500** are considered "seeing the line"
- Sensors below 1500 are ignored (seeing white surface)

**Why 1500?**
- Typical midpoint between white (400-800) and black (2500-3500)
- Prevents white surface from influencing position
- Acts as noise filter

**How to Adjust:**

**Method 1: Use Calibration** (Recommended)
```
if (value > sensorThreshold[i]) {  // Use calibrated threshold
```

**Method 2: Manual Testing**
```
// Print raw values to find your threshold
void loop() {
  readIRSensors();
  Serial.println(sensorValues);  // Check white vs black values
}

// If white=600, black=3000, use threshold around 1800
if (value > 1800) {
```

**Method 3: Percentage-based**
```
int threshold = (sensorMin[i] + sensorMax[i]) / 2;  // 50% of range
if (value > threshold) {
```

---

### **4. Line Lost Detection**

```
if (activeSensors == 0) {
    return -999;  // Error code
}
```

**When This Happens:**
- No sensors see the line (all below threshold)
- Robot went off track
- Line ended (finish line scenario)

**Why -999?**
- Clear error code (impossible position)
- Easy to check: `if (position == -999)`
- Different from valid positions (-4 to +4)

**Usage in main.cpp:**
```
int position = readLinePosition();

if (position == -999) {
    stopMotors();  // Safety - lost line
    Serial.println("LINE LOST!");
} else {
    // Normal operation
    applyPIDCorrection(position);
}
```

---

### **5. Position Value Range**

**Raw Position Range:**

The formula gives values roughly in this range:

**Far Left** (only sensor 0 active):
```
position = (-4 × 3000) / 3000 = -4
```

**Far Right** (only sensor 7 active):
```
position = (4 × 3000) / 3000 = 4
```

**Centered** (sensors 3 & 4 active equally):
```
position = ((-1 × 2500) + (1 × 2500)) / 5000 = 0
```

**But it's NOT limited to -4 to +4!**

**Why?** The weighted average can produce values outside this range:

**Example: Line spanning 3 sensors**
```
Sensor 2: value=2000, weight=-2
Sensor 3: value=2500, weight=-1
Sensor 4: value=2300, weight=+1

weightedSum = (-2×2000) + (-1×2500) + (1×2300)
            = -4000 - 2500 + 2300 = -4200

activeSensors = 2000 + 2500 + 2300 = 6800

position = -4200 / 6800 = -0.617
```

**Typical Range:** -5 to +5 (but can be wider)

---

### **6. readLinePositionNormalized() - Scaled Output**

```
int readLinePositionNormalized() {
    int rawPosition = readLinePosition();
    
    if (rawPosition == -999) {
        return -999;  // Preserve error code
    }
    
    // Scale to -100 to +100
    return constrain(rawPosition * 25, -100, 100);
}
```

**What This Does:**

**Step 1: Get Raw Position**
```
int rawPosition = readLinePosition();  // e.g., -2.3
```

**Step 2: Scale by 25**
```
scaled = rawPosition * 25
       = -2.3 * 25
       = -57.5
```

**Step 3: Constrain to ±100**
```
final = constrain(-57.5, -100, 100)
      = -57  (within range, unchanged)
```

**Why Scale by 25?**
- Raw range ≈ -4 to +4
- Target range = -100 to +100
- Scale factor = 100 / 4 = 25

**Why Constrain?**
- Prevents extreme outliers
- Guarantees -100 ≤ position ≤ +100
- Safe for PID calculations

**Comparison:**

| Position Type | Range | Use Case |
|---------------|-------|----------|
| Raw | -5 to +5 | Direct math, debugging |
| Normalized | -100 to +100 | PID control, easier tuning |

---

## How to Customize

### **Change Threshold**

```
// Option 1: Fixed threshold
if (value > 2000) {  // Higher threshold (more strict)

// Option 2: Use calibration
if (value > sensorThreshold[i]) {

// Option 3: Percentage
int thresh = sensorMin[i] + (sensorMax[i] - sensorMin[i]) * 0.6;
if (value > thresh) {
```

### **Change Weight Distribution**

**Exponential Weights** (favor edges):
```
const int SENSOR_WEIGHTS = {-16, -8, -4, -2, 2, 4, 8, 16};
```

**Linear with Zero Center**:
```
const int SENSOR_WEIGHTS = {-7, -5, -3, -1, 1, 3, 5, 7};
```

**Custom Asymmetric**:
```
const int SENSOR_WEIGHTS = {-10, -6, -3, -1, 1, 3, 6, 10};
// More sensitivity on left side
```

### **Change Normalization Scale**

```
// Scale to -255 to +255 (motor PWM range)
return constrain(rawPosition * 64, -255, 255);

// Scale to -1000 to +1000 (high precision)
return constrain(rawPosition * 250, -1000, 1000);
```

### **Add Smoothing Filter**

```
int readLinePositionSmoothed() {
    static int lastPosition = 0;
    int newPosition = readLinePosition();
    
    if (newPosition == -999) return -999;
    
    // Exponential moving average (smoothing)
    lastPosition = (lastPosition * 7 + newPosition) / 8;
    return lastPosition;
}
```

---

## Troubleshooting

### **Problem: Position always -999**
**Cause:** No sensors above threshold
**Solution:**
- Lower threshold: `if (value > 1000)`
- Check sensor readings: `printSensorValues()`
- Verify sensors see black line

### **Problem: Position jumps erratically**
**Cause:** Threshold too low, noise
**Solution:**
- Raise threshold: `if (value > 2000)`
- Add smoothing filter
- Calibrate sensors

### **Problem: Position stuck at extreme (-4 or +4)**
**Cause:** Line at edge, or only one sensor active
**Solution:**
- Normal behavior at track edges
- Adjust robot starting position
- Widen sensor array spacing

### **Problem: Position doesn't change**
**Cause:** Multiple sensors always active equally
**Solution:**
- Check sensor spacing (should be ~8-10mm apart)
- Verify line width matches sensor spacing
- Try different threshold

### **Problem: Normalized position always ±100**
**Cause:** Raw position exceeds ±4, gets clamped
**Solution:**
- Normal for extreme positions
- Adjust scale factor: `* 20` instead of `* 25`
- Use raw position instead of normalized

---

## Example Scenarios

### **Scenario 1: Line Centered**
```
Sensors: 
Active:  [   -    -    -    3     4    -    -    -  ]
Weights: [  -4   -3   -2   -1     1    2    3    4  ]

weightedSum = (-1 × 2567) + (1 × 2601) = 34
activeSensors = 2567 + 2601 = 5168
position = 34 / 5168 = 0.006 ≈ 0

Result: CENTERED ✓
```

### **Scenario 2: Line Slightly Left**
```
Sensors: [450, 478, 2345, 2601, 523, 465, 485,  -    -    -  ]
Weights: [ -4   -3   -2    -1    1    2    3    4  ]

weightedSum = (-2 × 2345) + (-1 × 2601) = -7291
activeSensors = 2345 + 2601 = 4946
position = -7291 / 4946 = -1.47

Result: LEFT OF CENTER ✓
Normalized: -1.47 × 25 = -37
```

### **Scenario 3: Line Far Right**
```
Sensors: [450, 478, 501, 523, 465, 485, 2567,  -    6     7  ]
Weights: [ -4   -3   -2   -1    1    2    3     4  ]

weightedSum = (3 × 2567) + (4 × 2890) = 19261
activeSensors = 2567 + 2890 = 5457
position = 19261 / 5457 = 3.53

Result: FAR RIGHT ✓
Normalized: 3.53 × 25 = 88
```

### **Scenario 4: Line Lost**
```
Sensors: [450, 478, 501, 523, 465, 485, 490, 495    -    -  ]

No sensors above threshold (1500)
activeSensors = 0

Result: -999 (LINE LOST) ✓
```

---

## Key Concepts

### **Weighted Average**
- Mathematical technique to find "center of mass"
- Each value contributes based on its weight
- Used in physics, statistics, robotics

### **Threshold**
- Cutoff value to separate signal from noise
- Values above = "line detected"
- Values below = "no line"

### **Normalization**
- Converting values to standard range
- Makes numbers easier to work with
- PID tuning becomes environment-independent

### **Error Code**
- Special value indicating problem/failure
- -999 is impossible position (clear error)
- Better than returning 0 (could be valid center position)

### **Constrain**
- Limits value to min/max range
- Prevents overflow/extreme values
- Formula: `min(max(value, minVal), maxVal)`

---

## Testing Checklist

**Basic Function:**
- [ ] Line centered → position ≈ 0
- [ ] Line left → position < 0
- [ ] Line right → position > 0
- [ ] No line → position = -999

**Range Testing:**
- [ ] Far left → position ≈ -3 to -4
- [ ] Far right → position ≈ +3 to +4
- [ ] Normalized stays within ±100

**Edge Cases:**
- [ ] Single sensor active → valid position
- [ ] All sensors active → position ≈ 0
- [ ] No sensors active → -999

**Threshold:**
- [ ] White surface → sensors not active
- [ ] Black line → sensors active
- [ ] Adjust threshold if needed

---

## Pro Tips

1. **Print position continuously** - helps visualize line tracking
2. **Test with normalized** - easier to understand ±100 scale
3. **Threshold is critical** - spend time tuning it
4. **Use calibration** - makes threshold automatic
5. **Watch for -999** - indicates robot off track
6. **Smooth if needed** - reduces PID jitter
7. **Log position data** - helps debug PID tuning

---

## Integration with Other Modules

### **Uses:**
- `IRSensor.h` - Reads `sensorValues[]` array
- `Calibration.h` (optional) - Can use `sensorThreshold[]`

### **Used By:**
- `PIDController.h` - Feeds position to PID
- `main.cpp` - Calls `readLinePosition()` every loop

### **Data Flow:**
```
IRSensor.h
  ↓
sensorValues (raw: 0-4095)[1]
  ↓
readLinePosition()
  ↓
Weighted Average Calculation
  ↓
Position (-5 to +5)
  ↓
readLinePositionNormalized() (optional)
  ↓
Position (-100 to +100)
  ↓
PIDController.h (uses as input)
```

---

## Quick Test Code

```
#include "IR/IRSensor.h"
#include "LinePos/Linepos.h"

void setup() {
  Serial.begin(115200);
  initIRSensors();
}

void loop() {
  readIRSensors();
  
  int rawPos = readLinePosition();
  int normPos = readLinePositionNormalized();
  
  if (rawPos == -999) {
    Serial.println("LINE LOST!");
  } else {
    Serial.print("Raw: ");
    Serial.print(rawPos);
    Serial.print(" | Normalized: ");
    Serial.println(normPos);
  }
  
  delay(100);
}
```

**Expected Output:**
```
Raw: 0 | Normalized: 0          (centered)
Raw: -1.5 | Normalized: -37     (slightly left)
Raw: 2.3 | Normalized: 57       (right)
LINE LOST!                       (off track)
```

---