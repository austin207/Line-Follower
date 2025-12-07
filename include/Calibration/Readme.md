# Calibration.h - IR Sensor Auto-Calibration System

## What Does This File Do?

This file automatically **adapts your IR sensors** to different environments (lighting conditions, surface colors, track materials). Without calibration, your robot might work perfectly on one track but fail completely on another!

Think of it like **adjusting your camera's exposure** - the same scene looks different in bright sunlight vs. dim room lighting.

---

## Why Do We Need Calibration?

### **The Problem:**

IR sensors return **analog values** (0-4095 on ESP32), but these numbers change based on:
- Room lighting (bright office vs. dark garage)
- Surface color (pure black vs. dark gray)
- Track material (matte vs. glossy paper)
- Sensor height above ground
- Battery voltage

**Example:**
- **Bright room**: White=500, Black=2500
- **Dark room**: White=1500, Black=3800
- **Same track, different readings!**

### **The Solution:**

Calibration **learns** the min/max values for your specific environment, then **normalizes** all future readings to a standard 0-1000 scale.

---

## How It Works

### **Calibration Process (3 Steps)**

```
1. COLLECT DATA
   ↓
   Move sensors over BLACK and WHITE for 5 seconds
   Record the LOWEST value seen (white surface)
   Record the HIGHEST value seen (black line)

2. CALCULATE THRESHOLD
   ↓
   Threshold = (Min + Max) / 2
   Any value above threshold = "line detected"

3. SAVE TO FLASH
   ↓
   Store calibration data permanently
   Survives power-off and reboots
```

### **Normalization Formula**

```
normalized = (rawValue - min) * 1000 / (max - min)
```

**Example:**
- Min (white) = 500
- Max (black) = 2500
- Raw reading = 1500

```
normalized = (1500 - 500) * 1000 / (2500 - 500)
           = 1000 * 1000 / 2000
           = 500  (middle of 0-1000 range)
```

---

## Code Breakdown

### **1. Storage Arrays - The Memory**

```
int sensorMin;       // Lowest value each sensor saw (white surface)[1]
int sensorMax;       // Highest value each sensor saw (black line)[1]
int sensorThreshold; // Midpoint = (min + max) / 2[1]
bool isCalibrated = false; // Has calibration been done?
```

**What These Store:**
- `sensorMin[0]` = Sensor 1's minimum reading
- `sensorMax[0]` = Sensor 1's maximum reading
- `sensorThreshold[0]` = Sensor 1's threshold (decision point)

**Why 8?** - One set of values for each of the 8 IR sensors

---

### **2. Preferences - Persistent Storage**

```
Preferences preferences;
```

**What is Preferences?**
- ESP32's built-in **flash memory storage**
- Like a tiny database that survives power-off
- Stores data as key-value pairs (like a dictionary)

**Why Use It?**
- Without it: Calibration lost when robot powers off
- With it: Calibration loads automatically on startup

**Flash Memory Lifespan:**
- ~100,000 write cycles per location
- We only write during calibration (not a problem!)

---

### **3. initCalibration() - Startup Function**

```
void initCalibration() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;  // Start with highest possible
        sensorMax[i] = 0;     // Start with lowest possible
        sensorThreshold[i] = 2000; // Middle of 0-4095 range
    }
    loadCalibration();  // Try to load saved data
}
```

**What This Does:**
1. **Sets defaults** (used if no saved calibration exists)
2. **Tries to load** saved calibration from flash
3. If found → uses saved values
4. If not found → uses defaults (2000 threshold)

**Why These Defaults?**
- `sensorMin[i] = 4095`: Start high, update downward when white is seen
- `sensorMax[i] = 0`: Start low, update upward when black is seen
- `sensorThreshold[i] = 2000`: Safe middle value (works okay uncalibrated)

---

### **4. calibrate() - The Main Event**

```
void calibrate(unsigned long duration = 5000) {
    // 1. Reset values
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }
    
    // 2. Collect data for 5 seconds
    while (millis() - startTime < duration) {
        readIRSensors();
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (sensorValues[i] < sensorMin[i]) {
                sensorMin[i] = sensorValues[i];  // Update minimum
            }
            if (sensorValues[i] > sensorMax[i]) {
                sensorMax[i] = sensorValues[i];  // Update maximum
            }
        }
    }
    
    // 3. Calculate thresholds
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    }
    
    // 4. Save to flash
    saveCalibration();
}
```

**Step-by-Step:**

**Step 1: Reset** - Clear old calibration data

**Step 2: Collect** (5 seconds)
- Read sensors continuously
- Track the **lowest** value seen (white surface)
- Track the **highest** value seen (black line)
- Shows dots (`.....`) for progress feedback

**Step 3: Calculate**
- Threshold = average of min and max
- This is the "decision point" for line detection

**Step 4: Save**
- Write to flash memory
- Survives power-off

**How to Use:**
1. Call `calibrate()` from main.cpp or via BLE
2. **Slowly move** robot over black and white for 5 seconds
3. Make sure sensors see **both** black line and white surface
4. Done! Calibration saved automatically

**Changing Duration:**
```
calibrate(3000);   // 3 second calibration (faster but less accurate)
calibrate(10000);  // 10 second calibration (slower but more thorough)
```

---

### **5. readCalibratedSensors() - Normalize Readings**

```
void readCalibratedSensors(int* calibratedValues) {
    readIRSensors();  // Get raw values
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int range = sensorMax[i] - sensorMin[i];
        
        if (range == 0) {  // Safety: prevent divide-by-zero
            calibratedValues[i] = 0;
            continue;
        }
        
        // Map to 0-1000 range
        long value = (long)(sensorValues[i] - sensorMin[i]) * 1000 / range;
        calibratedValues[i] = constrain(value, 0, 1000);
    }
}
```

**What This Does:**
- Converts raw sensor values (0-4095) → normalized values (0-1000)
- **0** = definitely white surface
- **1000** = definitely black line
- **500** = middle (gray area)

**Why Normalize?**
- Consistent values across different environments
- Easier to work with (0-1000 is simpler than 0-4095)
- Makes PID tuning environment-independent

**Example Usage:**
```
int calibratedValues;[1]
readCalibratedSensors(calibratedValues);
Serial.println(calibratedValues);  // Prints 0-1000
```

---

### **6. isOnLine() - Simple Line Detection**

```
bool isOnLine(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= NUM_SENSORS) return false;
    return sensorValues[sensorIndex] > sensorThreshold[sensorIndex];
}
```

**What This Does:**
- Returns `true` if sensor sees black line
- Returns `false` if sensor sees white surface

**How It Works:**
- If raw value > threshold → black line detected
- If raw value < threshold → white surface detected

**Example Usage:**
```
if (isOnLine(0)) {
    Serial.println("Sensor 0 sees the line!");
}

if (isOnLine(3) && isOnLine(4)) {
    Serial.println("Line is centered!");
}
```

---

### **7. getLineBinary() - Visual Debugging**

```
uint8_t getLineBinary() {
    uint8_t binary = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (isOnLine(i)) {
            binary |= (1 << i);  // Set bit i to 1
        }
    }
    return binary;
}
```

**What This Does:**
- Converts 8 sensor states into a single number (0-255)
- Each bit represents one sensor (1=line, 0=no line)

**Binary Representation:**
```
Sensors:  7  6  5  4  3  2  1  0
State:    0  0  1  1  1  1  0  0
Binary:   00111100 = 60 (decimal)
```

**Example Usage:**
```
uint8_t pattern = getLineBinary();
Serial.println(pattern, BIN);  // Prints: 00111100

// Common patterns:
// 00011000 = Line centered (sensors 3,4)
// 11110000 = Line on right side
// 00001111 = Line on left side
// 00000000 = Line lost
// 11111111 = Wide line or intersection
```

**Why Use Binary?**
- Compact representation
- Easy to visualize sensor array state
- Good for debugging

---

### **8. printCalibrationData() - Show Results**

```
void printCalibrationData() {
    Serial.println("\n--- Calibration Data ---");
    Serial.print("Min Values: ");
    // Prints all 8 minimum values
    
    Serial.print("Max Values: ");
    // Prints all 8 maximum values
    
    Serial.print("Threshold:  ");
    // Prints all 8 threshold values
}
```

**Example Output:**
```
--- Calibration Data ---
Min Values: 450   480   470   460   455   465   475   490
Max Values: 2600  2550  2580  2590  2610  2570  2560  2540
Threshold:  1525  1515  1525  1525  1532  1517  1517  1515
```

**How to Read:**
- **Min** = White surface readings
- **Max** = Black line readings
- **Threshold** = Decision point (middle)

**Troubleshooting:**
- If Min ≈ Max → Sensor not working or didn't see both surfaces
- If Threshold near 0 or 4095 → Calibration failed

---

### **9. saveCalibration() - Write to Flash**

```
void saveCalibration() {
    preferences.begin("calibration", false);  // Open storage
    
    // Save all 8 sensors × 3 values = 24 numbers
    for (int i = 0; i < NUM_SENSORS; i++) {
        String minKey = "min" + String(i);  // "min0", "min1", ...
        preferences.putInt(minKey.c_str(), sensorMin[i]);
    }
    // Same for max and threshold...
    
    preferences.putBool("calibrated", true);  // Flag: data exists
    preferences.end();  // Close storage
}
```

**What This Does:**
- Opens "calibration" namespace in flash memory
- Saves 24 values (8 sensors × 3 values each)
- Uses keys like "min0", "max0", "thr0" for sensor 0

**Storage Structure:**
```
Namespace: "calibration"
├── min0: 450
├── min1: 480
├── max0: 2600
├── max1: 2550
├── thr0: 1525
├── thr1: 1515
└── calibrated: true
```

**Flash Usage:**
- ~200 bytes total
- ESP32 has ~512KB flash for storage (plenty of room!)

---

### **10. loadCalibration() - Read from Flash**

```
bool loadCalibration() {
    preferences.begin("calibration", true);  // Read-only mode
    
    bool wasPreviouslyCalibrated = preferences.getBool("calibrated", false);
    
    if (!wasPreviouslyCalibrated) {
        Serial.println("No saved calibration found.");
        return false;  // Use defaults
    }
    
    // Load all values from flash
    for (int i = 0; i < NUM_SENSORS; i++) {
        String minKey = "min" + String(i);
        sensorMin[i] = preferences.getInt(minKey.c_str(), 4095);
    }
    // Same for max and threshold...
    
    isCalibrated = true;
    return true;  // Success!
}
```

**What This Does:**
1. Check if calibration data exists
2. If yes → load it into arrays
3. If no → return false (use defaults)

**Auto-Load:**
- Called by `initCalibration()` during startup
- Robot automatically uses saved calibration

---

### **11. clearCalibration() - Factory Reset**

```
void clearCalibration() {
    preferences.begin("calibration", false);
    preferences.clear();  // Delete all data
    preferences.end();
    
    isCalibrated = false;
    Serial.println("Calibration data cleared!");
}
```

**When to Use:**
- Moving to new track/environment
- Sensor behavior seems wrong
- Want to start fresh

**Usage:**
```
clearCalibration();  // Delete saved data
calibrate(5000);     // Recalibrate from scratch
```

---

## How to Customize

### **Change Calibration Duration**
```
calibrate(3000);   // 3 seconds (faster)
calibrate(10000);  // 10 seconds (more thorough)
```

### **Change Default Threshold**
```
void initCalibration() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorThreshold[i] = 1800;  // Use 1800 instead of 2000
    }
}
```

### **Change Normalization Range**
```
// Instead of 0-1000, use 0-100
long value = (long)(sensorValues[i] - sensorMin[i]) * 100 / range;
calibratedValues[i] = constrain(value, 0, 100);
```

### **Add Calibration Button** (Hardware)
```
#define CALIBRATE_BUTTON_PIN 15

void setup() {
    pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
        calibrate(5000);
        delay(1000);  // Debounce
    }
}
```

---

## Troubleshooting

### **Problem: "No saved calibration found"**
**Cause:** First time running, or flash memory cleared
**Solution:** Normal! Just run calibration once

### **Problem: Min ≈ Max values**
**Cause:** Sensor didn't see both black AND white during calibration
**Solution:** 
- Make sure to move over BOTH surfaces
- Check sensor wiring
- Verify sensor height (too high = no contrast)

### **Problem: Threshold is 0 or 4095**
**Cause:** Calibration failed, using default
**Solution:**
- Check Serial Monitor during calibration
- Ensure sensors are reading (not broken)
- Try `clearCalibration()` then recalibrate

### **Problem: Line detection unstable**
**Cause:** Poor calibration or environmental changes
**Solution:**
- Recalibrate in actual competition environment
- Increase calibration duration to 10 seconds
- Check lighting hasn't changed drastically

### **Problem: Calibration not saving**
**Cause:** Flash memory full (rare) or write error
**Solution:**
- Check Serial Monitor for "saved" confirmation
- Try `preferences.clear()` to free space
- Verify ESP32 has enough flash (~512KB available)

---

## Calibration Quality Check

**Good Calibration:**
```
Min Values: 400-600    (white surface)
Max Values: 2400-2700  (black line)
Range: ~2000           (good contrast)
Threshold: ~1500       (middle point)
```

**Bad Calibration:**
```
Min Values: 1900-2100  (too high - didn't see white)
Max Values: 2000-2200  (too low - didn't see black)
Range: ~100            (poor contrast)
Threshold: 2000        (not meaningful)
```

**Ideal Range:** 1500-2500 difference between min and max

---

## Key Concepts

**Calibration:**
- Process of adapting system to specific environment
- Like "white balance" on a camera

**Normalization:**
- Converting raw values to standard scale (0-1000)
- Makes code environment-independent

**Threshold:**
- Decision point between "line" and "no line"
- Calculated as midpoint between min and max

**Persistent Storage:**
- Data that survives power-off
- Stored in ESP32 flash memory (not RAM)

**Binary Representation:**
- Compact way to show 8 sensor states
- Each bit = one sensor (1=line, 0=no line)

---

## Usage Checklist

**Initial Setup:**
- [ ] Call `initCalibration()` in setup()
- [ ] Check Serial Monitor for "loaded" or "not found" message

**First Calibration:**
- [ ] Trigger calibration via BLE or code
- [ ] Slowly move robot over black and white for 5 seconds
- [ ] Check Serial Monitor shows calibration data
- [ ] Verify "saved to flash" message appears

**Testing:**
- [ ] Move robot over line → sensors should detect it
- [ ] Use `getLineBinary()` to visualize sensor states
- [ ] Check thresholds make sense (middle of min/max)

**Troubleshooting:**
- [ ] Use `printCalibrationData()` to debug
- [ ] Recalibrate if environment changes
- [ ] Use `clearCalibration()` for fresh start

---

## Pro Tips

1. **Calibrate in competition environment** - don't calibrate at home then compete elsewhere!
2. **Move slowly** during calibration - ensures sensors see full range
3. **Check battery** - low voltage changes sensor readings
4. **Tilt sensors** slightly during calibration - captures edge cases
5. **Use normalized values** - more reliable than raw readings
6. **Print binary pattern** - great for debugging sensor behavior

---

## Integration with Other Modules

**Used By:**
- `Linepos.h` - Uses calibrated thresholds for line detection
- `main.cpp` - Calls `initCalibration()` and `calibrate()`

**Uses:**
- `IRSensor.h` - Reads raw sensor values via `readIRSensors()`
- `Preferences` library - Stores data in flash

**Workflow:**
```
1. initCalibration() → Load saved data (if exists)
2. calibrate() → Collect new calibration data
3. saveCalibration() → Store to flash
4. readCalibratedSensors() → Normalize raw values
5. isOnLine() → Line detection using thresholds
```

---