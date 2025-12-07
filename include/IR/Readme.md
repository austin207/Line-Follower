# IRSensor.h - IR Sensor Array Interface

## What Does This File Do?

This file is the **hardware interface** between your ESP32 and the 8-channel IR sensor array. It reads analog values from the sensors and stores them in memory for other parts of your code to use.

Think of it as the **eyes of your robot** - it continuously looks at the ground and reports what it sees!

---

## Why Do We Need This?

### **The Problem:**

Your robot needs to know:
- Where is the black line?
- Am I on the track or off it?
- Which sensors see the line?

But sensors speak in **numbers** (voltages), not English!

### **The Solution:**

This module **translates** sensor voltages into digital values (0-4095) that your code can understand and process.

---

## Hardware Setup

### **IR Sensor Array**

**Type:** QTR-8A or QTR-8RC (8-channel analog reflectance sensor)

**How IR Sensors Work:**
1. **Emit** infrared light at the ground
2. **Measure** how much light reflects back
3. **More reflection** (white surface) = Lower voltage = Lower ADC value
4. **Less reflection** (black line) = Higher voltage = Higher ADC value

### **Pin Connections**

| Sensor | ESP32 Pin | GPIO | ADC Channel | Description |
|--------|-----------|------|-------------|-------------|
| A1 | D13 | GPIO13 | ADC2_4 | Leftmost sensor |
| A2 | D12 | GPIO12 | ADC2_5 | Left sensor 2 |
| A3 | D14 | GPIO14 | ADC2_6 | Left sensor 3 |
| A4 | D27 | GPIO27 | ADC2_7 | Left-center sensor |
| A5 | D26 | GPIO26 | ADC2_9 | Right-center sensor |
| A6 | D25 | GPIO25 | ADC2_8 | Right sensor 3 |
| A7 | D33 | GPIO33 | ADC1_5 | Right sensor 2 |
| A8 | D32 | GPIO32 | ADC1_4 | Rightmost sensor |

### **Wiring Diagram**

```
IR Sensor Array               ESP32
┌─────────────┐           ┌──────────┐
│ VCC         │───────────│ 3.3V/5V  │
│ GND         │───────────│ GND      │
│ A1          │───────────│ GPIO13   │
│ A2          │───────────│ GPIO12   │
│ A3          │───────────│ GPIO14   │
│ A4          │───────────│ GPIO27   │
│ A5          │───────────│ GPIO26   │
│ A6          │───────────│ GPIO25   │
│ A7          │───────────│ GPIO33   │
│ A8          │───────────│ GPIO32   │
└─────────────┘           └──────────┘
```

**Power Requirements:**
- Voltage: 3.3V or 5V (check your sensor datasheet)
- Current: ~100mA max (all 8 LEDs on)
- **Note:** Some sensors need 5V for optimal performance

---

## Code Breakdown

### **1. Pin Definitions - Hardware Map**

```
const int IR_PINS = {13, 12, 14, 27, 26, 25, 33, 32};[1]
const int NUM_SENSORS = 8;
```

**What This Does:**
- Creates an **array** storing all 8 GPIO pin numbers
- `IR_PINS[0]` = GPIO 13 (sensor A1)
- `IR_PINS[1]` = GPIO 12 (sensor A2)
- `IR_PINS[7]` = GPIO 32 (sensor A8)

**Why Use an Array?**
- Easy to loop through all sensors
- Add/remove sensors by changing one line
- Centralized pin configuration

**How to Change Pins:**
```
const int IR_PINS = {15, 2, 4, 16, 17, 5, 18, 19};  // Different pins[1]
```

**Why NUM_SENSORS?**
- Used in loops: `for(int i = 0; i < NUM_SENSORS; i++)`
- If you switch to 6 sensors, change to `const int NUM_SENSORS = 6;`

---

### **2. Sensor Data Storage - The Memory**

```
int sensorValues;[1]
```

**What This Is:**
- A global array storing the **latest** readings from all 8 sensors
- Values range from **0 to 4095** (12-bit ADC)
- Updated every time `readIRSensors()` is called

**Example:**
```
readIRSensors();
Serial.println(sensorValues);  // Prints sensor A1 reading (e.g., 1523)
Serial.println(sensorValues);  // Prints sensor A8 reading (e.g., 2847)[2]
```

**Value Interpretation:**
- **0-800**: Very bright reflection (white surface)
- **800-1500**: Light gray surface
- **1500-2500**: Medium gray (threshold area)
- **2500-3500**: Dark gray / black line
- **3500-4095**: Very dark / strong black line

**Why Global?**
- Other files (Calibration.h, Linepos.h) need to read these values
- No need to pass arrays between functions

---

### **3. initIRSensors() - Setup Function**

```
void initIRSensors() {
  analogReadResolution(12);  // Set ADC to 12-bit mode
  Serial.println("IR Sensor Array Initialized (Analog Mode)");
}
```

**What This Does:**

**Step 1: Set ADC Resolution**
```
analogReadResolution(12);
```
- ESP32 ADC can be 9-bit, 10-bit, 11-bit, or **12-bit**
- 12-bit = 2^12 = **4096 possible values** (0-4095)
- Higher resolution = better precision

**Comparison:**
| Bits | Range | Precision |
|------|-------|-----------|
| 9-bit | 0-511 | Poor (512 steps) |
| 10-bit | 0-1023 | Okay (Arduino default) |
| 12-bit | 0-4095 | Best (ESP32 max) |

**Step 2: Print Confirmation**
- Helps debugging - confirms initialization happened
- Shows in Serial Monitor during startup

**Why No pinMode()?**
- ADC pins are **automatically configured** when you call `analogRead()`
- ESP32 handles this internally

**When to Call:**
```
void setup() {
  Serial.begin(115200);
  initIRSensors();  // Call once during setup
}
```

---

### **4. readIRSensors() - Data Acquisition**

```
void readIRSensors() {
  for(int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
  }
}
```

**What This Does:**

**Step-by-Step:**
1. Loop through all 8 sensors (i = 0 to 7)
2. Read analog voltage from each pin
3. Store result in `sensorValues` array

**How analogRead() Works:**
```
int value = analogRead(13);  // Read GPIO13
```
- Measures voltage on the pin (0V to 3.3V)
- Converts to digital number (0 to 4095)
- Formula: `ADC_value = (Voltage / 3.3V) * 4095`

**Example:**
```
Pin voltage = 1.65V
ADC value = (1.65 / 3.3) * 4095 = 2047 (middle value)
```

**Timing:**
- Each `analogRead()` takes ~10-20 microseconds
- Reading all 8 sensors takes ~80-160 microseconds
- Fast enough for 50Hz control loop (20ms per cycle)

**When to Call:**
```
void loop() {
  readIRSensors();  // Call every loop iteration
  // Now sensorValues[] contains fresh data
  int position = readLinePosition();  // Uses sensorValues[]
}
```

---

### **5. ADC Characteristics - Understanding the Numbers**

**ESP32 ADC Quirks:**

**ADC1 vs ADC2:**
- **ADC1** (GPIO 32, 33): Works anytime, WiFi-safe
- **ADC2** (GPIO 12, 13, 14, 25, 26, 27): Can't use with WiFi active

**Your Setup:** 6 sensors on ADC2, 2 on ADC1
- **Solution:** No WiFi in line follower = no problem!
- BLE uses different hardware (WiFi-safe)

**Non-linearity:**
- ESP32 ADC isn't perfectly linear
- Readings near 0V and 3.3V are less accurate
- Middle range (0.3V - 2.8V) is most reliable
- **Impact:** Minimal for line following (we use calibration anyway)

**Attenuation Settings:**
```
analogSetAttenuation(ADC_11db);  // 0-3.3V range (default)
// Other options:
// ADC_0db   → 0-1.1V range (more precise, smaller range)
// ADC_2_5db → 0-1.5V range
// ADC_6db   → 0-2.2V range
```
**Default (11db) is fine** for QTR sensors (output 0-3.3V)

---

### **6. Commented printSensorValues() - Debug Tool**

```
/*
void printSensorValues() {
  Serial.print("Sensors: ");
  for(int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");  // Tab separator
  }
  Serial.println();
}
*/
```

**Why Commented Out?**
- Kept for debugging but not used in final code
- Other modules print sensor data differently
- Uncomment when you need quick debugging

**How to Use:**
```
// 1. Uncomment the function
void printSensorValues() { ... }

// 2. Call in loop
void loop() {
  readIRSensors();
  printSensorValues();  // Shows sensor values in Serial Monitor
  delay(100);
}
```

**Example Output:**
```
Sensors: 456    489    501    2345   2567   523    478    490
Sensors: 445    478    495    2389   2601   518    469    485
```

**Interpretation:**
- Sensors 0-2: ~450-500 (white surface)
- Sensors 3-4: ~2300-2600 (black line!)
- Sensors 5-7: ~470-520 (white surface)
- **Conclusion:** Line is under sensors 3-4 (center)

---

## 🔧 How to Customize

### **Change Number of Sensors**

**For 6 Sensors:**
```
const int IR_PINS = {13, 12, 14, 27, 26, 25};[3]
const int NUM_SENSORS = 6;
int sensorValues;[3]
```

**For 16 Sensors:** (two arrays)
```
const int IR_PINS = {13,12,14,27,26,25,33,32,15,2,4,16,17,5,18,19};[4]
const int NUM_SENSORS = 16;
int sensorValues;[4]
```

### **Change Pin Assignments**

```
// Example: Use different GPIOs
const int IR_PINS = {34, 35, 36, 39, 32, 33, 25, 26};[1]
```

**Safe ADC1 Pins** (WiFi-compatible):
- GPIO 32, 33, 34, 35, 36, 39

**ADC2 Pins** (no WiFi):
- GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27

### **Add Digital Mode** (for QTR-8RC)

```
void readIRSensorsDigital() {
  for(int i = 0; i < NUM_SENSORS; i++) {
    pinMode(IR_PINS[i], INPUT);
    sensorValues[i] = digitalRead(IR_PINS[i]) ? 4095 : 0;
    // HIGH (1) = black line → 4095
    // LOW (0) = white surface → 0
  }
}
```

### **Add Averaging Filter** (noise reduction)

```
void readIRSensors() {
  for(int i = 0; i < NUM_SENSORS; i++) {
    int sum = 0;
    for(int j = 0; j < 4; j++) {  // Take 4 samples
      sum += analogRead(IR_PINS[i]);
      delayMicroseconds(100);
    }
    sensorValues[i] = sum / 4;  // Average
  }
}
```

---

## Troubleshooting

### **Problem: All sensors read 4095**
**Cause:** Sensors not powered, or wiring issue
**Solution:**
- Check VCC and GND connections
- Verify sensor array has power LED on
- Test individual sensor with multimeter

### **Problem: All sensors read 0**
**Cause:** Sensor output not connected
**Solution:**
- Check signal wire connections (A1-A8)
- Verify GPIO pins match code
- Test with multimeter (should see 0-3.3V)

### **Problem: Values don't change**
**Cause:** Sensors too far from surface, or broken
**Solution:**
- Adjust sensor height (optimal: 3-5mm above surface)
- Check IR LEDs are working (use phone camera - LEDs glow purple)
- Try different surface (glossy paper works best)

### **Problem: Noisy/jumpy readings**
**Cause:** Electrical noise, poor power supply
**Solution:**
- Add 100µF capacitor across VCC/GND
- Use shorter wires
- Enable averaging filter (see customization)

### **Problem: Readings reversed** (black=low, white=high)
**Cause:** Some sensor types output inverted signals
**Solution:**
```
void readIRSensors() {
  for(int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = 4095 - analogRead(IR_PINS[i]);  // Invert
  }
}
```

---

## Expected Values

### **Typical Readings (QTR-8A)**

**White matte paper:**
- Raw: 200-600
- Calibrated: 0-100

**Gray surface:**
- Raw: 1000-2000
- Calibrated: 400-600

**Black electrical tape:**
- Raw: 2500-3500
- Calibrated: 800-1000

**Very black line (sharpie):**
- Raw: 3000-4000
- Calibrated: 900-1000

### **Value Range by Surface**

| Surface | Reflection | Typical ADC Value |
|---------|-----------|-------------------|
| Mirror | 100% | 0-200 |
| White paper | 80% | 300-700 |
| Light gray | 50% | 1200-1800 |
| Dark gray | 30% | 2000-2500 |
| Black tape | 10% | 2800-3400 |
| Matte black | 5% | 3500-4000 |

---

## Key Concepts

### **ADC (Analog-to-Digital Converter)**
- Converts continuous voltage (0-3.3V) to discrete number (0-4095)
- ESP32 has two ADCs (ADC1 and ADC2)
- 12-bit resolution = 4096 distinct values

### **Resolution**
- Number of bits used for conversion
- More bits = finer precision
- 12-bit = 0.8mV per step (3300mV / 4095)

### **Reflectance Sensor**
- Measures light bouncing off surface
- Black absorbs light (low reflection)
- White reflects light (high reflection)

### **Array**
- Collection of similar items stored together
- `sensorValues[8]` = 8 integers in a row
- Access with index: `sensorValues[0]` to `sensorValues[7]`

### **Global Variable**
- Declared outside functions
- Accessible from anywhere in your program
- Lives for entire program lifetime

---

## Testing Checklist

**Hardware:**
- [ ] Sensor array has power (check LED)
- [ ] All 8 signal wires connected correctly
- [ ] Sensor height 3-5mm above surface
- [ ] VCC connected to correct voltage (3.3V or 5V)

**Software:**
- [ ] `initIRSensors()` called in setup()
- [ ] Serial Monitor shows initialization message
- [ ] `readIRSensors()` called in loop()
- [ ] Can print sensor values to Serial Monitor

**Calibration:**
- [ ] White surface gives low values (200-700)
- [ ] Black line gives high values (2500-3800)
- [ ] Good contrast (difference > 1500)
- [ ] All 8 sensors respond to surfaces

---

## Pro Tips

1. **Check with phone camera** - IR LEDs should glow purple/pink
2. **Optimal height** - 3-5mm above surface for best contrast
3. **Use matte surfaces** - glossy surfaces cause reflections
4. **Test indoors first** - sunlight contains IR (interferes)
5. **Add capacitor** - 100µF across VCC/GND reduces noise
6. **Print raw values** - helps diagnose sensor issues
7. **Calibrate often** - lighting changes affect readings

---

## Integration with Other Modules

### **Provides Data To:**
- `Calibration.h` - Uses `sensorValues[]` for min/max tracking
- `Linepos.h` - Uses `sensorValues[]` for position calculation
- `main.cpp` - Calls `readIRSensors()` every loop

### **Depends On:**
- Arduino ADC functions (`analogRead`)
- ESP32 hardware (ADC1/ADC2 peripherals)

### **Data Flow:**
```
Physical Surface (black/white)
        ↓
IR Sensor Array (emits/receives light)
        ↓
Analog Voltage (0-3.3V on GPIO pins)
        ↓
ESP32 ADC (converts to 0-4095)
        ↓
readIRSensors() (stores in sensorValues[])
        ↓
Other Modules (use sensorValues[] for calculations)
```

---

## Quick Test Code

```
#include "IR/IRSensor.h"

void setup() {
  Serial.begin(115200);
  initIRSensors();
}

void loop() {
  readIRSensors();
  
  // Print all 8 values in one line
  for(int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
  
  delay(100);  // 10Hz update rate
}
```

**Expected Output:**
```
456   478   501   2345  2567  523   478   490
445   469   495   2389  2601  518   465   485
```

**Test Procedure:**
1. Upload code
2. Open Serial Monitor (115200 baud)
3. Move sensors over white paper → values should be LOW (200-700)
4. Move sensors over black line → values should be HIGH (2500-3800)
5. All 8 sensors should respond

---