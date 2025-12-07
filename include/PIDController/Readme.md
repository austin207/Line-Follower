# PIDController.h - Dynamic PID Management Layer

## What Does This File Do?

This file is a **wrapper** around the PID algorithm that adds **real-time parameter updates** from BLE. It manages the lifecycle of the PID controller and handles dynamic tuning without manual code uploads.

Think of it as a **smart PID manager** - it automatically updates PID settings when you change values from your phone!

---

## Why Do We Need This?

### **The Problem: PID.h Alone Isn't Enough**

**PID.h provides:**
- Pure PID calculation algorithm ✓
- Fixed gains set at initialization ✓

**But we need:**
- Update gains wirelessly via BLE ❌
- Change output limits dynamically ❌
- Track current state for debugging ❌
- Handle line-lost scenarios ❌

### **The Solution: PIDController Wrapper**

**PIDController.h adds:**
- BLE integration (read dynamic parameters)
- PID recreation when limits change
- State tracking (position, correction)
- Debug printing
- Line-lost handling

**Analogy:**
- **PID.h** = Calculator (does math)
- **PIDController.h** = Smart assistant (manages calculator + handles updates)

---

## Architecture Pattern

### **Composition Pattern**

```
PIDController (Manager)
    ↓
   owns
    ↓
  PID (Worker)
```

**PIDController:**
- Has-a PID object (composition)
- Manages PID lifecycle
- Handles parameter updates
- Provides convenience functions

**Why Not Inheritance?**
- PID is a complete unit (shouldn't be extended)
- Controller adds different functionality (not specialization)
- Easier to swap PID implementations

---

## Code Breakdown

### **1. Class Structure - Private Members**

```
class PIDController {
private:
    PID* pid;              // Pointer to PID object
    int lastMinOut;        // Previous min output limit
    int lastMaxOut;        // Previous max output limit
    int currentCorrection; // Latest PID output
    int currentPosition;   // Latest line position
```

**Understanding Pointers:**

**PID* pid;** - Pointer to PID object
- `*` means "pointer" (stores memory address, not object itself)
- Allows dynamic creation/destruction of PID
- Enables `new` and `delete` operations

**Why Use Pointer Instead of Direct Object?**

**Option 1: Direct object (what we DON'T do)**
```
PID pid;  // Fixed at compile time, can't recreate
```
- Can't change output limits (requires new PID object)
- Can't delete and recreate

**Option 2: Pointer (what we DO)**
```
PID* pid;  // Can point to different PID objects
pid = new PID(...);  // Create new
delete pid;          // Destroy old
pid = new PID(...);  // Create new with different params
```
- Flexible: Can recreate with new limits
- Dynamic: Allocates memory at runtime

**State Variables:**
- `lastMinOut`, `lastMaxOut`: Track previous limits (detect changes)
- `currentCorrection`: Last calculated correction (for debugging)
- `currentPosition`: Last line position (for debugging)

---

### **2. Constructor - Initialization**

```
PIDController() {
    // Create initial PID instance with BLE defaults
    pid = new PID(bleKp, bleKi, bleKd, bleMinOutput, bleMaxOutput);
    lastMinOut = bleMinOutput;
    lastMaxOut = bleMaxOutput;
    currentCorrection = 0;
    currentPosition = 0;
}
```

**Step-by-Step:**

**Step 1: Create PID Object**
```
pid = new PID(bleKp, bleKi, bleKd, bleMinOutput, bleMaxOutput);
```
- `new` allocates memory on heap
- Calls PID constructor with BLE default values
- Returns pointer to new PID object
- `bleKp`, `bleKi`, etc. come from `BLEConfig.h`

**Step 2: Remember Initial Limits**
```
lastMinOut = bleMinOutput;  // Usually -255
lastMaxOut = bleMaxOutput;  // Usually +255
```
- Stores current limits for change detection
- Used in `update()` to detect if recreation needed

**Step 3: Initialize State**
```
currentCorrection = 0;
currentPosition = 0;
```
- No correction initially
- Position unknown initially

**When Called:**
```
PIDController pidController;  // Constructor runs automatically
```

---

### **3. Destructor - Cleanup**

```
~PIDController() {
    delete pid;
}
```

**What is a Destructor?**
- Function that runs automatically when object is destroyed
- Name starts with `~` (tilde)
- Cleans up allocated memory

**Why Needed?**
- We used `new` to allocate PID → must use `delete` to free memory
- Without destructor → **memory leak** (memory never freed)

**When Called:**
```
{
    PIDController controller;  // Constructor
    // Use controller...
}  // Scope ends → Destructor runs → delete pid
```

**Memory Management:**
```
Constructor: new PID     → Allocate memory
Destructor:  delete pid  → Free memory
```

---

### **4. update() - The Core Function**

```
int update(int linePosition) {
    currentPosition = linePosition;
    
    // Get latest parameters from BLE
    float kp, ki, kd;
    int minOut, maxOut;
    getCurrentPIDParams(&kp, &ki, &kd, &minOut, &maxOut);
    
    // Check if output limits changed
    if (minOut != lastMinOut || maxOut != lastMaxOut) {
        delete pid;  // Destroy old PID
        pid = new PID(kp, ki, kd, minOut, maxOut);  // Create new
        lastMinOut = minOut;
        lastMaxOut = maxOut;
        Serial.println("PID output limits updated - controller reset");
    } else {
        // Just update tunings if only Kp, Ki, Kd changed
        pid->setTunings(kp, ki, kd);
    }
    
    // Calculate correction
    if (linePosition != -999) {
        currentCorrection = pid->compute(0, linePosition);
    } else {
        currentCorrection = 0;  // Line lost
    }
    
    return currentCorrection;
}
```

**Step-by-Step Walkthrough:**

---

**Step 1: Save Current Position**
```
currentPosition = linePosition;
```
- Stores for later debugging/printing
- Available via `getPosition()`

---

**Step 2: Fetch Latest BLE Parameters**
```
float kp, ki, kd;
int minOut, maxOut;
getCurrentPIDParams(&kp, &ki, &kd, &minOut, &maxOut);
```

**What This Does:**
- Calls function from `BLEConfig.h`
- Reads global variables (`bleKp`, `bleKi`, etc.)
- Returns all 5 parameters via pointers

**Example:**
```
// Before (in BLEConfig.h):
bleKp = 8.0
bleKi = 0.0
bleKd = 80.0
bleMinOutput = -255
bleMaxOutput = 255

// After:
kp = 8.0
ki = 0.0
kd = 80.0
minOut = -255
maxOut = 255
```

**Pointer Parameters (`&` symbol):**
```
getCurrentPIDParams(&kp, &ki, &kd, &minOut, &maxOut);
```
- `&kp` means "address of kp"
- Function can modify these variables directly
- Returns multiple values (C++ doesn't support multiple returns normally)

---

**Step 3: Check if Recreation Needed**
```
if (minOut != lastMinOut || maxOut != lastMaxOut) {
    // Limits changed → recreate PID
}
```

**Why Recreate?**
- PID output limits set in constructor
- No function to change limits after creation
- Must create new PID object with new limits

**When This Happens:**
- User changes MinOutput via BLE (e.g., -255 → -200)
- User changes MaxOutput via BLE (e.g., 255 → 200)
- Different speed modes requiring different limits

---

**Step 4A: Recreate PID (If Limits Changed)**
```
delete pid;  // Free old PID memory
pid = new PID(kp, ki, kd, minOut, maxOut);  // Create new
lastMinOut = minOut;   // Update tracked values
lastMaxOut = maxOut;
Serial.println("PID output limits updated - controller reset");
```

**What Happens:**
1. **Delete old PID**: Frees memory, destroys object
2. **Create new PID**: Allocates new memory with new limits
3. **Update tracking**: Remember new limits
4. **Print message**: Confirm recreation happened

**Side Effect:**
- PID state is **reset** (integral=0, derivative=0)
- Like pressing "reset" button
- Fresh start with new limits

---

**Step 4B: Just Update Tunings (If Only Gains Changed)**
```
} else {
    pid->setTunings(kp, ki, kd);
}
```

**Why This Path?**
- Limits unchanged, only Kp/Ki/Kd changed
- No need to recreate entire PID
- Call `setTunings()` to update gains
- **Preserves PID state** (integral, derivative kept)

**Comparison:**

| Scenario | Action | State Preserved? |
|----------|--------|------------------|
| Kp changes | `setTunings()` | Yes ✓ |
| Ki changes | `setTunings()` | Yes ✓ |
| Kd changes | `setTunings()` | Yes ✓ |
| MinOutput changes | `delete` + `new` | No (reset) |
| MaxOutput changes | `delete` + `new` | No (reset) |

---

**Step 5: Calculate Correction**
```
if (linePosition != -999) {
    currentCorrection = pid->compute(0, linePosition);
} else {
    currentCorrection = 0;  // Line lost
}
```

**Normal Operation (line detected):**
```
currentCorrection = pid->compute(0, linePosition);
```
- Setpoint = 0 (want to be centered)
- currentPosition = linePosition
- PID calculates correction
- Example: position=-30 → correction=+120 (turn right)

**Line Lost (position = -999):**
```
currentCorrection = 0;
```
- Don't apply any correction
- Motors will stop in `main.cpp`
- Prevents wild corrections when no line

---

**Step 6: Return Correction**
```
return currentCorrection;
```
- Returns calculated correction to caller
- Typically `main.cpp` applies this to motors

**Usage:**
```
int correction = pidController.update(position);
applyPIDCorrection(correction);
```

---

### **5. Getter Functions - State Access**

**getCorrection() - Read Last Correction**
```
int getCorrection() {
    return currentCorrection;
}
```

**When to Use:**
- Debugging: Check what correction PID calculated
- Logging: Record correction values
- Testing: Verify PID output

**Example:**
```
int corr = pidController.getCorrection();
Serial.print("Last correction: ");
Serial.println(corr);
```

---

**getPosition() - Read Last Position**
```
int getPosition() {
    return currentPosition;
}
```

**When to Use:**
- Debugging: Verify position calculation
- Display: Show on LCD/OLED
- Logging: Track robot path

**Example:**
```
int pos = pidController.getPosition();
Serial.print("Current position: ");
Serial.println(pos);
```

---

### **6. reset() - Clear PID State**

```
void reset() {
    pid->reset();
}
```

**What This Does:**
- Calls `reset()` on underlying PID object
- Clears integral accumulation
- Resets derivative
- Resets previousError

**When to Use:**
- Robot picked up and repositioned
- Starting new run
- Line lost for extended time
- Want fresh PID state

**Example:**
```
if (linePosition == -999) {
    pidController.reset();  // Clear state when line lost
}
```

---

### **7. printDebug() - Status Display**

```
void printDebug() {
    float kp, ki, kd;
    int minOut, maxOut;
    getCurrentPIDParams(&kp, &ki, &kd, &minOut, &maxOut);
    
    Serial.print("Pos: ");
    Serial.print(currentPosition);
    Serial.print(" | PID[");
    Serial.print(kp, 2);
    Serial.print(", ");
    Serial.print(ki, 2);
    Serial.print(", ");
    Serial.print(kd, 2);
    Serial.print("] Out[");
    Serial.print(minOut);
    Serial.print(", ");
    Serial.print(maxOut);
    Serial.print("] | Corr: ");
    Serial.println(currentCorrection);
}
```

**What This Prints:**
```
Pos: -30 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 120
```

**Breakdown:**
- **Pos: -30** → Line position (left of center)
- **PID[8.00, 0.00, 80.00]** → Current Kp, Ki, Kd values
- **Out[-255, 255]** → Output limits
- **Corr: 120** → Calculated correction (turn right)

**When to Use:**
- Debugging PID behavior
- Tuning (see how parameters affect correction)
- Monitoring during operation

**Example Output Stream:**
```
Pos: -30 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 120
Pos: -15 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 80
Pos: -5 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 30
Pos: 0 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 5
```

**Interpretation:**
- Position converging to 0 (centered)
- Correction decreasing as error reduces
- PID working correctly!

---

## Update Flow Diagram

```
BLE App (Phone)
  ↓
User changes Kp to 12.0
  ↓
BLEConfig.h updates bleKp = 12.0
  ↓
main.cpp calls pidController.update(position)
  ↓
PIDController.update():
  ├─ Read BLE params (kp=12.0, ki=0.0, kd=80.0)
  ├─ Check limits changed? No
  ├─ Call pid->setTunings(12.0, 0.0, 80.0)
  ├─ Call pid->compute(0, position)
  └─ Return correction
  ↓
main.cpp applies correction to motors
  ↓
Robot steers with NEW Kp value!
```

**Key Points:**
- No code reupload needed
- Changes apply immediately (next loop)
- State preserved (integral/derivative kept)

---

## How to Customize

### **Add Auto-Tuning** (Advanced)

```
void autoTune() {
    // Relay feedback auto-tuning
    // Increases Kp until oscillation detected
    // Calculates optimal Kp, Ki, Kd
    // (Implementation requires 50+ lines)
}
```

### **Add PID Component Logging**

```
void printComponents() {
    float p, i, d;
    pid->getComponents(&p, &i, &d);
    Serial.print("P: "); Serial.print(p);
    Serial.print(" I: "); Serial.print(i);
    Serial.print(" D: "); Serial.println(d);
}
```

### **Add Adaptive Gains** (Speed-Dependent)

```
int update(int linePosition, int currentSpeed) {
    float kp, ki, kd;
    getCurrentPIDParams(&kp, &ki, &kd, ...);
    
    // Scale gains based on speed
    float speedFactor = currentSpeed / 150.0;  // Normalize to base speed
    kp *= speedFactor;
    kd *= speedFactor;
    
    pid->setTunings(kp, ki, kd);
    // ... rest of update
}
```

### **Add Error Rate Limiting**

```
int update(int linePosition) {
    // Limit how fast position can change (reject spikes)
    static int lastPosition = 0;
    int maxChange = 50;
    
    if (abs(linePosition - lastPosition) > maxChange) {
        linePosition = lastPosition + maxChange * sign(linePosition - lastPosition);
    }
    
    lastPosition = linePosition;
    // ... rest of update
}
```

---

## Troubleshooting

### **Problem: "PID output limits updated" prints constantly**
**Cause:** Output limits oscillating between values
**Solution:**
- Check BLE app for spurious updates
- Add hysteresis (only recreate if change > threshold)
```
if (abs(minOut - lastMinOut) > 5 || abs(maxOut - lastMaxOut) > 5) {
    // Recreate only if change significant
}
```

### **Problem: PID reset unexpectedly**
**Cause:** Output limits changed (even slightly)
**Solution:**
- Avoid changing limits during operation
- Set limits once at startup
- Use `setTunings()` for dynamic changes

### **Problem: Correction always 0**
**Cause:** Line position = -999 (line lost)
**Solution:**
- Check line detection threshold
- Verify sensors calibrated
- Test position calculation separately

### **Problem: Correction doesn't change with BLE updates**
**Cause:** BLE parameters not updating
**Solution:**
- Check BLE connection status
- Verify `getCurrentPIDParams()` reads BLE globals
- Print kp, ki, kd in `update()` to debug

---

## Example Usage Scenarios

### **Scenario 1: Normal Operation**

```
// In main.cpp
PIDController pidController;

void loop() {
    readIRSensors();
    int position = readLinePosition();
    
    int correction = pidController.update(position);
    applyPIDCorrection(correction);
    
    pidController.printDebug();
    delay(20);
}
```

**Output:**
```
Pos: -20 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 85
Pos: -10 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 45
Pos: 0 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 2
```

---

### **Scenario 2: BLE Parameter Update (Gains Only)**

```
User changes Kp from 8.0 to 12.0 via BLE app
```

**What Happens:**
1. BLEConfig.h: `bleKp = 12.0`
2. `update()` reads new value
3. Limits unchanged → `setTunings(12.0, 0.0, 80.0)`
4. PID state preserved
5. Next correction uses Kp=12.0

**Output:**
```
Pos: -20 | PID[12.00, 0.00, 80.00] Out[-255, 255] | Corr: 128
```
(Larger correction due to higher Kp)

---

### **Scenario 3: BLE Parameter Update (Limits Changed)**

```
User changes MaxOutput from 255 to 200 via BLE app
```

**What Happens:**
1. BLEConfig.h: `bleMaxOutput = 200`
2. `update()` reads new value
3. Limits changed → `delete pid; pid = new PID(...)`
4. PID state **reset** (integral=0, derivative=0)
5. "PID output limits updated" printed

**Output:**
```
PID output limits updated - controller reset
Pos: -20 | PID[8.00, 0.00, 80.00] Out[-255, 200] | Corr: 85
```
(MaxOutput now limited to 200)

---

### **Scenario 4: Line Lost**

```
Robot goes off track, position = -999
```

**What Happens:**
1. `update(-999)` called
2. `if (linePosition != -999)` → false
3. `currentCorrection = 0`
4. Motors stop (main.cpp logic)

**Output:**
```
Pos: -999 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 0
```

---

## Key Concepts

### **Wrapper Pattern**
- Object that "wraps" another object
- Adds functionality without modifying original
- Delegation: forwards calls to wrapped object

### **Dynamic Memory Allocation**
- `new`: Allocate memory at runtime (heap)
- `delete`: Free allocated memory
- Pointers: Store memory addresses

### **Pointer vs Reference**
- `PID* pid`: Pointer (can be null, reassigned)
- `PID& pid`: Reference (must be initialized, can't reassign)
- Pointers allow `new`/`delete`, references don't

### **Destructor**
- Automatic cleanup function
- Runs when object destroyed
- Must `delete` what was `new`ed

### **State Management**
- Track current values (position, correction)
- Detect changes (compare to previous)
- React to changes (recreate vs update)

---

## Testing Checklist

**Initialization:**
- [ ] `PIDController` creates successfully
- [ ] Internal `PID` object allocated
- [ ] Default BLE values loaded

**Update Function:**
- [ ] `update()` returns correction
- [ ] BLE parameter changes applied
- [ ] Limits change triggers recreation
- [ ] Gain change uses `setTunings()`
- [ ] Line lost → correction = 0

**State Tracking:**
- [ ] `getPosition()` returns last position
- [ ] `getCorrection()` returns last correction
- [ ] Values update each loop

**Debug:**
- [ ] `printDebug()` shows all parameters
- [ ] Output formatted correctly

**Memory:**
- [ ] No memory leaks (destructor frees PID)
- [ ] Recreation doesn't crash

---

## Pro Tips

1. **Use printDebug()** - essential for tuning and debugging
2. **Monitor recreation** - shouldn't happen often (only on limit changes)
3. **Preserve state when possible** - use `setTunings()` over recreation
4. **Test BLE updates** - verify parameters apply in real-time
5. **Check line-lost handling** - ensure correction=0 when position=-999
6. **Watch for memory leaks** - destructor must `delete pid`
7. **Log PID components** - add `getComponents()` logging for deep debugging

---

## Integration with Other Modules

### **Uses:**
- `PID.h` - Core algorithm (composition)
- `BLEConfig.h` - Parameter source (`getCurrentPIDParams()`)
- `Linepos.h` - Position input (indirectly via main.cpp)

### **Used By:**
- `main.cpp` - Creates instance, calls `update()`

### **Data Flow:**
```
BLE App
  ↓
BLEConfig.h (bleKp, bleKi, bleKd, bleMinOutput, bleMaxOutput)
  ↓
PIDController.update()
  ├─ getCurrentPIDParams() → read BLE values
  ├─ Check if recreation needed
  ├─ pid->setTunings() or new PID()
  └─ pid->compute() → calculate correction
  ↓
Return correction
  ↓
main.cpp → MotorControl.h → Motors
```

---

## Quick Test Code

```
#include "PIDController/PIDController.h"
#include "BLE/BLEConfig.h"

PIDController testController;

void setup() {
  Serial.begin(115200);
  initBLE();  // Initialize BLE (provides bleKp, etc.)
}

void loop() {
  // Simulate line positions
  int positions[] = {-50, -30, -10, 0, 10, 30, 50};
  
  for (int pos : positions) {
    int correction = testController.update(pos);
    testController.printDebug();
    delay(500);
  }
  
  // Test BLE update (change Kp)
  Serial.println("\n=== Changing Kp to 15.0 ===");
  bleKp = 15.0;
  
  int correction = testController.update(-20);
  testController.printDebug();
  
  delay(5000);
}
```

**Expected Output:**
```
Pos: -50 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: -255
Pos: -30 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: -240
Pos: -10 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: -80
Pos: 0 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 0
Pos: 10 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 80
Pos: 30 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 240
Pos: 50 | PID[8.00, 0.00, 80.00] Out[-255, 255] | Corr: 255

=== Changing Kp to 15.0 ===
Pos: -20 | PID[15.00, 0.00, 80.00] Out[-255, 255] | Corr: -255
```

---