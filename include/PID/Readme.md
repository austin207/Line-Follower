# PID.h - PID Controller Algorithm

## What Does This File Do?

This file implements a **PID (Proportional-Integral-Derivative) controller** - a mathematical algorithm that automatically corrects errors to keep your robot centered on the line.

Think of it as a **self-correcting autopilot** - it continuously calculates how much to steer based on how far off-track you are!

---

## Why Do We Need PID?

### **The Problem: Simple Control Doesn't Work**

**Attempt 1: On/Off Control** (Bang-Bang)
```
if (line is left) {
    turn left at max speed;
} else if (line is right) {
    turn right at max speed;
}
```
**Result:** Robot oscillates wildly, overshoots constantly ❌

**Attempt 2: Proportional Only**
```
correction = Kp * error;
```
**Result:** Better, but still overshoots and has steady-state error ❌

### **The Solution: PID Control**

PID combines **three** types of corrections:
1. **P (Proportional):** React to current error
2. **I (Integral):** Fix accumulated past errors
3. **D (Derivative):** Predict future errors

**Result:** Smooth, accurate line following with minimal oscillation ✓

---

## Understanding PID Components

### **Visual Analogy: Driving a Car**

**Scenario:** You're slightly drifting right on the highway.

**P (Proportional):** "I'm 2 feet off-center → steer left proportionally"
- Far off → steer a lot
- Slightly off → steer a little
- On-center → don't steer

**I (Integral):** "I've been drifting right for 10 seconds → add extra left correction"
- Eliminates persistent drift
- Compensates for wind, road slope, etc.

**D (Derivative):** "I'm drifting right faster now → steer harder to stop the drift"
- Dampens oscillations
- Prevents overshooting
- Adds "smoothness"

---

## The Math Behind PID

### **PID Formula**

```
output(t) = Kp×e(t) + Ki×∫e(t)dt + Kd×(de(t)/dt)
```

**In Plain English:**
```
Correction = (Kp × error) + (Ki × sum of errors) + (Kd × rate of change)
```

### **Breaking It Down**

**1. Proportional Term**
```
P = Kp × error
```
- **error** = setpoint - current position
- **Kp** = how aggressively to respond (gain)
- Larger error → larger correction

**Example:**
```
Setpoint = 0 (centered)
Current position = -50 (left)
error = 0 - (-50) = 50
Kp = 2.0

P = 2.0 × 50 = 100  (strong right correction)
```

**2. Integral Term**
```
I = Ki × ∫error dt
  = Ki × (error₁ + error₂ + error₃ + ...)
```
- **∫** = summation over time (integral)
- **Ki** = how much past errors matter
- Accumulates error over time

**Example:**
```
Previous errors:[1][2][3][4]
integral = 10 + 15 + 12 + 8 = 45
Ki = 0.1

I = 0.1 × 45 = 4.5  (small correction for drift)
```

**3. Derivative Term**
```
D = Kd × (error - previousError) / Δt
```
- **Δt** = time since last calculation
- **Kd** = how much to dampen oscillations
- Responds to rate of change

**Example:**
```
Current error = 20
Previous error = 50
Δt = 0.02 seconds
Kd = 1.5

D = 1.5 × (20 - 50) / 0.02
  = 1.5 × (-1500)
  = -2250  (big correction because error decreasing fast)
```

**Total Output:**
```
output = P + I + D
       = 100 + 4.5 + (-2250)
       = -2145.5
       → constrained to -255 to +255
       = -255  (max left correction)
```

---

## Code Breakdown

### **1. Class Structure - Object-Oriented Design**

```
class PID {
private:
    // Private variables (internal use only)
    
public:
    // Public functions (can be called from outside)
};
```

**Why Use a Class?**
- Encapsulation: All PID data in one object
- Reusability: Create multiple PIDs if needed
- Clean interface: Hide complexity

**Example:**
```
PID linePID(8.0, 0.0, 80.0, -255, 255);  // Create PID object
int correction = linePID.compute(0, position);  // Use it
```

---

### **2. Private Variables - Internal State**

```
private:
    float Kp;  // Proportional gain
    float Ki;  // Integral gain  
    float Kd;  // Derivative gain
    
    float previousError;
    float integral;
    float derivative;
    
    unsigned long lastTime;
    
    int minOutput;
    int maxOutput;
```

**PID Constants (Kp, Ki, Kd):**
- Set during initialization
- Can be changed via `setTunings()`
- These are the **tuning parameters** you adjust

**State Variables:**
- `previousError`: Error from last iteration (for derivative)
- `integral`: Accumulated sum of errors (for integral)
- `derivative`: Rate of error change (for derivative)

**Timing:**
- `lastTime`: Timestamp of last calculation (milliseconds)
- Used to calculate Δt (time delta)

**Output Limits:**
- `minOutput`: Minimum correction (-255 typical)
- `maxOutput`: Maximum correction (+255 typical)

---

### **3. Constructor - Initialization**

```
PID(float kp, float ki, float kd, int minOut, int maxOut) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    minOutput = minOut;
    maxOutput = maxOut;
    
    previousError = 0;
    integral = 0;
    derivative = 0;
    lastTime = millis();
}
```

**What This Does:**
- Sets PID gains (Kp, Ki, Kd)
- Sets output limits
- Initializes state variables to zero
- Records current time

**Usage:**
```
// PID(Kp, Ki, Kd, minOutput, maxOutput)
PID controller(10.0, 0.5, 50.0, -255, 255);
```

**Parameter Guide:**
| Parameter | Typical Range | Effect |
|-----------|---------------|--------|
| Kp | 1-20 | Higher = more aggressive response |
| Ki | 0-1 | Higher = faster drift elimination |
| Kd | 10-200 | Higher = more damping (smoothness) |
| minOutput | -255 | Minimum motor correction |
| maxOutput | +255 | Maximum motor correction |

---

### **4. compute() - The Main Algorithm**

```
int compute(int setpoint, int currentPosition) {
    // Calculate error
    int error = setpoint - currentPosition;
    
    // Calculate time delta
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    
    // Prevent division by zero
    if (deltaTime <= 0) deltaTime = 0.001;
    
    // P term
    float P = Kp * error;
    
    // I term
    integral += error * deltaTime;
    
    // Anti-windup
    float maxIntegral = maxOutput / Ki;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    
    float I = Ki * integral;
    
    // D term
    derivative = (error - previousError) / deltaTime;
    float D = Kd * derivative;
    
    // Total output
    float output = P + I + D;
    
    // Constrain output
    output = constrain(output, minOutput, maxOutput);
    
    // Update state
    previousError = error;
    lastTime = currentTime;
    
    return (int)output;
}
```

**Step-by-Step Walkthrough:**

---

**Step 1: Calculate Error**
```
int error = setpoint - currentPosition;
```

**Example:**
```
Setpoint = 0 (want to be centered)
Current position = -30 (line is left)
error = 0 - (-30) = 30  (positive = need to go right)
```

**Sign Convention:**
- Positive error → line is LEFT → turn RIGHT
- Negative error → line is RIGHT → turn LEFT
- Zero error → centered → no correction

---

**Step 2: Calculate Time Delta**
```
unsigned long currentTime = millis();
float deltaTime = (currentTime - lastTime) / 1000.0;
```

**Example:**
```
lastTime = 1000 ms
currentTime = 1020 ms
deltaTime = (1020 - 1000) / 1000.0 = 0.02 seconds
```

**Why Time Delta?**
- Integral and derivative need time intervals
- Handles variable loop speeds
- Makes PID time-independent

**Safety Check:**
```
if (deltaTime <= 0) deltaTime = 0.001;
```
- Prevents division by zero
- Happens if `compute()` called twice in same millisecond

---

**Step 3: Proportional Term**
```
float P = Kp * error;
```

**Example:**
```
Kp = 8.0
error = 30
P = 8.0 × 30 = 240
```

**What This Means:**
- Line is 30 units left
- Apply 240 units of right correction
- Larger Kp → stronger response

---

**Step 4: Integral Term**
```
integral += error * deltaTime;
```

**Example:**
```
Previous integral = 5.0
error = 30
deltaTime = 0.02
integral = 5.0 + (30 × 0.02) = 5.6
```

**Then:**
```
float I = Ki * integral;
```

**Example:**
```
Ki = 0.1
integral = 5.6
I = 0.1 × 5.6 = 0.56
```

**What This Does:**
- Accumulates error over time
- Small constant offset → integral grows → adds correction
- Eliminates steady-state error

---

**Step 5: Anti-Windup (Critical!)**
```
float maxIntegral = maxOutput / Ki;
if (integral > maxIntegral) integral = maxIntegral;
if (integral < -maxIntegral) integral = -maxIntegral;
```

**Why Anti-Windup?**

**Problem:**
- Robot stuck (line lost)
- Error stays large for long time
- Integral keeps growing → **windup**
- When line found again, huge overcorrection!

**Solution:**
- Limit integral to reasonable value
- `maxIntegral = maxOutput / Ki`
- Prevents massive corrections

**Example:**
```
maxOutput = 255
Ki = 0.1
maxIntegral = 255 / 0.1 = 2550

If integral = 5000:  → clamp to 2550
If integral = -3000: → clamp to -2550
```

---

**Step 6: Derivative Term**
```
derivative = (error - previousError) / deltaTime;
float D = Kd * derivative;
```

**Example:**
```
error = 30
previousError = 50
deltaTime = 0.02
Kd = 80.0

derivative = (30 - 50) / 0.02 = -1000
D = 80.0 × (-1000) = -80000  (huge!)
```

**What This Means:**
- Error is decreasing fast (50 → 30)
- Apply negative correction (reduce turning)
- Prevents overshooting

**Why Kd is Large:**
- Derivative values are small (rate of change)
- Need high Kd to make impact
- Typical: Kd ≈ 10× Kp

---

**Step 7: Calculate Total Output**
```
float output = P + I + D;
```

**Example:**
```
P = 240
I = 0.56
D = -80000

output = 240 + 0.56 + (-80000) = -79759.44
```

---

**Step 8: Constrain Output**
```
output = constrain(output, minOutput, maxOutput);
```

**Example:**
```
output = -79759.44
constrain to [-255, 255]
output = -255  (max left correction)
```

**Why Constrain?**
- Motors only accept -255 to +255
- Prevents overflow
- Limits extreme corrections

---

**Step 9: Update State Variables**
```
previousError = error;
lastTime = currentTime;
```

**Why:**
- Save current error for next derivative calculation
- Update timestamp for next time delta
- Prepare for next iteration

---

**Step 10: Return Integer Output**
```
return (int)output;
```

**Example:**
```
output = -79.6
(int)output = -79
```

**Why Cast to Int?**
- Motor functions expect integers
- PWM is 0-255 (no decimals)

---

### **5. reset() - Clear PID State**

```
void reset() {
    previousError = 0;
    integral = 0;
    derivative = 0;
    lastTime = millis();
}
```

**When to Use:**
- Robot picked up and repositioned
- Starting a new run
- Line lost for extended period
- Want to clear accumulated integral

**Example:**
```
if (linePosition == -999) {
    linePID.reset();  // Clear state when line lost
}
```

---

### **6. setTunings() - Update PID Gains**

```
void setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}
```

**When to Use:**
- Adjusting PID via BLE
- Different track conditions
- Speed changes (may need different gains)

**Example:**
```
linePID.setTunings(12.0, 0.2, 100.0);  // Update all three gains
```

**Note:** Does NOT reset state (integral, derivative preserved)

---

### **7. getComponents() - Debug Helper**

```
void getComponents(float* p, float* i, float* d) {
    *p = Kp * previousError;
    *i = Ki * integral;
    *d = Kd * derivative;
}
```

**When to Use:**
- Debugging PID behavior
- Seeing which term dominates
- Tuning optimization

**Example:**
```
float p, i, d;
linePID.getComponents(&p, &i, &d);
Serial.print("P: "); Serial.print(p);
Serial.print(" I: "); Serial.print(i);
Serial.print(" D: "); Serial.println(d);
```

**Output:**
```
P: 240  I: 5.6  D: -80000
→ D term is dominating (too high Kd?)
```

---

## PID Tuning Guide

### **The Ziegler-Nichols Method** (Classic Approach)

**Step 1: Start with P Only**
```
PID controller(1.0, 0.0, 0.0, -255, 255);  // Ki=0, Kd=0
```

**Step 2: Increase Kp Until Oscillation**
- Slowly increase Kp
- Watch robot oscillate side-to-side
- Find Kp where oscillations are constant (not growing/shrinking)
- Call this **Ku** (ultimate gain)

**Step 3: Measure Oscillation Period**
- Time how long one complete oscillation takes
- Call this **Tu** (oscillation period in seconds)

**Step 4: Calculate PID Values**
```
Kp = 0.6 × Ku
Ki = 1.2 × Ku / Tu
Kd = 0.075 × Ku × Tu
```

**Example:**
```
Ku = 15 (oscillates at Kp=15)
Tu = 0.5 seconds

Kp = 0.6 × 15 = 9.0
Ki = 1.2 × 15 / 0.5 = 36
Kd = 0.075 × 15 × 0.5 = 0.56
```

---

### **Manual Tuning** (Trial-and-Error)

**Step 1: Start Conservative**
```
PID controller(5.0, 0.0, 50.0, -255, 255);
```

**Step 2: Tune P**
- Increase Kp until oscillation starts
- Back off 20-30%
- **Good Kp:** Responds quickly, slight oscillation

**Step 3: Add D**
- Increase Kd to dampen oscillations
- Rule of thumb: Kd ≈ 10× Kp
- **Good Kd:** Smooth following, no overshoot

**Step 4: Add I (Optional)**
- Start with Ki = 0
- Add small Ki (0.01-0.1) if robot drifts
- **Good Ki:** No steady-state error

**Typical Values:**
```
// Slow, smooth
PID(5.0, 0.0, 50.0, -255, 255);

// Medium (default)
PID(8.0, 0.0, 80.0, -255, 255);

// Fast, aggressive
PID(15.0, 0.1, 150.0, -255, 255);
```

---

### **Effect of Each Gain**

| Gain | Too Low | Just Right | Too High |
|------|---------|------------|----------|
| **Kp** | Slow response, large error | Quick response, small overshoot | Oscillation, instability |
| **Ki** | Steady-state error | Eliminates drift | Windup, overshoot |
| **Kd** | Oscillation, overshoot | Smooth, damped | Noise amplification, jittery |

---

## How to Customize

### **Change Output Limits**

```
// Limit to ±200 instead of ±255
PID controller(8.0, 0.0, 80.0, -200, 200);
```

### **Add Derivative Filtering** (Reduce Noise)

```
// Inside compute(), replace:
derivative = (error - previousError) / deltaTime;

// With filtered version:
float rawDerivative = (error - previousError) / deltaTime;
derivative = derivative * 0.7 + rawDerivative * 0.3;  // Low-pass filter
```

### **Add Setpoint Ramping** (Smooth Target Changes)

```
float rampedSetpoint = setpoint;
if (abs(setpoint - previousSetpoint) > 10) {
    rampedSetpoint = previousSetpoint + 10 * sign(setpoint - previousSetpoint);
}
int error = rampedSetpoint - currentPosition;
```

### **Change Integral Limits**

```
// More aggressive anti-windup
float maxIntegral = maxOutput / (2 * Ki);  // Half the default
```

---

## Troubleshooting

### **Problem: Robot oscillates wildly**
**Cause:** Kp too high, or Kd too low
**Solution:**
- Reduce Kp by 30%
- Increase Kd
- Check sensor noise

### **Problem: Robot responds slowly**
**Cause:** Kp too low
**Solution:**
- Increase Kp gradually
- Test on straight section first

### **Problem: Robot drifts to one side**
**Cause:** Steady-state error, Ki = 0
**Solution:**
- Add small Ki (0.01-0.1)
- Or use motor trim instead

### **Problem: Robot overshoots curves**
**Cause:** Kd too low, or Kp too high
**Solution:**
- Increase Kd
- Reduce Kp slightly

### **Problem: Jittery/vibrating movement**
**Cause:** Kd too high, amplifying sensor noise
**Solution:**
- Reduce Kd
- Add derivative filtering
- Smooth sensor readings

### **Problem: Output always ±255**
**Cause:** PID output saturating
**Solution:**
- Reduce ALL gains proportionally
- Check minOutput/maxOutput limits
- Verify error values reasonable

---

## Example PID Behavior

**Scenario: Robot starts 50 units left of center**

| Time | Error | P | I | D | Output | Action |
|------|-------|---|---|---|--------|--------|
| 0.00s | 50 | 400 | 0 | 0 | 255 | Turn right (max) |
| 0.02s | 30 | 240 | 1 | -80000 | -255 | Turn left (overshoot prevention) |
| 0.04s | 15 | 120 | 2 | -60000 | -255 | Turn left |
| 0.06s | 5 | 40 | 3 | -40000 | -255 | Turn left |
| 0.08s | -2 | -16 | 3 | -28000 | -255 | Turn left (slight overshoot) |
| 0.10s | 0 | 0 | 3 | -8000 | -255 | Centered, correct back |
| 0.12s | 1 | 8 | 3 | 4000 | 255 | Small correction |
| 0.14s | 0 | 0 | 3 | -4000 | -255 | Stable |

**Observations:**
- P term provides main correction
- D term prevents overshoot (large negative values)
- I term accumulates slowly
- Output saturates frequently (normal)

---

## Key Concepts

### **Feedback Control**
- Continuously measures error
- Adjusts output to minimize error
- Self-correcting system

### **Proportional Control**
- Output proportional to error
- Fast response, but steady-state error
- Foundation of PID

### **Integral Control**
- Accumulates error over time
- Eliminates steady-state error
- Can cause windup if not limited

### **Derivative Control**
- Responds to rate of error change
- Dampens oscillations
- Predicts future error

### **Anti-Windup**
- Limits integral accumulation
- Prevents excessive overshoot
- Critical for practical systems

### **Saturation**
- Output hitting min/max limits
- Normal during large corrections
- Indicates system working hard

---

## Testing Checklist

**Basic Function:**
- [ ] PID object creates successfully
- [ ] `compute()` returns values in range
- [ ] Positive error → positive output
- [ ] Negative error → negative output

**P Term:**
- [ ] Increasing Kp → stronger response
- [ ] Kp = 0 → no correction

**I Term:**
- [ ] Persistent error → integral grows
- [ ] Integral limited by anti-windup
- [ ] Ki = 0 → no integral effect

**D Term:**
- [ ] Fast error change → large D correction
- [ ] Increasing Kd → smoother response
- [ ] Kd = 0 → no damping

**Edge Cases:**
- [ ] `reset()` clears state
- [ ] `setTunings()` updates gains
- [ ] deltaTime = 0 handled safely

---

## Pro Tips

1. **Start simple** - P only, then add D, finally I if needed
2. **Tune on straight line** - easier to see oscillation
3. **Print PID components** - see which term dominates
4. **Use normalized position** - makes tuning transferable
5. **Different speeds need different gains** - tune for your base speed
6. **Document your gains** - write down what works
7. **I term is optional** - many line followers work with just PD

---

## Integration with Other Modules

### **Used By:**
- `PIDController.h` - Creates and manages PID instances
- `main.cpp` (indirectly via PIDController)

### **Uses:**
- Arduino `millis()` - for timing
- Arduino `constrain()` - for limiting output

### **Data Flow:**
```
Linepos.h
  ↓
position (error from setpoint)
  ↓
PID.compute(setpoint=0, position)
  ↓
P calculation
I calculation (with anti-windup)
D calculation
  ↓
output = P + I + D
  ↓
constrain to [-255, 255]
  ↓
return correction
  ↓
MotorControl.h (applies correction)
```

---

## Quick Test Code

```
#include "PID.h"

PID testPID(8.0, 0.0, 80.0, -255, 255);

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Simulate line position
  int position = random(-50, 50);  // Random position
  
  // Calculate correction
  int correction = testPID.compute(0, position);
  
  // Print results
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" | Correction: ");
  Serial.println(correction);
  
  // Print PID components
  float p, i, d;
  testPID.getComponents(&p, &i, &d);
  Serial.print("P: "); Serial.print(p);
  Serial.print(" I: "); Serial.print(i);
  Serial.print(" D: "); Serial.println(d);
  
  delay(20);  // 50Hz update
}
```

**Expected Output:**
```
Position: -30 | Correction: -240
P: -240 I: -1.5 D: -12000

Position: 15 | Correction: 120
P: 120 I: 0.8 D: 18000
```

---