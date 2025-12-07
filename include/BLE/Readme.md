# BLEConfig.h - Bluetooth Low Energy Configuration

## What Does This File Do?

This file creates a **wireless control system** for your line follower robot using Bluetooth Low Energy (BLE). It allows you to adjust the robot's settings from your phone **without uploading new code** every time you want to change something.

Think of it as a **remote control panel** for your robot's brain!

---

## Purpose

**Problem**: Tuning a line follower robot normally requires:
1. Change code → Upload → Test → Repeat (very slow!)

**Solution**: This BLE system lets you:
- Adjust PID values (Kp, Ki, Kd) in real-time
- Change motor speed limits
- Calibrate sensors remotely
- Emergency stop the robot
- Monitor live sensor data

All from your phone via a Flutter app!

---

## How It Works

### **BLE Architecture**

BLE uses a **server-client model**:
- **ESP32 = BLE Server** (your robot)
- **Flutter App = BLE Client** (your phone)

The server has **9 characteristics** (like individual channels) that store different settings:

| Characteristic | Type | What It Does | Default Value |
|----------------|------|--------------|---------------|
| **Kp** | Float | Proportional gain for PID | 8.0 |
| **Ki** | Float | Integral gain for PID | 0.0 |
| **Kd** | Float | Derivative gain for PID | 80.0 |
| **MinOutput** | Int | Minimum motor speed | -255 |
| **MaxOutput** | Int | Maximum motor speed | 255 |
| **Position** | Int | Current line position (read-only) | - |
| **Calibrate** | Bool | Trigger sensor calibration | 0 |
| **BaseSpeed** | Int | Robot base speed | 150 |
| **EStop** | Bool | Emergency stop | 0 |

---

## Code Breakdown

### **1. UUIDs - Unique Identifiers**

```
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define KP_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
```

**What are UUIDs?**
- Universally Unique IDentifiers (like phone numbers for BLE)
- Your Flutter app uses these to find the correct characteristics
- **Don't change these** unless you also update your Flutter app!

**How to Generate New UUIDs** (if needed):
- Visit: https://www.uuidgenerator.net/
- Copy the generated UUID
- Replace in both ESP32 code and Flutter app

---

### **2. Global Variables - Storage**

```
float bleKp = 8.0;      // PID proportional gain
float bleKi = 0.0;      // PID integral gain
float bleKd = 80.0;     // PID derivative gain
```

**What are these?**
- These store the current PID values
- When your Flutter app sends new values, these variables update
- Other parts of your code (PIDController.h) read these values

**How to Change Defaults:**
```
float bleKp = 10.0;     // Change starting Kp to 10
int bleMaxOutput = 200; // Limit max motor speed to 200 instead of 255
```

---

### **3. BLE Pointers - Connections**

```
BLEServer* pServer = nullptr;
BLECharacteristic* pKpCharacteristic = nullptr;
```

**What are pointers?**
- Think of them as **bookmarks** to BLE objects
- `nullptr` means "not pointing to anything yet"
- They get set up during `initBLE()`

**Why so many?**
- One for the server
- One for each of the 9 characteristics

---

### **4. Status Flags - State Tracking**

```
bool deviceConnected = false;          // Is phone connected?
bool emergencyStopTriggered = false;   // Is emergency stop active?
bool startCalibration = false;         // Should calibration start?
```

**How These Work:**
- `deviceConnected`: Automatically set when phone connects/disconnects
- `emergencyStopTriggered`: Set when you press "STOP" in the app
- `startCalibration`: Set when you press "Calibrate" in the app

**Usage in main.cpp:**
```
if (emergencyStopTriggered) {
    stopMotors();  // Safety first!
}
```

---

### **5. Callback Classes - Event Handlers**

```
class KpCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        bleKp = atof(value.c_str());  // Convert string to float
        Serial.println(bleKp);
    }
};
```

**What is a Callback?**
- A function that **automatically runs** when something happens
- Like a notification: "Hey! The phone just sent new data!"

**How It Works:**
1. Flutter app sends new Kp value (e.g., "12.5")
2. BLE receives it as a string
3. `onWrite()` callback **automatically triggers**
4. Convert string → float: `atof("12.5")` = 12.5
5. Update global variable: `bleKp = 12.5`
6. Print confirmation to Serial Monitor

**All 9 Characteristics Have Callbacks:**
- `KpCallbacks`, `KiCallbacks`, `KdCallbacks` → PID gains (float)
- `MinOutCallbacks`, `MaxOutCallbacks` → Motor limits (int)
- `BaseSpeedCallbacks` → Base speed (int, calls `setBaseSpeed()`)
- `CalibrateCallbacks` → Trigger flag (bool)
- `EStopCallbacks` → Emergency stop (bool)

---

### **6. initBLE() - Setup Function**

```
void initBLE() {
    BLEDevice::init("ESP32-LineFollower");  // Name shown in phone's BLE list
    pServer = BLEDevice::createServer();
    // ... create characteristics ...
    BLEDevice::startAdvertising();          // Start broadcasting
}
```

**What This Does:**
1. **Create BLE device** with name "ESP32-LineFollower"
2. **Create server** to handle connections
3. **Create service** (container for characteristics)
4. **Create 9 characteristics** with UUIDs
5. **Attach callbacks** to handle incoming data
6. **Set default values** for each characteristic
7. **Start advertising** (make robot discoverable)

**How to Change Robot Name:**
```
BLEDevice::init("MyRobot123");  // Shows as "MyRobot123" in phone
```

---

### **7. Characteristic Properties**

```
BLECharacteristic::PROPERTY_READ    // Phone can read current value
BLECharacteristic::PROPERTY_WRITE   // Phone can send new value
BLECharacteristic::PROPERTY_NOTIFY  // ESP32 can push updates to phone
```

**Examples:**
- **Kp, Ki, Kd**: `READ + WRITE` (phone can read and change)
- **Position**: `READ + NOTIFY` (ESP32 sends live updates)
- **Calibrate, EStop**: `WRITE` only (phone sends commands)

---

### **8. Helper Functions**

```
void updatePositionValue(int position) {
    if (deviceConnected) {
        pPositionCharacteristic->setValue(position);
        pPositionCharacteristic->notify();  // Push to phone
    }
}
```

**What This Does:**
- Sends current line position to Flutter app in real-time
- Only sends if phone is connected (saves bandwidth)
- Called from `main.cpp` every loop iteration

```
void getCurrentPIDParams(float* kp, float* ki, float* kd, int* minOut, int* maxOut) {
    *kp = bleKp;
    *ki = bleKi;
    // ...
}
```

**What This Does:**
- Packages all 5 PID parameters into one function call
- Used by `PIDController.h` to get latest values
- The `*` means "pointer" - allows function to modify multiple variables

---

## How to Customize

### **Change Default PID Values**
```
float bleKp = 12.0;   // Start with Kp=12 instead of 8
float bleKd = 100.0;  // Start with Kd=100 instead of 80
```

### **Change Robot Name**
```
BLEDevice::init("SuperBot2025");
```

### **Change Default Base Speed**
```
pBaseSpeedCharacteristic->setValue("180");  // Start at speed 180 instead of 150
```

### **Add New Characteristic** (Advanced)
1. **Define UUID:**
   ```
   #define NEW_PARAM_UUID "beb5483e-36e1-4688-b7f5-ea07361b26b1"
   ```

2. **Create global variable:**
   ```
   int myNewParameter = 100;
   ```

3. **Create callback class:**
   ```
   class NewParamCallbacks : public BLECharacteristicCallbacks {
       void onWrite(BLECharacteristic* pCharacteristic) {
           myNewParameter = atoi(pCharacteristic->getValue().c_str());
       }
   };
   ```

4. **Add to initBLE():**
   ```
   pNewCharacteristic = pService->createCharacteristic(
       NEW_PARAM_UUID,
       BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
   );
   pNewCharacteristic->setCallbacks(new NewParamCallbacks());
   ```

---

## Troubleshooting

**Problem: Phone can't find "ESP32-LineFollower"**
- Solution: Check Serial Monitor for "BLE Server started!"
- Make sure Bluetooth is enabled on phone
- Try restarting ESP32

**Problem: Can't write to characteristic**
- Solution: Check if characteristic has `PROPERTY_WRITE` flag
- Verify UUID matches between ESP32 and Flutter app

**Problem: Updates not happening**
- Solution: Check callback function - is `Serial.println()` showing new values?
- Verify data type conversion (`atof` for float, `atoi` for int)

---

## Key Concepts

**BLE vs Classic Bluetooth:**
- BLE = Low power, perfect for IoT devices
- Classic = High power, used for audio streaming

**Server vs Client:**
- Server = ESP32 (provides data)
- Client = Phone (requests data)

**Characteristic:**
- Like a "variable" you can read/write wirelessly
- Has UUID (address) and value (data)

**Callback:**
- Function that runs automatically when event happens
- No need to constantly check - BLE notifies you!

---

## Testing Checklist

- [ ] Serial Monitor shows "BLE Server started!"
- [ ] Phone can discover "ESP32-LineFollower"
- [ ] Can read Kp, Ki, Kd values
- [ ] Changing values updates Serial Monitor
- [ ] Position updates in real-time
- [ ] Emergency stop works
- [ ] Calibration trigger works

---

## Learning Resources

- **BLE Basics**: https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
- **UUID Generator**: https://www.uuidgenerator.net/
- **ESP32 BLE Library Docs**: https://github.com/nkolban/ESP32_BLE_Arduino

---

## Tips

1. **Always test BLE before adding motors** - easier to debug
2. **Use Serial Monitor** to verify values are updating
3. **Keep UUIDs consistent** between ESP32 and Flutter app
4. **Start with default values** - tune later via BLE
5. **Add Serial.println() in callbacks** to debug incoming data

---