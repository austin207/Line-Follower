#include <Arduino.h>
#include "IR/IRSensor.h"
#include "LinePos/Linepos.h"
#include "PID/PID.h"
#include "BLE/BLEConfig.h"

// Create PID controller with initial values
PID* linePID = nullptr;

void setup() {
  Serial.begin(115200);
  
  // Initialize components
  initIRSensors();
  initBLE();
  
  // Create initial PID instance
  linePID = new PID(bleKp, bleKi, bleKd, bleMinOutput, bleMaxOutput);
  
  Serial.println("System Ready!");
}

void loop() {
  // Read sensors
  readIRSensors();
  int position = readLinePosition();
  
  // Get latest PID parameters from BLE
  float kp, ki, kd;
  int minOut, maxOut;
  getCurrentPIDParams(&kp, &ki, &kd, &minOut, &maxOut);
  
  // Recreate PID if output limits changed
  static int lastMinOut = bleMinOutput;
  static int lastMaxOut = bleMaxOutput;
  
  if (minOut != lastMinOut || maxOut != lastMaxOut) {
    delete linePID;
    linePID = new PID(kp, ki, kd, minOut, maxOut);
    lastMinOut = minOut;
    lastMaxOut = maxOut;
    Serial.println("PID output limits updated - controller reset");
  } else {
    // Just update tunings if only Kp, Ki, Kd changed
    linePID->setTunings(kp, ki, kd);
  }
  
  if (position != -999) {
    // Calculate PID correction
    int correction = linePID->compute(0, position);
    
    // Send position to Flutter app for monitoring
    updatePositionValue(position);
    
    Serial.print("Pos: ");
    Serial.print(position);
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
    Serial.println(correction);
  } else {
    Serial.println("Line LOST!");
  }
  
  delay(50);
}
