#include <Arduino.h>
#include "IR/IRSensor.h"
#include "Calibration/Calibration.h"
#include "LinePos/Linepos.h"
#include "BLE/BLEConfig.h"
#include "PIDController/PIDController.h"
#include "Motor/MotorControl.h"

PIDController pidController;
bool isRunning = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize all subsystems
  initIRSensors();
  initCalibration();
  initMotors();
  initBLE();
  
  Serial.println("=== Line Follower Ready ===");
  Serial.println("Send '1' to CALIBRATE to calibrate sensors");
  Serial.println("Send '1' to ESTOP to stop, '0' to resume");
}

void loop() {
  // Handle emergency stop
  if (emergencyStopTriggered) {
    if (isRunning) {
      emergencyStop();
      isRunning = false;
    }
    delay(100);
    return;
  } else {
    if (!isRunning) {
      resume();
      isRunning = true;
    }
  }
  
  // Handle calibration request
  if (startCalibration) {
    stopMotors();
    calibrate(5000);
    startCalibration = false;
  }
  
  // Read sensors and calculate line position
  readIRSensors();
  int position = readLinePosition();
  
  // Update PID and get correction
  int correction = pidController.update(position);
  
  // Apply correction to motors
  if (position != -999) {
    applyPIDCorrection(correction);
    
    // Send live position to Flutter app
    updatePositionValue(position);
    
    // Debug output
    pidController.printDebug();
  } else {
    // Line lost - stop motors
    stopMotors();
    Serial.println("Line LOST! Motors stopped.");
  }
  
  delay(20);  // 50Hz control loop
}
