#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Preferences.h>
#include "IR/IRSensor.h"

// Calibration data storage
int sensorMin[8];
int sensorMax[8];
int sensorThreshold[8];
bool isCalibrated = false;

void saveCalibration();
bool loadCalibration();
void clearCalibration();
void printCalibrationData();

// Preferences for persistent storage
Preferences preferences;

// Initialize calibration arrays with default values
void initCalibration() {
    // Set default values (uncalibrated state)
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;  // Start with max possible value
        sensorMax[i] = 0;     // Start with min possible value
        sensorThreshold[i] = 2000; // Default threshold
    }
    
    // Try to load saved calibration from flash
    loadCalibration();
}

// Perform calibration routine
// Move robot over black and white surfaces during this process
void calibrate(unsigned long duration = 5000) {
    Serial.println("=== CALIBRATION STARTED ===");
    Serial.println("Move sensor array over BLACK and WHITE surfaces...");
    
    // Reset calibration values
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }
    
    unsigned long startTime = millis();
    int samples = 0;
    
    // Collect min/max values for specified duration
    while (millis() - startTime < duration) {
        readIRSensors();
        
        for (int i = 0; i < NUM_SENSORS; i++) {
            // Update minimum value
            if (sensorValues[i] < sensorMin[i]) {
                sensorMin[i] = sensorValues[i];
            }
            
            // Update maximum value
            if (sensorValues[i] > sensorMax[i]) {
                sensorMax[i] = sensorValues[i];
            }
        }
        
        samples++;
        
        // Visual feedback every 500ms
        if (samples % 50 == 0) {
            Serial.print(".");
        }
        
        delay(10);
    }
    
    Serial.println("\n=== CALIBRATION COMPLETE ===");
    
    // Calculate threshold values (midpoint between min and max)
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    }
    
    isCalibrated = true;
    
    // Save to flash memory
    saveCalibration();
}

// Read calibrated sensor values (normalized 0-1000)
void readCalibratedSensors(int* calibratedValues) {
    readIRSensors();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int range = sensorMax[i] - sensorMin[i];
        
        // Prevent division by zero
        if (range == 0) {
            calibratedValues[i] = 0;
            continue;
        }
        
        // Map sensor value to 0-1000 range
        long value = (long)(sensorValues[i] - sensorMin[i]) * 1000 / range;
        
        // Constrain to valid range
        calibratedValues[i] = constrain(value, 0, 1000);
    }
}

// Check if sensor detects line (using threshold)
bool isOnLine(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= NUM_SENSORS) return false;
    return sensorValues[sensorIndex] > sensorThreshold[sensorIndex];
}

// Get binary representation of line detection (for debugging)
uint8_t getLineBinary() {
    uint8_t binary = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (isOnLine(i)) {
            binary |= (1 << i);
        }
    }
    return binary;
}

// Print calibration data
void printCalibrationData() {
    Serial.println("\n--- Calibration Data ---");
    
    Serial.print("Min Values: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorMin[i]);
        Serial.print("\t");
    }
    Serial.println();
    
    Serial.print("Max Values: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorMax[i]);
        Serial.print("\t");
    }
    Serial.println();
    
    Serial.print("Threshold:  ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorThreshold[i]);
        Serial.print("\t");
    }
    Serial.println("\n");
}

// Save calibration to flash memory (persistent)
void saveCalibration() {
    preferences.begin("calibration", false);
    
    // Save min values
    for (int i = 0; i < NUM_SENSORS; i++) {
        String minKey = "min" + String(i);
        preferences.putInt(minKey.c_str(), sensorMin[i]);
    }
    
    // Save max values
    for (int i = 0; i < NUM_SENSORS; i++) {
        String maxKey = "max" + String(i);
        preferences.putInt(maxKey.c_str(), sensorMax[i]);
    }
    
    // Save threshold values
    for (int i = 0; i < NUM_SENSORS; i++) {
        String thrKey = "thr" + String(i);
        preferences.putInt(thrKey.c_str(), sensorThreshold[i]);
    }
    
    preferences.putBool("calibrated", true);
    preferences.end();
    
    Serial.println("Calibration saved to flash memory!");
}

// Load calibration from flash memory
bool loadCalibration() {
    preferences.begin("calibration", true); // Read-only mode
    
    bool wasPreviouslyCalibrated = preferences.getBool("calibrated", false);
    
    if (!wasPreviouslyCalibrated) {
        preferences.end();
        Serial.println("No saved calibration found.");
        return false;
    }
    
    // Load min values
    for (int i = 0; i < NUM_SENSORS; i++) {
        String minKey = "min" + String(i);
        sensorMin[i] = preferences.getInt(minKey.c_str(), 4095);
    }
    
    // Load max values
    for (int i = 0; i < NUM_SENSORS; i++) {
        String maxKey = "max" + String(i);
        sensorMax[i] = preferences.getInt(maxKey.c_str(), 0);
    }
    
    // Load threshold values
    for (int i = 0; i < NUM_SENSORS; i++) {
        String thrKey = "thr" + String(i);
        sensorThreshold[i] = preferences.getInt(thrKey.c_str(), 2000);
    }
    
    preferences.end();
    
    isCalibrated = true;
    Serial.println("Calibration loaded from flash memory!");
    printCalibrationData();
    
    return true;
}

// Clear saved calibration
void clearCalibration() {
    preferences.begin("calibration", false);
    preferences.clear();
    preferences.end();
    
    isCalibrated = false;
    Serial.println("Calibration data cleared!");
}

#endif // CALIBRATION_H
