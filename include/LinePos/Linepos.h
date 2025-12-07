#ifndef LINEPOS_H
#define LINEPOS_H

#include "IR/IRSensor.h"

// Sensor weights: -4, -3, -2, -1, 1, 2, 3, 4 (left to right)
// Center reference point is between sensors 3 and 4 (0-indexed)
const int SENSOR_WEIGHTS[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

// Read line position using weighted average
// Returns: Position value (negative = left, positive = right, 0 = center)
// Returns -999 if no line detected
int readLinePosition() {
    int weightedSum = 0;
    int activeSensors = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        // For QTR-8A: Higher analog value = black line detected
        // Threshold to determine if sensor sees the line (adjust based on your readings)
        int value = sensorValues[i];
        
        // Only count sensors that detect the line (above threshold)
        if (value > 1500) { // Adjust this threshold based on your sensor readings
            weightedSum += SENSOR_WEIGHTS[i] * value;
            activeSensors += value;
        }
    }
    
    // No line detected
    if (activeSensors == 0) {
        return -999; // Error code for "line lost"
    }
    
    // Calculate position: Σ(sensorValue[i] × weight[i]) / Σ(active_sensors)
    return weightedSum / activeSensors;
}

// Alternative: Get normalized position (-100 to +100 for easier PID control)
int readLinePositionNormalized() {
    int rawPosition = readLinePosition();
    
    if (rawPosition == -999) {
        return -999; // Line lost
    }
    
    // Scale to -100 (far left) to +100 (far right)
    // Raw range is approximately -4 to +4 based on weights
    return constrain(rawPosition * 25, -100, 100);
}

#endif // LINEPOS_H
