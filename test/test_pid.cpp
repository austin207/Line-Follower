#include "PID/PID.h"
#include "LinePos/Linepos.h"

// Create PID controller
// PID(Kp, Ki, Kd, minOutput, maxOutput)
PID linePID(8.0, 0.0, 80.0, -255, 255);

void loop() {
    readIRSensors();
    int position = readLinePosition();
    
    if (position != -999) {
        // Setpoint = 0 (keep line centered)
        int correction = linePID.compute(0, position);
        
        Serial.print("Position: ");
        Serial.print(position);
        Serial.print(" | Correction: ");
        Serial.println(correction);
        
        // Use correction for motor control (next step)
    }
}
