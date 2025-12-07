#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "PID/PID.h"
#include "BLE/BLEConfig.h"
#include "LinePos/Linepos.h"

class PIDController {
private:
    PID* pid;
    int lastMinOut;
    int lastMaxOut;
    int currentCorrection;
    int currentPosition;

public:
    // Constructor
    PIDController() {
        // Create initial PID instance with BLE defaults
        pid = new PID(bleKp, bleKi, bleKd, bleMinOutput, bleMaxOutput);
        lastMinOut = bleMinOutput;
        lastMaxOut = bleMaxOutput;
        currentCorrection = 0;
        currentPosition = 0;
    }
    
    // Destructor
    ~PIDController() {
        delete pid;
    }
    
    // Update PID with latest BLE parameters and calculate correction
    int update(int linePosition) {
        currentPosition = linePosition;
        
        // Get latest parameters from BLE
        float kp, ki, kd;
        int minOut, maxOut;
        getCurrentPIDParams(&kp, &ki, &kd, &minOut, &maxOut);
        
        // Check if output limits changed (requires PID recreation)
        if (minOut != lastMinOut || maxOut != lastMaxOut) {
            delete pid;
            pid = new PID(kp, ki, kd, minOut, maxOut);
            lastMinOut = minOut;
            lastMaxOut = maxOut;
            Serial.println("PID output limits updated - controller reset");
        } else {
            // Just update tunings if only Kp, Ki, Kd changed
            pid->setTunings(kp, ki, kd);
        }
        
        // Calculate correction (setpoint = 0 for centered line)
        if (linePosition != -999) {
            currentCorrection = pid->compute(0, linePosition);
        } else {
            currentCorrection = 0; // No correction if line is lost
        }
        
        return currentCorrection;
    }
    
    // Get current correction value
    int getCorrection() {
        return currentCorrection;
    }
    
    // Get current position
    int getPosition() {
        return currentPosition;
    }
    
    // Reset PID controller
    void reset() {
        pid->reset();
    }
    
    // Print debug info
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
};

#endif // PID_CONTROLLER_H
