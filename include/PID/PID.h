#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
private:
    // PID constants
    float Kp;  // Proportional gain
    float Ki;  // Integral gain  
    float Kd;  // Derivative gain
    
    // PID variables
    float previousError;
    float integral;
    float derivative;
    
    // Timing
    unsigned long lastTime;
    
    // Output limits
    int minOutput;
    int maxOutput;

public:
    // Constructor
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
    
    // Calculate PID output
    int compute(int setpoint, int currentPosition) {
        // Calculate error (desired position - actual position)
        int error = setpoint - currentPosition;
        
        // Calculate time delta
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
        
        // Prevent division by zero
        if (deltaTime <= 0) deltaTime = 0.001;
        
        // Proportional term
        float P = Kp * error;
        
        // Integral term (accumulated error over time)
        integral += error * deltaTime;
        
        // Anti-windup: limit integral to prevent it from growing too large
        float maxIntegral = maxOutput / Ki;
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;
        
        float I = Ki * integral;
        
        // Derivative term (rate of change of error)
        derivative = (error - previousError) / deltaTime;
        float D = Kd * derivative;
        
        // Calculate total PID output
        float output = P + I + D;
        
        // Constrain output to limits
        output = constrain(output, minOutput, maxOutput);
        
        // Update variables for next iteration
        previousError = error;
        lastTime = currentTime;
        
        return (int)output;
    }
    
    // Reset PID controller
    void reset() {
        previousError = 0;
        integral = 0;
        derivative = 0;
        lastTime = millis();
    }
    
    // Update PID constants on the fly
    void setTunings(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
    
    // Get individual PID components for debugging
    void getComponents(float* p, float* i, float* d) {
        *p = Kp * previousError;
        *i = Ki * integral;
        *d = Kd * derivative;
    }
};

#endif // PID_H
