#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// TB6612FNG Pin Definitions
#define PWMA_PIN    5   // Motor A speed control
#define AIN2_PIN    18  // Motor A direction control 2
#define AIN1_PIN    19  // Motor A direction control 1
#define BIN1_PIN    21  // Motor B direction control 1
#define BIN2_PIN    22  // Motor B direction control 2
#define PWMB_PIN    23  // Motor B speed control

// PWM Configuration
#define PWM_FREQ      20000  // 20kHz - above human hearing range
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_A  0     // LEDC channel for Motor A
#define PWM_CHANNEL_B  1     // LEDC channel for Motor B

// Motor speeds
int baseSpeed = 150;         // Default base speed (0-255)
int motorASpeed = 0;
int motorBSpeed = 0;

void stopMotors();

// Initialize motor control pins and PWM
void initMotors() {
    // Configure direction control pins as outputs
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);
    
    // Configure PWM channels for motor speed control
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    
    // Attach PWM channels to pins
    ledcAttachPin(PWMA_PIN, PWM_CHANNEL_A);
    ledcAttachPin(PWMB_PIN, PWM_CHANNEL_B);
    
    // Stop motors initially
    stopMotors();
    
    Serial.println("Motor control initialized!");
}

// Motor A control
void setMotorA(int speed) {
    motorASpeed = constrain(speed, -255, 255);
    
    if (motorASpeed > 0) {
        // Forward
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_A, motorASpeed);
    } else if (motorASpeed < 0) {
        // Backward
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
        ledcWrite(PWM_CHANNEL_A, -motorASpeed);
    } else {
        // Stop (brake)
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_A, 0);
    }
}

// Motor B control
void setMotorB(int speed) {
    motorBSpeed = constrain(speed, -255, 255);
    
    if (motorBSpeed > 0) {
        // Forward
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_B, motorBSpeed);
    } else if (motorBSpeed < 0) {
        // Backward
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, HIGH);
        ledcWrite(PWM_CHANNEL_B, -motorBSpeed);
    } else {
        // Stop (brake)
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_B, 0);
    }
}

// Apply PID correction to motors for line following
void applyPIDCorrection(int correction) {
    // Differential steering: one motor speeds up, other slows down
    // Correction is negative when line is left, positive when right
    
    int leftMotorSpeed = baseSpeed - correction;
    int rightMotorSpeed = baseSpeed + correction;
    
    // Constrain to valid PWM range
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    
    setMotorA(leftMotorSpeed);   // Left motor
    setMotorB(rightMotorSpeed);  // Right motor
}

// Stop both motors
void stopMotors() {
    setMotorA(0);
    setMotorB(0);
}

// Move forward at base speed
void moveForward() {
    setMotorA(baseSpeed);
    setMotorB(baseSpeed);
}

// Move backward at base speed
void moveBackward() {
    setMotorA(-baseSpeed);
    setMotorB(-baseSpeed);
}

// Turn left in place
void turnLeft(int speed = 100) {
    setMotorA(-speed);
    setMotorB(speed);
}

// Turn right in place
void turnRight(int speed = 100) {
    setMotorA(speed);
    setMotorB(-speed);
}

// Set base speed (for BLE control)
void setBaseSpeed(int speed) {
    baseSpeed = constrain(speed, 0, 255);
    Serial.print("Base speed set to: ");
    Serial.println(baseSpeed);
}

// Get current motor speeds
void getMotorSpeeds(int* leftSpeed, int* rightSpeed) {
    *leftSpeed = motorASpeed;
    *rightSpeed = motorBSpeed;
}

// Emergency stop
void emergencyStop() {
    stopMotors();
    Serial.println("EMERGENCY STOP ACTIVATED!");
}

// Resume from emergency stop
void resume() {
    Serial.println("Motors resumed");
}

#endif // MOTOR_CONTROL_H
