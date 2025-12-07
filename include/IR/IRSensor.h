#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

// IR Sensor Pin Definitions
const int IR_PINS[8] = {13, 12, 14, 27, 26, 25, 33, 32};
const int NUM_SENSORS = 8;

// Sensor readings array (analog values 0-4095)
int sensorValues[8];

// Function to initialize IR sensor pins and ADC
void initIRSensors() {
  // Set ADC resolution to 12-bit (0-4095)
  analogReadResolution(12);
  
  // ADC pins don't need pinMode setup for analogRead
  Serial.println("IR Sensor Array Initialized (Analog Mode)");
}

// Function to read all 8 IR sensors (analog)
void readIRSensors() {
  for(int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
    // QTR-8A: Lower values = stronger reflection (white surface)
    //         Higher values = weaker reflection (black line)
  }
}

// Function to print sensor values for debugging
/*
void printSensorValues() {
  Serial.print("Sensors: ");
  for(int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}
*/
#endif // IR_SENSOR_H
