#ifndef BLE_CONFIG_H
#define BLE_CONFIG_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Motor/MotorControl.h"  

// UUIDs for BLE Service and Characteristics
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define KP_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define KI_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define KD_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define MIN_OUT_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26ac"
#define MAX_OUT_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26ad"
#define POSITION_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26ab"
#define CALIBRATE_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ae"
#define BASE_SPEED_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26af"
#define ESTOP_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26b0"

// Global PID values (will be accessed by main code)
float bleKp = 8.0;
float bleKi = 0.0;
float bleKd = 80.0;
int bleMinOutput = -255;
int bleMaxOutput = 255;

// BLE Server and Characteristics
BLEServer* pServer = nullptr;
BLECharacteristic* pKpCharacteristic = nullptr;
BLECharacteristic* pKiCharacteristic = nullptr;
BLECharacteristic* pKdCharacteristic = nullptr;
BLECharacteristic* pMinOutCharacteristic = nullptr;
BLECharacteristic* pMaxOutCharacteristic = nullptr;
BLECharacteristic* pPositionCharacteristic = nullptr;
BLECharacteristic* pCalibrateCharacteristic = nullptr;
BLECharacteristic* pBaseSpeedCharacteristic = nullptr;
BLECharacteristic* pEStopCharacteristic = nullptr;

bool startCalibration = false;
bool deviceConnected = false;
bool emergencyStopTriggered = false;  // Defined here for BLE access

// Server callbacks for connection status
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client Disconnected");
        pServer->startAdvertising();
    }
};

// Callback for Kp characteristic
class KpCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            bleKp = atof(value.c_str());
            Serial.print("Kp updated to: ");
            Serial.println(bleKp, 4);
        }
    }
};

// Callback for Ki characteristic
class KiCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            bleKi = atof(value.c_str());
            Serial.print("Ki updated to: ");
            Serial.println(bleKi, 4);
        }
    }
};

// Callback for Kd characteristic
class KdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            bleKd = atof(value.c_str());
            Serial.print("Kd updated to: ");
            Serial.println(bleKd, 4);
        }
    }
};

// Callback for MinOutput characteristic
class MinOutCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            bleMinOutput = atoi(value.c_str());
            Serial.print("MinOutput updated to: ");
            Serial.println(bleMinOutput);
        }
    }
};

// Callback for MaxOutput characteristic
class MaxOutCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            bleMaxOutput = atoi(value.c_str());
            Serial.print("MaxOutput updated to: ");
            Serial.println(bleMaxOutput);
        }
    }
};

// Callback for Calibrate characteristic
class CalibrateCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0 && value[0] == '1') {
            startCalibration = true;
            Serial.println("Calibration triggered via BLE");
        }
    }
};

// Callback for BaseSpeed characteristic
class BaseSpeedCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            int speed = atoi(value.c_str());
            setBaseSpeed(speed);  // Call function from MotorControl.h
        }
    }
};

// Callback for EStop characteristic
class EStopCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0 && value[0] == '1') {
            emergencyStopTriggered = true;
        } else {
            emergencyStopTriggered = false;
        }
    }
};

// Initialize BLE Server
void initBLE() {
    Serial.println("Initializing BLE...");
    
    // Create BLE Device
    BLEDevice::init("ESP32-LineFollower");
    
    // Create BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create BLE Service
    BLEService* pService = pServer->createService(SERVICE_UUID);
    
    // Create Kp Characteristic (READ + WRITE)
    pKpCharacteristic = pService->createCharacteristic(
        KP_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE
    );
    pKpCharacteristic->setCallbacks(new KpCallbacks());
    pKpCharacteristic->setValue(String(bleKp).c_str());
    
    // Create Ki Characteristic (READ + WRITE)
    pKiCharacteristic = pService->createCharacteristic(
        KI_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE
    );
    pKiCharacteristic->setCallbacks(new KiCallbacks());
    pKiCharacteristic->setValue(String(bleKi).c_str());
    
    // Create Kd Characteristic (READ + WRITE)
    pKdCharacteristic = pService->createCharacteristic(
        KD_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE
    );
    pKdCharacteristic->setCallbacks(new KdCallbacks());
    pKdCharacteristic->setValue(String(bleKd).c_str());
    
    // Create MinOutput Characteristic (READ + WRITE)
    pMinOutCharacteristic = pService->createCharacteristic(
        MIN_OUT_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE
    );
    pMinOutCharacteristic->setCallbacks(new MinOutCallbacks());
    pMinOutCharacteristic->setValue(String(bleMinOutput).c_str());
    
    // Create MaxOutput Characteristic (READ + WRITE)
    pMaxOutCharacteristic = pService->createCharacteristic(
        MAX_OUT_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE
    );
    pMaxOutCharacteristic->setCallbacks(new MaxOutCallbacks());
    pMaxOutCharacteristic->setValue(String(bleMaxOutput).c_str());
    
    // Create Position Characteristic (READ + NOTIFY for live monitoring)
    pPositionCharacteristic = pService->createCharacteristic(
        POSITION_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pPositionCharacteristic->addDescriptor(new BLE2902());
    
    // Create Calibrate Characteristic (WRITE)
    pCalibrateCharacteristic = pService->createCharacteristic(
        CALIBRATE_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCalibrateCharacteristic->setCallbacks(new CalibrateCallbacks());
    pCalibrateCharacteristic->setValue("0");

    // Create BaseSpeed Characteristic (READ + WRITE)
    pBaseSpeedCharacteristic = pService->createCharacteristic(
        BASE_SPEED_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE
    );
    pBaseSpeedCharacteristic->setCallbacks(new BaseSpeedCallbacks());
    pBaseSpeedCharacteristic->setValue("150");  // Default base speed

    // Create EStop Characteristic (WRITE)
    pEStopCharacteristic = pService->createCharacteristic(
        ESTOP_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pEStopCharacteristic->setCallbacks(new EStopCallbacks());
    pEStopCharacteristic->setValue("0");
    
    // Start the service
    pService->start();
    
    // Start advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Server started! Device name: ESP32-LineFollower");
}

// Update position value for live monitoring (call this in loop)
void updatePositionValue(int position) {
    if (deviceConnected) {
        pPositionCharacteristic->setValue(position);
        pPositionCharacteristic->notify();
    }
}

// Get current PID parameters (all 5 values)
void getCurrentPIDParams(float* kp, float* ki, float* kd, int* minOut, int* maxOut) {
    *kp = bleKp;
    *ki = bleKi;
    *kd = bleKd;
    *minOut = bleMinOutput;
    *maxOut = bleMaxOutput;
}

#endif // BLE_CONFIG_H
