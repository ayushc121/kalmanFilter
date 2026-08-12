#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>


// Tell the compiler that MPU_ADDR is defined in main.cpp
extern const int MPU_ADDR;

extern int32_t gyroBiasX;
extern int32_t gyroBiasY;
extern int32_t gyroBiasZ;

extern HardwareSerial gpsSerial; 
extern TinyGPSPlus gps;

extern double pNaught[3];

void calibrateIMU_3D();
void initializeGPSReference();

#endif