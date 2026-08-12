#include "Calibration.h"

void calibrateIMU_3D() {
  Serial.println("Calibrating Gyros... DO NOT MOVE THE BOARD");
  
  const int numSamples = 500;
  
  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);   

    Wire.read(); Wire.read(); // accelX
    Wire.read(); Wire.read(); // accelY
    Wire.read(); Wire.read(); // accelZ
    Wire.read(); Wire.read(); // temp
    gyroBiasX += (int16_t)(Wire.read() << 8 | Wire.read()); 
    gyroBiasY += (int16_t)(Wire.read() << 8 | Wire.read()); 
    gyroBiasZ += (int16_t)(Wire.read() << 8 | Wire.read());
        
    delay(3); 
  }
    
  gyroBiasX /= numSamples;
  gyroBiasY /= numSamples;
  gyroBiasZ /= numSamples;
  
  Serial.println("Gyro Calibration complete.");
}


void initializeGPSReference() {

    const int TARGET_SAMPLES = 50;
    int samplesCollected = 0;
    
    // Use double for precision during summation to prevent overflow
    double sumLon = 0.0;
    double sumLat = 0.0;
    double sumAlt = 0.0;

    Serial.println("Waiting for GPS lock and averaging samples...");

    while (samplesCollected < TARGET_SAMPLES) {

      while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }

        if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isValid()) {
            sumLon += gps.location.lng();
            sumLat += gps.location.lat();
            sumAlt += gps.altitude.meters();
            
            samplesCollected++;
            
            if (samplesCollected % 2 == 0) {
                Serial.print("Collected ");
                Serial.print(samplesCollected);
                Serial.println(" samples...");
            }
        }
    }

    // Calculate the averages and store them in your reference array
    pNaught[0] = sumLon / TARGET_SAMPLES;
    pNaught[1] = sumLat / TARGET_SAMPLES;
    pNaught[2] = sumAlt / TARGET_SAMPLES;

    Serial.println("GPS Reference Initialized!");
    Serial.print("Initial position (lng, lat, alt): (");
    Serial.print(pNaught[0], 8); Serial.print(", ");
    Serial.print(pNaught[1], 8); Serial.print(", ");
    Serial.print(pNaught[2]); Serial.println(")");
}