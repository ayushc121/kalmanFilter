#include <HardwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>

#include "Calibration.h"
#include "Madgwick.h"
#include "Kalman.h"



HardwareSerial gpsSerial(1); 
const int RX_PIN = 18; // Connected to GPS TX
const int TX_PIN = 17; // Connected to GPS RX

const int MPU_ADDR = 0x68;

// Data variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
int16_t tempRaw;

// Constants from external accelerometer calibration script
int32_t accelBiasX = -321;
int32_t accelBiasY = 492;
int32_t accelBiasZ = -1483;
float accelScaleX = 1.002191f;
float accelScaleY = 1.003132f;
float accelScaleZ = 0.994728f;

// initialized during calibration (during startup)
int32_t gyroBiasX = 0;
int32_t gyroBiasY = 0;
int32_t gyroBiasZ = 0;

// GPS parser object
TinyGPSPlus gps;

float state[9] = {0}; // px, py, pz, vx, vy, vz, bax, bay, baz  (positions (m ENU), velocities (m/s), biases)

// covariance matrices for each axis (3x3, but symmetric --> 6 unique entries)
// Ppp, Ppv, Ppb, Pvv, Pvb, Pbb
float PCovX[6] = {9.0f,  0.0f, 0.0f, 0.25f, 0.0f, 0.25f};
float PCovY[6] = {9.0f,  0.0f, 0.0f, 0.25f, 0.0f, 0.50f};
float PCovZ[6] = {16.0f, 0.0f, 0.0f, 0.25f, 0.0f, 0.25f};

double pNaught[3] = {0}; // initial position (longitude, latitude, altitude (m))

float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // (w, x, y, z) tracked by Magdwick filter, utilized by Kalman filter

// -------------------------
// Stopwatches
// -------------------------
unsigned long previousImuTime;
const unsigned long imuInterval = 10; // Read IMU every (n) ms

unsigned long previousPrintTime = 0;
const unsigned long printInterval = 500; // Print to Serial every (n) ms

float dt;   // for accurate state tracking 

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for ESP32-S3 (SDA = 14, SCL = 4)
  Wire.begin(14, 4); 
  Wire.setClock(100000);
  
  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 Initialized.");

  calibrateIMU_3D();

  // Start the GPS serial connection at 9600 baud
  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  initializeGPSReference();

  previousImuTime = millis();

}

void loop() {
  unsigned long currentMillis = millis();

  // =========================================================
  // Read GPS Buffer (continuously), Update state upon new GPS info
  // =========================================================
  while (gpsSerial.available() > 0) {
    // Feed characters to the parser one by one as they arrive
    gps.encode(gpsSerial.read());
  }


  if (gps.course.isValid() && gps.course.isUpdated() && gps.speed.mps() > 2.0f)
    ahrs_correct_yaw(gps.course.deg());
    
  if (gps.hdop.hdop() < 2.0f && gps.location.isUpdated() && gps.location.isValid()) {
    kalman_update(gps.location.lng(), gps.location.lat(), gps.altitude.meters());
    
    if (gps.speed.mps() < 0.1f)
      kalman_zupt();    // zero internal velocity when stationary according to GPS

  }



  // =========================================================
  // Read MPU6050 and Update State (100 Hz)
  // =========================================================
  if (currentMillis - previousImuTime >= imuInterval) {
    dt = (currentMillis - previousImuTime)/1000.0f; 
    previousImuTime = currentMillis;

    // Point to the first data register
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    
    // Request 14 consecutive bytes
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);

    // Reconstruct the 16-bit integers
    accelX = (int16_t)(Wire.read() << 8 | Wire.read()); 
    accelY = (int16_t)(Wire.read() << 8 | Wire.read()); 
    accelZ = (int16_t)(Wire.read() << 8 | Wire.read());    
    tempRaw = (Wire.read() << 8 | Wire.read()); 
    gyroX = (int16_t)(Wire.read() << 8 | Wire.read()); 
    gyroY = (int16_t)(Wire.read() << 8 | Wire.read()); 
    gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
    
    // Calibrating IMU measurements, constants from MPU 6050 datasheet
    accelX = (accelX - accelBiasX) * accelScaleX / 16384.0f;
    accelY = (accelY - accelBiasY) * accelScaleY / 16384.0f;
    accelZ = (accelZ - accelBiasZ) * accelScaleZ / 16384.0f;   
    
    gyroX = (gyroX - gyroBiasX) / 131.0f;  // now in deg/s
    gyroY = (gyroY - gyroBiasY) / 131.0f;
    gyroZ = (gyroZ - gyroBiasZ) / 131.0f;

    ahrs_update(dt);

    kalman_predict(dt);

  }

  // =========================================================
  // Serial Print Telemetry
  // =========================================================
  if (currentMillis - previousPrintTime >= printInterval) {
    previousPrintTime = currentMillis;

    // Print IMU Data
    Serial.print("IMU -> Accel [X:"); Serial.print(accelX, 4);
    Serial.print(" Y:"); Serial.print(accelY, 4);
    Serial.print(" Z:"); Serial.print(accelZ, 4);
    Serial.print("]  Gyro [X:"); Serial.print(gyroX, 4);
    Serial.print(" Y:"); Serial.print(gyroY, 3);
    Serial.print(" Z:"); Serial.print(gyroZ, 3);
    Serial.println("]");

    Serial.print("Quat [W:"); Serial.print(quaternion[0], 4);
    Serial.print(" X:"); Serial.print(quaternion[1], 4);
    Serial.print(" Y:"); Serial.print(quaternion[2], 4);
    Serial.print(" Z:"); Serial.print(quaternion[3], 4);
    Serial.println("]");

    Serial.print("Pos [X:"); Serial.print(state[0], 4);
    Serial.print(" Y:"); Serial.print(state[1], 4);
    Serial.print(" Z:"); Serial.print(state[2], 4);
    Serial.println("]");

    Serial.print("Acceleration Biases [X:"); Serial.print(state[6], 4);
    Serial.print(" Y:"); Serial.print(state[7], 4);
    Serial.print(" Z:"); Serial.print(state[8], 4);
    Serial.println("]");
    
    // Print GPS Data
    if (gps.location.isValid()) {
      Serial.print("GPS -> Lat: "); Serial.print(gps.location.lat(), 10);
      Serial.print("  Lon: "); Serial.print(gps.location.lng(), 10);
      Serial.print("  Alt: "); Serial.print(gps.altitude.meters()); Serial.println("m");
    }
      // Velocity
    if (gps.speed.isValid() && gps.course.isValid()) {
      Serial.print("Speed: "); Serial.print(gps.speed.mps()); Serial.print(" m/s");
      Serial.print("  Course: "); Serial.print(gps.course.deg()); Serial.println(" deg");
    }
    // Signal integrity
    if (gps.satellites.isValid() && gps.hdop.isValid()) {
      Serial.print("Sats: "); Serial.print(gps.satellites.value());
      Serial.print("  HDOP: "); Serial.println(gps.hdop.hdop());
    } 
    else {
      Serial.println("GPS -> Waiting for satellite lock...");
    }

    Serial.println("--------------------------------------------------");
  }
}