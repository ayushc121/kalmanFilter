#ifndef MADGWICK_H
#define MADGWICK_H

extern float quaternion[4];
extern float accelX, accelY, accelZ;
extern float gyroX, gyroY, gyroZ;

void ahrs_update(float dt);
void ahrs_correct_yaw(float gps_heading_deg);

#endif