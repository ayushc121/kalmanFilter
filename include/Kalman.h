#ifndef KALMAN_H
#define KALMAN_H

extern float state[9];
extern double pNaught[3];
extern float quaternion[4];
extern float PCovX[6];
extern float PCovY[6];
extern float PCovZ[6];

extern float accelX, accelY, accelZ;
    
void kalman_predict(float dt);
void kalman_update(float gps_lng, float gps_lat, float gps_alt);
void kalman_zupt();


#endif