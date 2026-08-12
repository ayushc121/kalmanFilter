#include "Madgwick.h"
#include "QuatFxns.h"
#include <cmath>

#define beta 0.15

void ahrs_update(float dt) {

    float w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];

    float radX = gyroX * M_PI / 180.0f;
    float radY = gyroY * M_PI / 180.0f;
    float radZ = gyroZ * M_PI / 180.0f;

    float rates[4] = {0.0f, radX, radY, radZ};

    // finding delta quaternion due to gyroscope
    float quatdot_gyro[4];
    multiplyQuaternions(quaternion, rates, quatdot_gyro);
    for (int i = 0; i < 4; i++) {
        quatdot_gyro[i] *= 0.5f;
    }

    // Normalize accelerometer reading so it's a unit vector (direction only)
    float ax = accelX, ay = accelY, az = accelZ;
    float a_norm = sqrtf(ax*ax + ay*ay + az*az);

    if (a_norm > 0.0f) {
        ax /= a_norm;
        ay /= a_norm;
        az /= a_norm;

        // Rotates world-frame gravity into sensor frame, giving predicted accelerometer reading
        float qstar[4] = {quaternion[0], -x, -y, -z};
        float worldgrav[4] = {0.0f, 0.0f, 0.0f, 1.0f};

        float tmp[4];
        float f[4];

        multiplyQuaternions(qstar, worldgrav, tmp);
        multiplyQuaternions(tmp, quaternion, f);

        // f is the difference between predicted and measured acceleration
        f[1] -= ax;
        f[2] -= ay;
        f[3] -= az;

        // negative grad f is the direction we want to move the orientation (minimize difference)
        float gradient[4];
        gradient[0] = -2.0f*y*f[1] + 2.0f*x*f[2];
        gradient[1] =  2.0f*z*f[1] + 2.0f*w*f[2] - 4.0f*x*f[3];
        gradient[2] = -2.0f*w*f[1] + 2.0f*z*f[2] - 4.0f*y*f[3];
        gradient[3] =  2.0f*x*f[1] + 2.0f*y*f[2];
        normalize(gradient);

        // dynamic beta so that less accurate accelerometer measurements don't hurt orientation accuracy

        float a_error = fabsf(a_norm - 1.0f);  // 0 at rest, grows with linear accel
        float effective_beta = beta * fmaxf(0.0f, 1.0f - a_error / 0.25f);

        // Subtract accelerometer correction from gyro rate
        for (int i = 0; i < 4; i++) {
            quatdot_gyro[i] -= effective_beta * gradient[i];
        }
    }

    // Integrate and renormalise
    for (int i = 0; i < 4; i++) {
        quaternion[i] += quatdot_gyro[i] * dt;
    }
    normalize(quaternion);
}


void ahrs_correct_yaw(float gps_heading_deg) {

    float w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];

    float yawRad = atan2(2.0f * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), 
                     1.0f - 2.0f * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]));

    float yawDeg = yawRad * (180.0f / M_PI);

    if (yawDeg < 0.0f) {
        yawDeg += 360.0f;
    }

    float error = gps_heading_deg - yawDeg;

    // prevent wrapping
    if (error > 180.0f)  error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    float correction = error * 0.05;

    float halfCorRad = correction * M_PI/360;     // convert to radians, divide by 2

    // applying quaternion offset
    float w_off = cos(halfCorRad);
    float z_off = sin(halfCorRad);

    quaternion[0] = (w_off * w) - (z_off * z);
    quaternion[1] = (w_off * x) - (z_off * y);
    quaternion[2] = (w_off * y) + (z_off * x);
    quaternion[3] = (w_off * z) + (z_off * w);

    normalize(quaternion);

}
