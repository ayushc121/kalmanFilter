#include "Kalman.h"
#include "QuatFxns.h"
#include <cmath>
#include <Arduino.h>

// helper function to update any 6-element covariance array
void updateCovariance(float* PCov, float Q_pp, float Q_pv, float Q_vv, float Q_bb, float dt) {
    float h = 0.5f * dt * dt;

    float Ppp = PCov[0];
    float Ppv = PCov[1];
    float Ppb = PCov[2];
    float Pvv = PCov[3];
    float Pvb = PCov[4];
    float Pbb = PCov[5];

    // compute and assign  new values directly back to the array
    PCov[0] = Ppp + 2*dt*Ppv - 2*h*Ppb + dt*dt*Pvv - 2*dt*h*Pvb + h*h*Pbb + Q_pp;
    PCov[1] = Ppv + dt*Pvv - h*Pvb - dt*Ppb - dt*dt*Pvb + dt*h*Pbb + Q_pv;
    PCov[2] = Ppb + dt*Pvb - h*Pbb;
    PCov[3] = Pvv - 2*dt*Pvb + dt*dt*Pbb + Q_vv;
    PCov[4] = Pvb - dt*Pbb;
    PCov[5] = Pbb + Q_bb;
}

void kalman_predict(float dt) {

    // solving for acceleration in world coordinates rather than body
    float QuatAccel[4] = {0, accelX, accelY, accelZ};
    float w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];
    float qInv[4] = {w, -x, -y, -z}; 

    float tmp[4];

    multiplyQuaternions(quaternion, QuatAccel, tmp);
    multiplyQuaternions(tmp, qInv, QuatAccel);

    // getting accelerations in m/s^2
    float worldAx = QuatAccel[1];
    float worldAy = QuatAccel[2];
    float worldAz = QuatAccel[3] - 1.0f;  // remove gravity
    const float G = 9.81f;
    float ax = worldAx - state[6];   // subtracting biases (in m/s^2)
    float ay = worldAy - state[7];
    float az = worldAz - state[8];    
    
    // updating positions and velocities
    state[0] += state[3]*dt + 0.5f*ax*dt*dt;
    state[1] += state[4]*dt + 0.5f*ay*dt*dt;
    state[2] += state[5]*dt + 0.5f*az*dt*dt;

    state[3] += ax*dt;
    state[4] += ay*dt;
    state[5] += az*dt;

    // updating the covariance matrix for each axis (each axis has its own 3x3 matrix)
    float sigmaAx = 0.05f;
    float sigmaAy = 0.05f;
    float sigmaAz = 0.05f;
    float sigmaBx = 0.0005f;
    float sigmaBy = 0.0005f;
    float sigmaBz = 0.0002f;

    // Q matrices for each axis
    float Q_pp_x = sigmaAx * sigmaAx * (dt*dt*dt*dt / 4.0f);
    float Q_pv_x = sigmaAx * sigmaAx * (dt*dt*dt / 2.0f);
    float Q_vv_x = sigmaAx * sigmaAx * (dt*dt);
    float Q_bb_x = sigmaBx * sigmaBx * dt;

    float Q_pp_y = sigmaAy * sigmaAy * (dt*dt*dt*dt / 4.0f);
    float Q_pv_y = sigmaAy * sigmaAy * (dt*dt*dt / 2.0f);
    float Q_vv_y = sigmaAy * sigmaAy * (dt*dt);
    float Q_bb_y = sigmaBy * sigmaBy * dt;

    float Q_pp_z = sigmaAz * sigmaAz * (dt*dt*dt*dt / 4.0f);
    float Q_pv_z = sigmaAz * sigmaAz * (dt*dt*dt / 2.0f);
    float Q_vv_z = sigmaAz * sigmaAz * (dt*dt);
    float Q_bb_z = sigmaBz * sigmaBz * dt;

    updateCovariance(PCovX, Q_pp_x, Q_pv_x, Q_vv_x, Q_bb_x, dt);
    updateCovariance(PCovY, Q_pp_y, Q_pv_y, Q_vv_y, Q_bb_y, dt);
    updateCovariance(PCovZ, Q_pp_z, Q_pv_z, Q_vv_z, Q_bb_z, dt);
}


double deg2rad(double deg) {
    return deg * (M_PI / 180.0);
}


void update_axis(int p_idx, int v_idx, int b_idx, float* PCov, float gps_p, float R) {

    float y  = gps_p - state[p_idx];       // innovation
    float S  = PCov[0] + R;                 // Ppp + R
    float Kp = PCov[0] / S;                 // Kalman gains
    float Kv = PCov[1] / S;
    float Kb = PCov[2] / S;

    state[p_idx] += Kp * y;                 // state update
    state[v_idx] += Kv * y;
    state[b_idx] += Kb * y;

    float new_Ppp = (1-Kp) * PCov[0];      // covariance update
    float new_Ppv = (1-Kp) * PCov[1];
    float new_Ppb = (1-Kp) * PCov[2];
    float new_Pvv =  PCov[3] - Kv * PCov[1];
    float new_Pvb =  PCov[4] - Kv * PCov[2];
    float new_Pbb =  PCov[5] - Kb * PCov[2];

    PCov[0] = new_Ppp;  
    PCov[1] = new_Ppv;  
    PCov[2] = new_Ppb;
    PCov[3] = new_Pvv;  
    PCov[4] = new_Pvb;  
    PCov[5] = new_Pbb;
}


void kalman_update(float gps_lng, float gps_lat, float gps_alt) {

    const double EARTH_RADIUS = 6378137.0; // meters

    double d_lon = deg2rad(gps_lng - pNaught[0]);
    double d_lat = deg2rad(gps_lat - pNaught[1]);

    float east = static_cast<float>(EARTH_RADIUS * d_lon * cos(deg2rad(pNaught[1])));
    float north = static_cast<float>(EARTH_RADIUS * d_lat);
    float up = static_cast<float>(gps_alt - pNaught[2]);

    float R_east  = 25.0f;   // 5m std dev — GPS showed 12m horizontal drift
    float R_north = 25.0f;
    float R_up    = 100.0f;  // 10m std dev — GPS altitude is unreliable

    update_axis(0, 3, 6, PCovX, east,  R_east);   // px, vx, bax
    update_axis(1, 4, 7, PCovY, north, R_north);  // py, vy, bay
    update_axis(2, 5, 8, PCovZ, up,    R_up);     // pz, vz, baz

}


void kalman_zupt() {
    float R_zupt = 0.1f;

    auto zupt_axis = [&](int v_idx, float* PCov) {
        float y  = -state[v_idx];           // innovation: 0 - v
        float S  = PCov[3] + R_zupt;        // Pvv + R
        float Kv = PCov[3] / S;             // gain on velocity only

        state[v_idx] += Kv * y;             // correct velocity

        PCov[3] = (1 - Kv) * PCov[3];      // shrink Pvv
        PCov[4] = (1 - Kv) * PCov[4];      // update Pvb for consistency
        
        // Ppp, Ppv, Ppb, Pbb left alone
        // ZUPT was causing these values to increase too much
    
    };

    zupt_axis(3, PCovX);
    zupt_axis(4, PCovY);
    zupt_axis(5, PCovZ);
}