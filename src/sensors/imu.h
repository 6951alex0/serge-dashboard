#pragma once
// IMU 6 axes QMI8658 (integree a la carte Waveshare).

bool imu_begin();
void imu_read(float *ax, float *ay, float *az,   // g  (repere carte : x+ avant, y+ gauche, z+ haut)
              float *gx, float *gy, float *gz);  // deg/s
