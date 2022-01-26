//#include "imu.h"
//#include <math.h>
//
//Accelerometer(ADXL345_DataRate::Rate_200Hz, ADXL345_Sensetivity::ADXL345_2g);
//
///*
//GyroData G = Gyro.Update();
//printf("X_omegadot : %f \n", G.X);
//printf("Y_omegadot : %f \n", G.Y);
//printf("Z_omegadot : %f \n", G.Z);
//
//AccelerationData AccData = Accelerometer.Update();
//printf("X_ddot : %f \n", AccData.X);
//printf("Y_ddot : %f \n", AccData.Y);
//printf("Z_ddot : %f \n", AccData.Z);
//*/
//
//float IMU::invSqrt(float number) {
//    union {
//        float f;
//        uint32_t i;
//    } conv;
//
//    float x2;
//    const float threehalfs = 1.5F;
//
//    x2 = number * 0.5F;
//    conv.f = number;
//    conv.i = 0x5f3759df - (conv.i >> 1);
//    conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f));
//    return conv.f;
//}
//
//void IMU::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
//    float recipNorm;
//    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
//    float qa, qb, qc;
//
//    // Auxiliary variables to avoid repeated arithmetic
//    q0q0 = q0 * q0;
//    q0q1 = q0 * q1;
//    q0q2 = q0 * q2;
//    q0q3 = q0 * q3;
//    q1q1 = q1 * q1;
//    q1q2 = q1 * q2;
//    q1q3 = q1 * q3;
//    q2q2 = q2 * q2;
//    q2q3 = q2 * q3;
//    q3q3 = q3 * q3;
//
//    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//    if ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
//        float halfvx, halfvy, halfvz;
//
//        // Normalise accelerometer measurement
//        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//        ax *= recipNorm;
//        ay *= recipNorm;
//        az *= recipNorm;
//
//        // Estimated direction of gravity
//        halfvx = q1q3 - q0q2;
//        halfvy = q0q1 + q2q3;
//        halfvz = q0q0 - 0.5f + q3q3;
//
//        // Error is sum of cross product between estimated direction and measured direction of field vectors
//        halfex += (ay * halfvz - az * halfvy);
//        halfey += (az * halfvx - ax * halfvz);
//        halfez += (ax * halfvy - ay * halfvx);
//    }
//    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
//    if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
//        // Compute and apply integral feedback if enabled
//        if (twoKi > 0.0f) {
//            integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
//            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
//            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
//            gx += integralFBx;  // apply integral feedback
//            gy += integralFBy;
//            gz += integralFBz;
//        }
//        else {
//            integralFBx = 0.0f; // prevent integral windup
//            integralFBy = 0.0f;
//            integralFBz = 0.0f;
//        }
//
//        // Apply proportional feedback
//        gx += twoKp * halfex;
//        gy += twoKp * halfey;
//        gz += twoKp * halfez;
//    }
//
//    // Integrate rate of change of quaternion
//    gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
//    gy *= (0.5f * (1.0f / sampleFreq));
//    gz *= (0.5f * (1.0f / sampleFreq));
//    qa = q0;
//    qb = q1;
//    qc = q2;
//    q0 += (-qb * gx - qc * gy - q3 * gz);
//    q1 += (qa * gx + qc * gz - q3 * gy);
//    q2 += (qa * gy - qb * gz + q3 * gx);
//    q3 += (qa * gz + qb * gy - qc * gx);
//
//    // Normalise quaternion
//    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//    q0 *= recipNorm;
//    q1 *= recipNorm;
//    q2 *= recipNorm;
//    q3 *= recipNorm;
//}