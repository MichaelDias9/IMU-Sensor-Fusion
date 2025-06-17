#include <iostream>
#include <cmath>

#include "Config.h"
#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(float& KpRollPitch, float& KpYaw, float& KiRollPitch, float& KiYaw, Attitude& attitude) :
    KpRollPitch_(KpRollPitch), KpYaw_(KpYaw), KiRollPitch_(KiRollPitch), KiYaw_(KiYaw), attitude_(attitude) {}

void ComplementaryFilter::updateWithGyro(float gyroX, float gyroY, float gyroZ){
    // Correct gyro data with integral terms and proportional terms
    gyroX -= ITermRoll_ * gyroDeltaT;
    gyroY -= ITermPitch_ * gyroDeltaT;
    gyroZ -= ITermYaw_ * gyroDeltaT;

    // Only use proportional terms once. Accel and mag readings will sets the values again when measurements are available
    if (PTermRoll_ != 0.0f && PTermPitch_ != 0.0f) {   
        gyroX -= PTermRoll_;
        gyroY -= PTermPitch_;
        PTermRoll_ = 0.0f;
        PTermPitch_ = 0.0f;
    }
    if (PTermYaw_ != 0.0f) {
        gyroZ -= PTermYaw_;
        PTermYaw_ = 0.0f;
    }

    // Update quaternion with corrected gyro data and normalize 
    quaternion_ = updateQuaternionWithAngularVelocity(quaternion_, gyroX, gyroY, gyroZ, gyroDeltaT);
    quaternion_ = normalizeQuaternion(quaternion_);

    // Calculate the new pitch, roll, and yaw from the quaternion
    float newPitch = atan2(2 * (quaternion_.w * quaternion_.z + quaternion_.x * quaternion_.y), 1 - 2 * (quaternion_.y * quaternion_.y + quaternion_.z * quaternion_.z));
    float newRoll = asin(2 * (quaternion_.w * quaternion_.x - quaternion_.y * quaternion_.z));
    float newYaw = atan2(2 * (quaternion_.w * quaternion_.y + quaternion_.x * quaternion_.z), 1 - 2 * (quaternion_.x * quaternion_.x + quaternion_.y * quaternion_.y));

    // Update the attitude with the new estimate
    attitude_.pitch = newPitch;
    attitude_.roll = newRoll;
    attitude_.yaw = newYaw;
    attitude_.w = quaternion_.w;
    attitude_.x = quaternion_.x;
    attitude_.y = quaternion_.y;
    attitude_.z = quaternion_.z;
}

void ComplementaryFilter::updateWithAccel(float accelX, float accelY, float accelZ){
    // Get the roll and pitch estimates from the accelerometer
    float roll = atan2(accelY, accelZ);
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

    // Get the expected gravity vector in the body frame
    Vector3 expectedGravityBody = rotateVectorByQuaternion(exptectedGravityWorld_, quaternion_);

    // Get the error between the expected gravity and the measured gravity with cross product
    Vector3 error = crossProduct(Vector3(accelX, accelY, accelZ), expectedGravityBody);

    // Update the correction vector 
    PTermRoll_ = KpRollPitch_ * error.x;
    PTermPitch_ = KpRollPitch_ * error.y;
    ITermRoll_ += KiRollPitch_ * error.x * gyroDeltaT;
    ITermPitch_ += KiRollPitch_ * error.y * gyroDeltaT;
}