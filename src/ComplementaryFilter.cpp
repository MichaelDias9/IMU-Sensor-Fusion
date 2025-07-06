#include <iostream>
#include <cmath>

#include "Config.h"
#include "ComplementaryFilter.h"

using namespace Math3D;
using namespace Structs3D;

ComplementaryFilter::ComplementaryFilter(QuaternionF& attitude, Vector3F& magVector) : attitude_(attitude), magVector_(magVector) {}

bool ComplementaryFilter::isValidGyroReading(float gyroX, float gyroY, float gyroZ) const {
    // Check for NaN or infinity
    if (!std::isfinite(gyroX) || !std::isfinite(gyroY) || !std::isfinite(gyroZ)) {
        return false;
    }
    
    // Check for extremely large values
    if (std::abs(gyroX) > MAX_GYRO_RATE || 
        std::abs(gyroY) > MAX_GYRO_RATE || 
        std::abs(gyroZ) > MAX_GYRO_RATE) {
        return false;
    }
    
    return true;
}

bool ComplementaryFilter::isValidAccelReading(float accelX, float accelY, float accelZ) const {
    // Check for NaN or infinity
    if (!std::isfinite(accelX) || !std::isfinite(accelY) || !std::isfinite(accelZ)) {
        return false;
    }
    
    // Calculate magnitude
    float magnitude = std::sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    
    // Check for reasonable magnitude (should be around 9.8 m/s^2 for gravity)
    if (magnitude < MIN_ACCEL_MAGNITUDE || magnitude > MAX_ACCEL_MAGNITUDE) {
        return false;
    }
    
    return true;
}

bool ComplementaryFilter::isValidMagReading(float magX, float magY, float magZ) const {
    // Check for NaN or infinity
    if (!std::isfinite(magX) || !std::isfinite(magY) || !std::isfinite(magZ)) {
        return false;
    }
    
    // Calculate magnitude
    float magnitude = std::sqrt(magX * magX + magY * magY + magZ * magZ);
    
    // Check for reasonable magnitude (Earth's magnetic field is ~25-65 μT)
    if (magnitude < MIN_MAG_MAGNITUDE || magnitude > MAX_MAG_MAGNITUDE) {
        return false;
    }
    
    return true;
}

void ComplementaryFilter::updateWithGyro(float gyroX, float gyroY, float gyroZ){    
    // Sanity check gyro values
    if (!isValidGyroReading(gyroX, gyroY, gyroZ)) {
        std::cout << "Warning: Invalid gyro reading detected, skipping update" << std::endl;
        return;
    }

    // Correct with proportional terms and reset. Accel and mag updates will set P terms again
    if (PTermRoll_ != 0.0f && PTermPitch_ != 0.0f) {   
        gyroX += PTermRoll_;
        gyroY += PTermPitch_;
        PTermRoll_ = 0.0f;
        PTermPitch_ = 0.0f;
    }
    if (PTermYaw_ != 0.0f) {
        gyroZ += PTermYaw_;
        PTermYaw_ = 0.0f;
    }

    // Correct gyro data with integral terms
    gyroX += ITermRoll_;
    gyroY += ITermPitch_;
    gyroZ += ITermYaw_; 

    // Update quaternion with corrected gyro data and normalize 
    quaternion_ = updateQuaternionWithAngularVelocity(quaternion_, gyroX, gyroY, gyroZ, gyroDeltaT);
    quaternion_ = normalizeQuaternion(quaternion_);

    attitude_.w = quaternion_.w;
    attitude_.x = quaternion_.x;
    attitude_.y = quaternion_.y;
    attitude_.z = quaternion_.z;
}

void ComplementaryFilter::updateWithAccel(float accelX, float accelY, float accelZ){
    // Sanity check accel values
    if (!isValidAccelReading(accelX, accelY, accelZ)) {
        std::cout << "Warning: Invalid accel reading detected, skipping update" << std::endl;
        return;
    }

    // Normalize the accel vector 
    Vector3F accelVector = normalizeVector(Vector3F(accelX, accelY, accelZ));

    // Get the expected gravity vector in the body frame using INVERSE rotation
    QuaternionF invQuaternion = conjugateQuaternion(quaternion_);
    Vector3F expectedGravityBody = rotateVectorByQuaternion(exptectedGravityWorld_, invQuaternion);

    // Get the error between the expected gravity and the measured gravity with cross product
    Vector3F error = crossProduct(accelVector, expectedGravityBody);

    // Update the correction vectors
    PTermRoll_ = KpRollPitch_ * error.x;
    PTermPitch_ = KpRollPitch_ * error.y;   
    ITermRoll_ += KiRollPitch_ * error.x * gyroDeltaT;
    ITermPitch_ += KiRollPitch_ * error.y * gyroDeltaT;
}

void ComplementaryFilter::updateWithMag(float magX, float magY, float magZ){
    // Sanity check mag values
    if (!isValidMagReading(magX, magY, magZ)) {
        std::cout << "Warning: Invalid mag reading detected, skipping update" << std::endl;
        return;
    }

    // Normalize the mag vector
    Vector3F magVector = normalizeVector(Vector3F(magX, magY, magZ));   

    // Get the down vector in the body frame using INVERSE rotation
    QuaternionF invQuaternion = conjugateQuaternion(quaternion_);
    Vector3F downBody = rotateVectorByQuaternion(Vector3F(0.0f, 0.0f, 1.0f), invQuaternion);

    //Get the measuredEast vector in the body frame using cross product with down body vector
    Vector3F measuredEastBody = crossProduct(downBody, magVector);

    // Get the expected east vector in the body frame using INVERSE rotation
    Vector3F expectedEastBody = rotateVectorByQuaternion(exptectedEastWorld_, invQuaternion);

    // Get measured east vector in the world frame using rotation
    Vector3F measuredEastWorld = rotateVectorByQuaternion(measuredEastBody, quaternion_);
    
    // Get the error between the expected east and the measured east with cross product
    Vector3F error = crossProduct(measuredEastBody, expectedEastBody);

    // Update the correction vector
    PTermYaw_ = KpYaw_ * error.z;
    ITermYaw_ += KiYaw_ * error.z * gyroDeltaT;
}