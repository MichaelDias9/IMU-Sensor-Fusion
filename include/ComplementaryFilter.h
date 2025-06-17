#pragma once
#include "Config.h"
#include "util/Attitude.h"
#include "util/Quaternion.h"
#include "ComplementaryFilter.h"

class ComplementaryFilter {
public:
    ComplementaryFilter(float& KpRollPitch, float& KpYaw, float& KiRollPitch, float& KiYaw, Attitude& attitude);
    void updateWithGyro(float gyroX, float gyroY, float gyroZ);
    void updateWithAccel(float accelX, float accelY, float accelZ);

private: 
    bool running_;

    Vector3 exptectedGravityWorld_ = {0.0f, 0.0f, -1.0f};
    Vector3 exptectedNorthWorld_ = {1.0f, 0.0f, 0.0f};

    Attitude& attitude_;
    Quaternion quaternion_ = {1.0f, 0.0f, 0.0f, 0.0f};

    const float& KpRollPitch_;
    const float& KpYaw_;
    const float& KiRollPitch_;
    const float& KiYaw_;  

    float PTermRoll_ = 0.0f;
    float PTermPitch_ = 0.0f;
    float PTermYaw_ = 0.0f;
    float ITermRoll_ = 0.0f;
    float ITermPitch_ = 0.0f;
    float ITermYaw_ = 0.0f;                   
};