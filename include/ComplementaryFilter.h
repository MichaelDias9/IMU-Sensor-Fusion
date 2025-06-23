#pragma once
#include "Config.h"
#include "util/Structs3D.h"
#include "util/Math3D.h"
#include "ComplementaryFilter.h"

using namespace Structs3D;

class ComplementaryFilter {
public:
    ComplementaryFilter(QuaternionF& attitude, Vector3F& magVector);
    void updateWithGyro(float gyroX, float gyroY, float gyroZ);
    void updateWithAccel(float accelX, float accelY, float accelZ);
    void updateWithMag(float magX, float magY, float magZ);

private: 
    bool running_;

    Vector3F exptectedGravityWorld_ = {0.0f, 0.0f, -1.0f};
    Vector3F exptectedEastWorld_ = {0.0f, 1.0f, 0.0f};

    QuaternionF& attitude_;
    Vector3F& magVector_;
    QuaternionF quaternion_ = {0.0f, 1.0f, 0.0f, 0.0f};

    float KpRollPitch_ = KpRollPitch;
    float KpYaw_ = KpYaw;
    float KiRollPitch_ = KiRollPitch;
    float KiYaw_ = KiYaw;

    float PTermRoll_ = 0.0f;
    float PTermPitch_ = 0.0f;
    float PTermYaw_ = 0.0f;
    float ITermRoll_ = 0.0f;
    float ITermPitch_ = 0.0f;
    float ITermYaw_ = 0.0f;            

    friend class ImGuiPanel;       
};