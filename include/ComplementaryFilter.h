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

    // Sanity check limits
    static constexpr float MAX_GYRO_RATE = 35.0f;      // rad/s (about 2000 deg/s)
    static constexpr float MAX_ACCEL_MAGNITUDE = 50.0f; // m/s^2 (about 5g)
    static constexpr float MIN_ACCEL_MAGNITUDE = 0.1f;  // m/s^2 (very small but non-zero)
    static constexpr float MAX_MAG_MAGNITUDE = 100.0f;  // μT (typical Earth field is ~50μT)
    static constexpr float MIN_MAG_MAGNITUDE = 10.0f;   // μT (minimum reasonable field)

    // Sanity check helper functions
    bool isValidGyroReading(float gyroX, float gyroY, float gyroZ) const;
    bool isValidAccelReading(float accelX, float accelY, float accelZ) const;
    bool isValidMagReading(float magX, float magY, float magZ) const;

    friend class ImGuiPanel;       
};