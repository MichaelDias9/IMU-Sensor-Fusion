#pragma once

#include "Config.h"
#include "util/Attitude.h"
#include "util/ThreadSafeRingBuffer3D.h"

void runApp(const GyroBuffer &gyroBuffer, 
            const AccelBuffer &accelBuffer,
            const MagBuffer &magBuffer, 
            const GyroTimesBuffer &gyroTimesBuffer,
            const AccelTimesBuffer &accelTimesBuffer,
            const MagTimesBuffer &magTimesBuffer,
            Attitude &estimatedAttitude,
            float& KpRollPitch, float& KpYaw, float& KiRollPitch, float& KiYaw);