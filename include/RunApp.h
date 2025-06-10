#pragma once

#include "Config.h"
#include "Attitude.h"
#include "ThreadSafeRingBuffer3D.h"

void runApp(const GyroBuffer &gyroBuffer, 
            const AccelBuffer &accelBuffer,
            const MagBuffer &magBuffer, 
            const GyroTimesBuffer &gyroTimesBuffer,
            const AccelTimesBuffer &accelTimesBuffer,
            const MagTimesBuffer &magTimesBuffer,
            Attitude &estimatedAttitude,
            float& alpha_);