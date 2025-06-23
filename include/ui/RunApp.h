#pragma once

#include "Config.h"
#include "util/Structs3D.h"
#include "util/ThreadSafeRingBuffer3D.h"
#include "ComplementaryFilter.h"

void runApp(const GyroBuffer &gyroBuffer, 
            const AccelBuffer &accelBuffer,
            const MagBuffer &magBuffer, 
            const GyroTimesBuffer &gyroTimesBuffer,
            const AccelTimesBuffer &accelTimesBuffer,
            const MagTimesBuffer &magTimesBuffer,
            Structs3D::QuaternionF &estimatedAttitude,
            Structs3D::Vector3F& accelVector,
            ComplementaryFilter &complementaryFilter);