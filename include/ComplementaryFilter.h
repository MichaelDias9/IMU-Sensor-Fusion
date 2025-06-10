#pragma once
#include "Config.h"
#include "Attitude.h"
#include "ComplementaryFilter.h"

struct Vector3 {
    float x, y, z;
};

struct Quaternion {
    float w, x, y, z;
};

class ComplementaryFilter {
public:
    ComplementaryFilter(float& alpha, int& filterFrequency, float filterTimeDelta,
                       GyroBuffer& gyroBuffer, AccelBuffer& accelBuffer,
                       GyroTimesBuffer& gyroTimeBuffer, AccelTimesBuffer& accelTimeBuffer,
                       Attitude& attitude);
    void update();

private: 
    Attitude& attitude_;
    
    GyroBuffer& gyroBuffer_;
    AccelBuffer& accelBuffer_;
    GyroTimesBuffer& gyroTimeBuffer_;
    AccelTimesBuffer& accelTimeBuffer_;

    bool running_;

    float& alpha_;
    const float filterFrequency_;
    const float deltaTime_;
    float currentTime_ = 0.0f;
    
    std::size_t previousGyroIndex_ = gyroBufferSize - 1;
    std::size_t previousAccelIndex_ = accelBufferSize - 1;    

    Vector3 bodyToReferenceFrameRates(const Vector3& bodyRates, float currentPitch, float currentRoll);

    Vector3 calculateWeightedAccelAverage(std::size_t latestAccelIndex,
                                          float timeContributionOfNewestAccelData,
                                          float timeContributionOfOldestAccelData);

    Vector3 integrateWeightedGyroReadingsInReferenceFrame(std::size_t latestGyroIndex,
                                                          float timeContributionOfNewestGyroData,
                                                          float timeContributionOfOldestGyroData);


    void logFilterDebugInfo(std::size_t latestAccelIndex, float accelMostRecentTime,
                            float timeContributionOfNewestAccelData,
                            float timeContributionOfOldestAccelData,
                            float deltaTimeWithoutPartialDataAccel) const;                                          

    friend class WebSocketSession;                                  
};