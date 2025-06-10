#include <iostream>
#include <cmath>

#include "Config.h"
#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(float& alpha, int& filterFrequency, float deltaTime,
                                       GyroBuffer& gyroBuffer, AccelBuffer& accelBuffer,
                                       GyroTimesBuffer& gyroTimeBuffer, AccelTimesBuffer& accelTimeBuffer,
                                       Attitude& attitude) 
                                       :
                                        filterFrequency_(filterFrequency), deltaTime_(deltaTime), alpha_(alpha),
                                        gyroBuffer_(gyroBuffer), accelBuffer_(accelBuffer),
                                        gyroTimeBuffer_(gyroTimeBuffer), accelTimeBuffer_(accelTimeBuffer),
                                        attitude_(attitude) {}

void ComplementaryFilter::update(){
    // Retrieve the newest indices
    std::size_t lastestGyroIndex = gyroBuffer_.getHead() - 1;
    std::size_t latestAccelIndex = accelBuffer_.getHead() - 1;

    // Retrieve newest timestamps
    float gyroMostRecentTime = gyroTimeBuffer_.at(lastestGyroIndex);
    float accelMostRecentTime = accelTimeBuffer_.at(latestAccelIndex);

    // Back up indeces to ensure only using data from before current filter time
    while(gyroMostRecentTime > currentTime_) {
        gyroMostRecentTime = gyroTimeBuffer_.at(--lastestGyroIndex);
    }
    while(accelMostRecentTime > currentTime_) {
        accelMostRecentTime = accelTimeBuffer_.at(--latestAccelIndex);
    }

    // Get the number of contributing samples
    std::size_t gyroContributingSamples = lastestGyroIndex - previousGyroIndex_;
    std::size_t accelContributingSamples = latestAccelIndex - previousAccelIndex_;

    // Calculate time contributions
    const float timeContributionOfNewestAccelData = currentTime_ - accelMostRecentTime;
    const float timeContributionOfOldestAccelData = (accelTimeBuffer_.at(previousAccelIndex_+1)) - (currentTime_ - deltaTime_);
    const float timeContributionOfNewestGyroData = currentTime_ - gyroMostRecentTime;
    const float timeContributionOfOldestGyroData = (gyroTimeBuffer_.at(previousGyroIndex_+1)) - (currentTime_ - deltaTime_);

    // Delta time without partial data 
    const float deltaTimeWithoutPartialDataAccel = deltaTime_ - (timeContributionOfNewestAccelData + timeContributionOfOldestAccelData);
    const float deltaTimeWithoutPartialDataGyro = deltaTime_ - (timeContributionOfNewestGyroData + timeContributionOfOldestGyroData);

    logFilterDebugInfo(latestAccelIndex, accelMostRecentTime, timeContributionOfNewestAccelData, timeContributionOfOldestAccelData, deltaTimeWithoutPartialDataAccel);

    // Get weighted average of accelerometer readings 
    Vector3 avgAccel = calculateWeightedAccelAverage(latestAccelIndex, timeContributionOfNewestAccelData, timeContributionOfOldestAccelData);
    std::cout << "Avg accel: " << avgAccel.x << ", " << avgAccel.y << ", " << avgAccel.z << std::endl;

    // Integrate the gyroscope readings with proper reference frame conversion
    Vector3 angularChanges = integrateWeightedGyroReadingsInReferenceFrame(
        lastestGyroIndex, timeContributionOfNewestGyroData, timeContributionOfOldestGyroData);

    float pitch_gyro = attitude_.pitch + angularChanges.y; // Reference frame pitch change
    float roll_gyro = attitude_.roll + angularChanges.x;   // Reference frame roll change

    // Get accelerometer angles using averaged accelerometer readings
    float pitch_accel = atan2(avgAccel.y, sqrt(avgAccel.x * avgAccel.x + avgAccel.z * avgAccel.z));
    float roll_accel = atan2(-avgAccel.x, sqrt(avgAccel.y * avgAccel.y + avgAccel.z * avgAccel.z));

    // Compute magnitude of accelerometer reading
    float accelMagnitude = sqrt(avgAccel.x * avgAccel.x + avgAccel.y * avgAccel.y + avgAccel.z * avgAccel.z);
    
    // Dynamically adjust alpha if acceleromter reading is biased
    std::cout << "accelMagnitude" << accelMagnitude << '\n';
    float adjustedAlpha = alpha_;
    if (accelMagnitude > 1.5f) { 
        adjustedAlpha = alpha_ + 0.8f * (1.0f - alpha_);  // Push alpha 80% closer to 1
    }

    // Run the filter calculations using gyro and accel angles
    float pitch = adjustedAlpha * pitch_gyro + (1 - adjustedAlpha) * pitch_accel;
    float roll = adjustedAlpha * roll_gyro + (1 - adjustedAlpha) * roll_accel;

    // Update the attitude with the new estimate
    attitude_.pitch = pitch;
    attitude_.roll = roll;

    // Increment time and indeces for next iteration
    currentTime_ = currentTime_ + deltaTime_;
    previousGyroIndex_ = lastestGyroIndex;
    previousAccelIndex_ = latestAccelIndex;
}

Vector3 ComplementaryFilter::bodyToReferenceFrameRates(const Vector3& bodyRates, float currentPitch, float currentRoll) {
    Vector3 referenceRates;
    
    float sinRoll = sin(currentRoll);
    float cosRoll = cos(currentRoll);
    float sinPitch = sin(currentPitch);
    float cosPitch = cos(currentPitch);
    float tanPitch = tan(currentPitch);
    
    // Transformation matrix from body rates to Euler angle rates
    // [roll_rate ]   [1   sin(φ)tan(θ)   cos(φ)tan(θ)] [ωx]
    // [pitch_rate] = [0   cos(φ)        -sin(φ)      ] [ωy]
    // [yaw_rate  ]   [0   sin(φ)/cos(θ)  cos(φ)/cos(θ)] [ωz]
    
    referenceRates.x = bodyRates.x + sinRoll * tanPitch * bodyRates.y + cosRoll * tanPitch * bodyRates.z;  // roll rate
    referenceRates.y = cosRoll * bodyRates.y - sinRoll * bodyRates.z;  // pitch rate
    referenceRates.z = (sinRoll / cosPitch) * bodyRates.y + (cosRoll / cosPitch) * bodyRates.z;  // yaw rate
    
    return referenceRates;
}

Vector3 ComplementaryFilter::integrateWeightedGyroReadingsInReferenceFrame(std::size_t latestGyroIndex,
                                                                            float timeContributionOfNewestGyroData,
                                                                            float timeContributionOfOldestGyroData) {
    Vector3 integratedAngles = {0.0f, 0.0f, 0.0f};
    
    // For proper integration, we need to convert body rates to reference frame rates at each time step
    // and then integrate. However, since the attitude changes during integration, we'll use a simplified
    // approach with small time steps or assume small angle changes.
    
    // Current attitude for transformation
    float currentPitch = attitude_.pitch;
    float currentRoll = attitude_.roll;
    
    // Integrate oldest gyro data (partial time contribution at start of interval)
    Vector3 oldestBodyRates = {gyroBuffer_.atX(previousGyroIndex_), 
                              gyroBuffer_.atY(previousGyroIndex_), 
                              gyroBuffer_.atZ(previousGyroIndex_)};
    Vector3 oldestRefRates = bodyToReferenceFrameRates(oldestBodyRates, currentPitch, currentRoll);
    
    integratedAngles.x += oldestRefRates.x * timeContributionOfOldestGyroData;
    integratedAngles.y += oldestRefRates.y * timeContributionOfOldestGyroData;
    integratedAngles.z += oldestRefRates.z * timeContributionOfOldestGyroData;
    
    // Integrate full-interval gyro data (complete time steps between partial contributions)
    for(std::size_t i = previousGyroIndex_ + 1; i < latestGyroIndex; ++i) {
        float timeBetweenGyroReadings = gyroTimeBuffer_.at(i + 1) - gyroTimeBuffer_.at(i);
        
        Vector3 bodyRates = {gyroBuffer_.atX(i), gyroBuffer_.atY(i), gyroBuffer_.atZ(i)};
        Vector3 refRates = bodyToReferenceFrameRates(bodyRates, currentPitch, currentRoll);
        
        integratedAngles.x += refRates.x * timeBetweenGyroReadings;
        integratedAngles.y += refRates.y * timeBetweenGyroReadings;
        integratedAngles.z += refRates.z * timeBetweenGyroReadings;
        
        // For better accuracy, we could update currentPitch/currentRoll here with the accumulated changes
        // but for small time steps, this approximation should be sufficient
    }
    
    // Integrate newest gyro data (partial time contribution at end of interval)
    Vector3 newestBodyRates = {gyroBuffer_.atX(latestGyroIndex), 
                              gyroBuffer_.atY(latestGyroIndex), 
                              gyroBuffer_.atZ(latestGyroIndex)};
    Vector3 newestRefRates = bodyToReferenceFrameRates(newestBodyRates, currentPitch, currentRoll);
    
    integratedAngles.x += newestRefRates.x * timeContributionOfNewestGyroData;
    integratedAngles.y += newestRefRates.y * timeContributionOfNewestGyroData;
    integratedAngles.z += newestRefRates.z * timeContributionOfNewestGyroData;
    
    return integratedAngles;
}

Vector3 ComplementaryFilter::calculateWeightedAccelAverage(std::size_t latestAccelIndex,
                                                           float timeContributionOfNewestAccelData,
                                                           float timeContributionOfOldestAccelData) {
    Vector3 avgAccel = {0.0f, 0.0f, 0.0f};
    
    // Add contribution of oldest accel data
    avgAccel.x += accelBuffer_.atX(previousAccelIndex_) * timeContributionOfOldestAccelData;
    avgAccel.y += accelBuffer_.atY(previousAccelIndex_) * timeContributionOfOldestAccelData;
    avgAccel.z += accelBuffer_.atZ(previousAccelIndex_) * timeContributionOfOldestAccelData;
    
    // Add contribution of non-interval crossing accel data
    for(std::size_t i = previousAccelIndex_ + 1; i < latestAccelIndex; ++i) {
        float timeBetweenAccelReadings = accelTimeBuffer_.at(i + 1) - accelTimeBuffer_.at(i);
        avgAccel.x += accelBuffer_.atX(i) * timeBetweenAccelReadings;
        avgAccel.y += accelBuffer_.atY(i) * timeBetweenAccelReadings;
        avgAccel.z += accelBuffer_.atZ(i) * timeBetweenAccelReadings;
    }
    
    // Add contribution of newest accel data
    avgAccel.x += accelBuffer_.atX(latestAccelIndex) * timeContributionOfNewestAccelData;
    avgAccel.y += accelBuffer_.atY(latestAccelIndex) * timeContributionOfNewestAccelData;
    avgAccel.z += accelBuffer_.atZ(latestAccelIndex) * timeContributionOfOldestAccelData;

    // Scale back up by frequency
    avgAccel.x *= filterFrequency_;
    avgAccel.y *= filterFrequency_;
    avgAccel.z *= filterFrequency_;
    
    return avgAccel;
}

void ComplementaryFilter::logFilterDebugInfo(std::size_t latestAccelIndex, float accelMostRecentTime,
                                             float timeContributionOfNewestAccelData,
                                             float timeContributionOfOldestAccelData,
                                             float deltaTimeWithoutPartialDataAccel) const {
    // Get buffer details for accelerometer
    std::size_t accelBufferStart = latestAccelIndex - 3;
    std::size_t accelBufferEnd = latestAccelIndex + 1;
    
    std::cout << "\n--- Filter Interval [" << currentTime_ - deltaTime_ << " to " << currentTime_ << "]:\n";
    std::cout << "Delta time: " << deltaTime_ << "\n";
    
    // Log buffer range (handle circular buffer wrap-around)
    for (std::size_t i = accelBufferStart; i <= accelBufferEnd; ++i) {
        std::cout << "  Index " << i << ": Time = " << accelTimeBuffer_.at(i) 
                  << (i == previousAccelIndex_ ? " <-- PREV" : "")
                  << (i == latestAccelIndex ? " <-- CURRENT" : "") << "\n";
    }
    
    // Log contribution of newest accel data
    std::cout << "Contribution of newest accel data: " << 
    currentTime_ << " - " << accelMostRecentTime << " = " << timeContributionOfNewestAccelData << "\n";
    
    // Log contribution of oldest accel data
    std::cout << "Contribution of oldest accel data: " << 
    accelTimeBuffer_.at(latestAccelIndex - 1) << " - " << (currentTime_ - deltaTime_) << " = " <<
    timeContributionOfOldestAccelData << "\n";
    
    // Log the delta time without partial data
    std::cout << "Delta time without partial data: " << deltaTimeWithoutPartialDataAccel << "\n";
    
    // Log the delta time without partial data plus the contribution of the newest accel data
    std::cout << "Delta time without partial data plus both contributions: " << 
    deltaTimeWithoutPartialDataAccel + timeContributionOfNewestAccelData + timeContributionOfOldestAccelData << "\n";
}