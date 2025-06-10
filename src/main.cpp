#define _USE_MATH_DEFINES
#include <thread>
#include <iostream>
#include <boost/asio.hpp>

#include "Config.h"
#include "Attitude.h"

#include "WebSocketSession.h"
#include "ComplementaryFilter.h"
#include "RunApp.h"

// Function to pre-fill the buffers with empty data
void prefillBuffers(GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer, 
                    GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer){    
    // Pre-fill gyro data buffer
    std::vector<float> gyroX(gyroBufferSize, 0.0f);
    std::vector<float> gyroY(gyroBufferSize, 0.0f);
    std::vector<float> gyroZ(gyroBufferSize, 0.0f);
    gyroDataBuffer.append(gyroX.data(), gyroY.data(), gyroZ.data(), gyroBufferSize);

    // Pre-fill accel data buffer
    std::vector<float> accelX(accelBufferSize, 0.0f);
    std::vector<float> accelY(accelBufferSize, 0.0f);
    std::vector<float> accelZ(accelBufferSize, 0.0f);
    accelDataBuffer.append(accelX.data(), accelY.data(), accelZ.data(), accelBufferSize);

    // Pre-fill mag data buffer
    std::vector<float> magX(magBufferSize, 0.0f);
    std::vector<float> magY(magBufferSize, 0.0f);
    std::vector<float> magZ(magBufferSize, 0.0f);
    magDataBuffer.append(magX.data(), magY.data(), magZ.data(), magBufferSize);

    // Pre-fill time buffers
    float timeStep;

    // Pre-fill gyro time buffer
    timeStep = 1.0f / gyroFreq;
    std::array<float, gyroBufferSize> gyroTimes;
    for (int i = 0; i < gyroBufferSize; i++) {
        gyroTimes[i] = (i * timeStep) - bufferSeconds;
    }
    gyroTimesBuffer.append(gyroTimes.data(), gyroBufferSize);

    // Pre-fill accel time buffer
    timeStep = 1.0f / accelFreq;
    std::array<float, accelBufferSize> accelTimes;
    for (int i = 0; i < accelBufferSize; i++) {
        accelTimes[i] = (i * timeStep) - bufferSeconds;
    }
    accelTimesBuffer.append(accelTimes.data(), accelBufferSize);

    // Pre-fill mag time buffer
    timeStep = 1.0f / magFreq;
    std::array<float, magBufferSize> magTimes;
    for (int i = 0; i < magBufferSize; i++) {
        magTimes[i] = (i * timeStep) - bufferSeconds;
    }
    magTimesBuffer.append(magTimes.data(), magBufferSize);
}

int main() {
    // Initialize Buffers
    GyroBuffer gyroDataBuffer;
    AccelBuffer accelDataBuffer;
    MagBuffer magDataBuffer;
    GyroTimesBuffer gyroTimesBuffer;
    AccelTimesBuffer accelTimesBuffer;
    MagTimesBuffer magTimesBuffer;
    prefillBuffers(gyroDataBuffer, accelDataBuffer, magDataBuffer, gyroTimesBuffer, accelTimesBuffer, magTimesBuffer);

    // Initialize the shared attitude object
    Attitude estimatedAttitude = {0.0f, 0.0f, 0.0f, 1.0, 0.0, 0.0, 0.0}; // Roll, Pitch, Yaw, w, x, y, z

    // Create the complementary filter object for estimating the attitude
    float alpha = compFilterAlpha;
    int filterFrequency = compFilterFrequency;
    const float filterTimeDelta = 1.0f / filterFrequency;
    ComplementaryFilter complementaryFilter(alpha, filterFrequency, filterTimeDelta,
                                            gyroDataBuffer, accelDataBuffer, gyroTimesBuffer, accelTimesBuffer, 
                                            estimatedAttitude);
    
    // Start WebSocket Thread. Receives and timestamps sensor data and triggers complementary filter updates
    boost::asio::io_context ioc;
    WebSocketSession socketServer(ioc, 8000, gyroDataBuffer, accelDataBuffer, magDataBuffer, 
                                  gyroTimesBuffer, accelTimesBuffer, magTimesBuffer, complementaryFilter);
    std::thread socketThread([&ioc]() { ioc.run(); });

    // Run App Window
    runApp(gyroDataBuffer, accelDataBuffer, magDataBuffer, gyroTimesBuffer, accelTimesBuffer, magTimesBuffer, estimatedAttitude, alpha);

    // Clean up on exit
    ioc.stop();
    socketThread.join();
}