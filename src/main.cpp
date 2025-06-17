#define _USE_MATH_DEFINES
#include <thread>
#include <iostream>
#include <boost/asio.hpp>

#include "Config.h"
#include "util/Attitude.h"
#include "communication/WebSocketSession.h"
#include "communication/USBReceiver.h"
#include "ComplementaryFilter.h"
#include "ui/RunApp.h"

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
    GyroBuffer gyroDataBuffer; AccelBuffer accelDataBuffer; MagBuffer magDataBuffer;
    GyroTimesBuffer gyroTimesBuffer; AccelTimesBuffer accelTimesBuffer; MagTimesBuffer magTimesBuffer;
    prefillBuffers(gyroDataBuffer, accelDataBuffer, magDataBuffer, gyroTimesBuffer, accelTimesBuffer, magTimesBuffer);

    // Initialize the shared attitude object
    Attitude estimatedAttitude = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 }; // Roll, Pitch, Yaw, w, x, y, z

    // Initialize the shared filter constants
    float DynamicKpRollPitch = KpRollPitch;
    float DynamicKpYaw = KpYaw;
    float DynamicKiRollPitch = KiRollPitch;
    float DynamicKiYaw = KiYaw;

    // Create the complementary filter object for estimating the attitude
    ComplementaryFilter complementaryFilter(DynamicKpRollPitch, DynamicKpYaw, DynamicKiRollPitch, DynamicKiYaw, estimatedAttitude);
    
    // Start websocket or USB thread. Receives and timestamps sensor data and triggers complementary filter updates
    std::optional<std::thread> usbThread;
    std::optional<USBReceiver> usbReceiver;

    // Start the web socket server on a separate thread
    boost::asio::io_context ioc;
    WebSocketSession server(ioc, 8000, gyroDataBuffer, accelDataBuffer, magDataBuffer, 
                                      gyroTimesBuffer, accelTimesBuffer, magTimesBuffer, complementaryFilter);

    server.run();
    std::thread socketThread([&ioc]() { ioc.run(); });

    if (connectionMode == 'u') {
        // Initialize USB receiver with buffer references
        usbReceiver.emplace(gyroDataBuffer, accelDataBuffer, magDataBuffer,
                            gyroTimesBuffer, accelTimesBuffer, magTimesBuffer, complementaryFilter);
        
        // Initialize the USB connection
        if (!usbReceiver->initialize()) {
            std::cerr << "Failed to initialize USB receiver" << std::endl;
            return -1;
        }
        
        // List available devices for debugging
        usbReceiver->listDevices();
        
        // Start USB receiving thread
        usbThread.emplace([&usbReceiver]() { usbReceiver->startReceiving(); });
    }

    // Run App Window
    runApp(gyroDataBuffer, accelDataBuffer, magDataBuffer, gyroTimesBuffer, accelTimesBuffer, magTimesBuffer, 
           estimatedAttitude, DynamicKpRollPitch, DynamicKpYaw, DynamicKiRollPitch, DynamicKiYaw);

    // Clean up on exit
    if (usbReceiver) {
        usbReceiver->stop();  // Signal USB receiver to stop
        if (usbThread && usbThread->joinable()) {
            usbThread->join();  // Wait for USB thread to finish
        }
    }
    ioc.stop();
    socketThread.join();
}