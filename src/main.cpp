#define _USE_MATH_DEFINES
#include <thread>
#include <iostream>
#include <boost/asio.hpp>

#include "Config.h"
#include "util/Structs3D.h"
#include "communication/WebSocketSession.h"
#include "communication/USBSession.h"
#include "ComplementaryFilter.h"
#include "ui/RunApp.h"

// Function to get the appropriate serial port name for each platform
std::string getDefaultSerialPort() {
#ifdef _WIN32
    return "COM9";  // Windows COM port
#elif defined(__APPLE__)
    return "/dev/tty.usbserial-0001";  // macOS USB serial port (adjust as needed)
#elif defined(__linux__)
    return "/dev/ttyUSB0";  // Linux USB serial port
#else
    return "";
#endif
}

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
    Structs3D::QuaternionF estimatedAttitude = { 1.0, 0.0, 0.0, 0.0 }; // w, x, y, z

    // Initialize the shared acceleration vector
    Structs3D::Vector3F accelVector = { 0.0, 0.0, 0.0 };

    // Create the complementary filter object for estimating the attitude
    ComplementaryFilter complementaryFilter(estimatedAttitude, accelVector);

    // Start the communication session on a separate thread based on the selected mode
    std::shared_ptr<void> sessionHolder;    // Create a shared_ptr to keep session alive
    boost::asio::io_context ioc;            // IO context for the communication session
    
    if (UseWebSocket) {
        auto server = std::make_shared<WebSocketSession>(ioc, 8000, 
            gyroDataBuffer, accelDataBuffer, magDataBuffer,
            gyroTimesBuffer, accelTimesBuffer, magTimesBuffer, 
            complementaryFilter);
        server->run();
        sessionHolder = server; // Keep alive
    } else {
        // USB mode - now supported on all platforms
        std::string portName = getDefaultSerialPort();
        if (portName.empty()) {
            std::cerr << "USB mode not supported on this platform" << std::endl;
            return 1;
        }
        
        std::cout << "Using serial port: " << portName << std::endl;
        std::cout << "Note: You may need to adjust the port name in getDefaultSerialPort() for your specific device" << std::endl;
        
        auto usb = std::make_shared<USBSession>(ioc, portName,
            gyroDataBuffer, accelDataBuffer, magDataBuffer,
            gyroTimesBuffer, accelTimesBuffer, magTimesBuffer,
            complementaryFilter);
        usb->run();
        sessionHolder = usb; // Keep alive
    }

    std::thread ioThread([&ioc]() { ioc.run(); });

    // Run App Window
    runApp(gyroDataBuffer, accelDataBuffer, magDataBuffer, gyroTimesBuffer, accelTimesBuffer, magTimesBuffer, 
           estimatedAttitude, accelVector, complementaryFilter);

    // Clean up on exit
    ioc.stop();
    ioThread.join();
    
    return 0;
}