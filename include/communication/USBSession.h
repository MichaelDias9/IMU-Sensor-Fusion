#pragma once
#include "Config.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <memory>
#include <vector>


class ComplementaryFilter;

class USBSession : public std::enable_shared_from_this<USBSession> {
private:
    boost::asio::serial_port serial_port_;
    boost::asio::streambuf input_buffer_;
    
    // Binary protocol structure
    struct SensorPacket {
        uint8_t flags;
        uint8_t count;
        float data[6];
    } __attribute__((packed));
    
    // Buffer for binary reading
    std::vector<uint8_t> binary_buffer_;
    size_t bytes_needed_;
    bool reading_header_;
    
    // Data buffers and filters
    GyroBuffer& gyroDataBuffer_;
    AccelBuffer& accelDataBuffer_;
    MagBuffer& magDataBuffer_;
    GyroTimesBuffer& gyroTimesBuffer_;
    AccelTimesBuffer& accelTimesBuffer_;
    MagTimesBuffer& magTimesBuffer_;
    ComplementaryFilter& complementaryFilter_;
    
    // Timing
    float magTimestamp_ = 0.0f;
    float accelTimestamp_ = 0.0f;
    float gyroTimestamp_ = 0.0f;
    const float magDeltaT = 1.0f / 100.0f;    // 100 Hz
    const float accelDeltaT = 1.0f / 200.0f;  // 200 Hz
    const float gyroDeltaT = 1.0f / 200.0f;   // 200 Hz

    void readBinaryHeader();
    void readBinaryData();
    void processBinaryPacket(const SensorPacket& packet);

public:
    USBSession(boost::asio::io_context& ioc, const std::string& portName, 
               GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
               GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
               ComplementaryFilter& complementaryFilter);
    
    ~USBSession();
    void run();
};