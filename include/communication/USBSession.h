#pragma once
#include "Config.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <memory>
#include <vector>

#define SYNC_BYTE 0xAA

class ComplementaryFilter;

class USBSession : public std::enable_shared_from_this<USBSession> {
private:
    boost::asio::serial_port serial_port_;
    boost::asio::streambuf input_buffer_;
    
    struct BatchHeader {
        uint16_t sequence;
        uint8_t gyro_samples;
        uint8_t accel_samples;
        uint8_t mag_samples;
    };
    
    enum class ReadState {
        SYNC,
        HEADER,
        DATA
    };
    ReadState read_state_;
    BatchHeader current_header_ = {0, 0, 0, 0};

    // Buffer for binary reading
    std::vector<uint8_t> binary_buffer_ = std::vector<uint8_t>(200);
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

    void startReading();
    void readPacketHeader();
    void processBatch(const BatchHeader& header, const float* data);
    void readPacketData();
    void readSyncByte();

public:
    USBSession(boost::asio::io_context& ioc, const std::string& portName, 
               GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
               GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
               ComplementaryFilter& complementaryFilter);
    
    ~USBSession();
    void run();
};