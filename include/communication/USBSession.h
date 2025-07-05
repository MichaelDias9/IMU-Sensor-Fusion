#pragma once

#include <boost/asio.hpp>

#include "Config.h"
#include "ComplementaryFilter.h"

namespace asio = boost::asio;

class USBSession {
public:
    USBSession(asio::io_context& ioc, const std::string& portName, 
               GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
               GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
               ComplementaryFilter& complementaryFilter);
    
    ~USBSession();
    
    void run();
    
private:
    void readLine();
    void processLine(const std::string& line);

    asio::serial_port serial_port_;
    asio::streambuf input_buffer_;

    GyroBuffer& gyroDataBuffer_;
    AccelBuffer& accelDataBuffer_;
    MagBuffer& magDataBuffer_;

    GyroTimesBuffer& gyroTimesBuffer_;
    AccelTimesBuffer& accelTimesBuffer_;
    MagTimesBuffer& magTimesBuffer_;

    float gyroTimestamp_ = 0.0f;
    float accelTimestamp_ = 0.0f;
    float magTimestamp_ = 0.0f;

    ComplementaryFilter& complementaryFilter_;
};