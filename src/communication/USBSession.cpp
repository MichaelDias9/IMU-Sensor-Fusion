#define WIN32_LEAN_AND_MEAN

#include "communication/USBSession.h"
#include <iostream>
#include <sstream>
#include <iomanip>

// Windows headers LAST
#include <Windows.h>  // Keep this AFTER other includes

USBSession::USBSession(asio::io_context& ioc, const std::string& portName, 
        GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
        GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
        ComplementaryFilter& complementaryFilter)
    : 
    serial_port_(ioc),
    gyroDataBuffer_(gyroDataBuffer),
    accelDataBuffer_(accelDataBuffer),
    magDataBuffer_(magDataBuffer),
    gyroTimesBuffer_(gyroTimesBuffer),
    accelTimesBuffer_(accelTimesBuffer),
    magTimesBuffer_(magTimesBuffer),
    complementaryFilter_(complementaryFilter)
{
    boost::system::error_code ec;
    serial_port_.open(portName, ec);
    if (ec) {
        std::cerr << "[USB] Failed to open port: " << ec.message() << std::endl;
        return;
    }
    
    // First set standard options
    serial_port_.set_option(asio::serial_port_base::character_size(8));
    serial_port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    serial_port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    
    // Set custom baud rate using Windows API
    HANDLE hCom = serial_port_.native_handle();
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    
    if (!GetCommState(hCom, &dcb)) {
        std::cerr << "[USB] GetCommState failed: " << GetLastError() << std::endl;
        return;
    }
    
    dcb.BaudRate = 921600;  // Custom baud rate
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    
    if (!SetCommState(hCom, &dcb)) {
        std::cerr << "[USB] SetCommState failed: " << GetLastError() << std::endl;
        return;
    }
    
    // Verify baud rate was set
    if (!GetCommState(hCom, &dcb)) {
        std::cerr << "[USB] Verify CommState failed: " << GetLastError() << std::endl;
    } else {
        std::cout << "[USB] Actual baud rate: " << dcb.BaudRate << std::endl;
    }
    
    std::cout << "[USB] Serial port opened at 921600 baud: " << portName << std::endl;
  }


USBSession::~USBSession() {
        if (serial_port_.is_open()) {
            boost::system::error_code ec;
            serial_port_.close(ec);
            if (ec) {
                std::cerr << "[USB] Error closing port: " 
                          << ec.message() << std::endl;
            }
        }
    }

void USBSession::run() {
    if (serial_port_.is_open()) {
        readLine();
    }
}

void USBSession::readLine() {
    asio::async_read_until(serial_port_, input_buffer_, '\n',
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                // Don't log aborted operations (normal during shutdown)
                if (ec != asio::error::operation_aborted) {
                    std::cerr << "[USB] Read error: " << ec.message() << std::endl;
                }
                return;
            }
            
            std::istream is(&input_buffer_);
            std::string line;
            if (std::getline(is, line)) {
                processLine(line);
            }
            readLine(); // Continue reading
        });
}

void USBSession::processLine(const std::string& line) {
    // Verify minimum message length (flags + space)
    if (line.size() < 4 || line[3] != ' ') {
        std::cerr << "[USB] Invalid message format: " << line << std::endl;
        return;
    }

    // Print received message and timestamp
    //std::cout << "[USB] Time: " << std::fixed << std::setprecision(6) << timeInSeconds << "s" << std::endl;
    //std::cout << "[USB] Received message: " << line << std::endl;

    // Parse sensor flags
    bool hasMag = (line[0] == '1');
    bool hasAccel = (line[1] == '1');
    bool hasGyro = (line[2] == '1');
    std::string dataStr = line.substr(4);  // Skip flags and space
    
    // Split comma-separated values
    std::vector<float> values;
    std::istringstream iss(dataStr);
    std::string token;
    while (std::getline(iss, token, ',')) {
        try {
            values.push_back(std::stof(token));
        } catch (const std::exception& e) {
            std::cerr << "[USB] Invalid float value: " << token << std::endl;
            return;
        }
    }

    // Validate data length
    size_t expectedCount = (hasGyro + hasAccel + hasMag) * 3;
    if (values.size() != expectedCount) {
        std::cerr << "[USB] Data count mismatch. Expected " 
                  << expectedCount << " values, got " << values.size() 
                  << " in message: " << line << std::endl;
        return;
    }

    // Process sensor data 
    size_t index = 0;  
    if (hasMag) {
        complementaryFilter_.updateWithMag(values[index], values[index+1], values[index+2]);
        magDataBuffer_.append(values[index], values[index+1], values[index+2]);
        magTimestamp_ += magDeltaT;
        magTimesBuffer_.append(magTimestamp_);
        index += 3;
    }
    if (hasAccel) {
        complementaryFilter_.updateWithAccel(values[index], values[index+1], values[index+2]);
        accelDataBuffer_.append(values[index], values[index+1], values[index+2]);
        accelTimestamp_ += accelDeltaT;
        accelTimesBuffer_.append(accelTimestamp_);
        index += 3;
    }
    if (hasGyro) {
        complementaryFilter_.updateWithGyro(values[index], values[index+1], values[index+2]);
        gyroDataBuffer_.append(values[index], values[index+1], values[index+2]);
        gyroTimestamp_ += gyroDeltaT;
        gyroTimesBuffer_.append(gyroTimestamp_);
        index += 3;
    }
}
