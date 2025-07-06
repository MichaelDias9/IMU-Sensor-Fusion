#include "communication/USBSession.h"
#include "ComplementaryFilter.h"
#include <iostream>
#include <cstring>

USBSession::USBSession(boost::asio::io_context& ioc, const std::string& portName, 
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
    complementaryFilter_(complementaryFilter),
    bytes_needed_(2), // Start by reading 2-byte header
    reading_header_(true)
{
    boost::system::error_code ec;
    serial_port_.open(portName, ec);
    if (ec) {
        std::cerr << "[USB] Failed to open port: " << ec.message() << std::endl;
        return;
    }
    
    // Set standard options
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    
    // Initialize binary buffer
    binary_buffer_.resize(sizeof(SensorPacket));
    
    std::cout << "[USB] Serial port opened at 115200 baud: " << portName << std::endl;
}

USBSession::~USBSession() {
    if (serial_port_.is_open()) {
        boost::system::error_code ec;
        serial_port_.close(ec);
        if (ec) {
            std::cerr << "[USB] Error closing port: " << ec.message() << std::endl;
        }
    }
}

void USBSession::run() {
    if (serial_port_.is_open()) {
        readBinaryHeader();
    }
}

void USBSession::readBinaryHeader() {
    reading_header_ = true;
    bytes_needed_ = 2; // flags + count
    
    boost::asio::async_read(serial_port_, 
        boost::asio::buffer(binary_buffer_.data(), bytes_needed_),
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                if (ec != boost::asio::error::operation_aborted) {
                    std::cerr << "[USB] Header read error: " << ec.message() << std::endl;
                }
                return;
            }
            
            // Parse header
            uint8_t flags = binary_buffer_[0];
            uint8_t count = binary_buffer_[1];
            
            // Validate count
            if (count > 6) {
                std::cerr << "[USB] Invalid data count: " << (int)count << std::endl;
                readBinaryHeader(); // Try again
                return;
            }
            
            // Store header in buffer
            binary_buffer_[0] = flags;
            binary_buffer_[1] = count;
            
            // Calculate bytes needed for data
            bytes_needed_ = count * sizeof(float);
            
            if (bytes_needed_ > 0) {
                readBinaryData();
            } else {
                readBinaryHeader(); // No data, read next header
            }
        });
}

void USBSession::readBinaryData() {
    reading_header_ = false;
    
    // Read float data into buffer after header
    boost::asio::async_read(serial_port_, 
        boost::asio::buffer(binary_buffer_.data() + 2, bytes_needed_),
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                if (ec != boost::asio::error::operation_aborted) {
                    std::cerr << "[USB] Data read error: " << ec.message() << std::endl;
                }
                return;
            }
            
            // Process complete packet
            const SensorPacket* packet = reinterpret_cast<const SensorPacket*>(binary_buffer_.data());
            processBinaryPacket(*packet);
            
            // Read next packet
            readBinaryHeader();
        });
}

void USBSession::processBinaryPacket(const SensorPacket& packet) {
    // Parse sensor flags - bit positions: 0=mag, 1=accel, 2=gyro
    bool hasMag = (packet.flags & 0x01) != 0;    // bit 0
    bool hasAccel = (packet.flags & 0x02) != 0;  // bit 1  
    bool hasGyro = (packet.flags & 0x04) != 0;   // bit 2
    
    /* Debug output
    std::cout << "[USB] Flags: 0x" << std::hex << (int)packet.flags << std::dec 
              << " (mag=" << hasMag << ", accel=" << hasAccel << ", gyro=" << hasGyro 
              << "), count=" << (int)packet.count << std::endl;
    */
    // Validate expected count
    size_t expectedCount = (hasGyro ? 3 : 0) + (hasAccel ? 3 : 0) + (hasMag ? 3 : 0);
    if (packet.count != expectedCount) {
        std::cerr << "[USB] Data count mismatch. Expected " 
                  << expectedCount << " values, got " << (int)packet.count << std::endl;
        return;
    }
    
    // Process data in order: mag, accel, gyro
    size_t index = 0;
    if (hasMag) {
        complementaryFilter_.updateWithMag(packet.data[index], packet.data[index+1], packet.data[index+2]);
        magDataBuffer_.append(packet.data[index], packet.data[index+1], packet.data[index+2]);
        magTimestamp_ += magDeltaT;
        magTimesBuffer_.append(magTimestamp_);
        index += 3;
    }
    if (hasAccel) {
        complementaryFilter_.updateWithAccel(packet.data[index], packet.data[index+1], packet.data[index+2]);
        accelDataBuffer_.append(packet.data[index], packet.data[index+1], packet.data[index+2]);
        accelTimestamp_ += accelDeltaT;
        accelTimesBuffer_.append(accelTimestamp_);
        index += 3;
    }
    if (hasGyro) {
        complementaryFilter_.updateWithGyro(packet.data[index], packet.data[index+1], packet.data[index+2]);
        gyroDataBuffer_.append(packet.data[index], packet.data[index+1], packet.data[index+2]);
        gyroTimestamp_ += gyroDeltaT;
        gyroTimesBuffer_.append(gyroTimestamp_);
        index += 3;
    }
}