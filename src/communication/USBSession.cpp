#include "communication/USBSession.h"
#include "ComplementaryFilter.h"
#include <iostream>
#include <cstring>
#include <vector>
#include <algorithm>

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
    
    // Initialize state and binary buffer
    read_state_ = ReadState::SYNC;
    binary_buffer_.resize(200);
    
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
        startReading();
    }
}

void USBSession::startReading() {
    switch (read_state_) {
        case ReadState::SYNC:
            readSyncByte();
            break;
        case ReadState::HEADER:
            readPacketHeader();
            break;
        case ReadState::DATA:
            readPacketData();
            break;
    }
}

void USBSession::readSyncByte() {
    auto self(shared_from_this());
    boost::asio::async_read(serial_port_, 
        boost::asio::buffer(binary_buffer_.data(), 1),
        [this, self](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                std::cerr << "[USB] Sync read error: " << ec.message() << std::endl;
                return;
            }
            
            if (binary_buffer_[0] == SYNC_BYTE) {
                read_state_ = ReadState::HEADER;
                readPacketHeader();
            } else {
                // Not sync byte, keep searching
                readSyncByte();
            }
        });
}

void USBSession::processBatch(const BatchHeader& header, const float* data) {
    if (!data && (header.gyro_samples > 0 || header.accel_samples > 0 || header.mag_samples > 0)) {
        std::cerr << "[USB] Warning: No data to process but samples indicated" << std::endl;
        return;
    }
    
    // Debug output
    std::cout << "[USB] Processing batch: gyro=" << (int)header.gyro_samples 
              << ", accel=" << (int)header.accel_samples 
              << ", mag=" << (int)header.mag_samples;  

    // Parse data from the buffer (order: mag -> accel -> gyro)
    const float* data_ptr = data;
    
    // Extract mag data (now supports multiple samples)
    struct MagSample {
        float x, y, z;
        float timestamp;
    };
    std::vector<MagSample> magSamples;
    
    for (int i = 0; i < header.mag_samples; i++) {
        magSamples.push_back({data_ptr[0], data_ptr[1], data_ptr[2], magTimestamp_});
        magTimestamp_ += magDeltaT;
        data_ptr += 3;
    }
    
    // Extract accel data
    struct AccelSample {
        float x, y, z;
        float timestamp;
    };
    std::vector<AccelSample> accelSamples;
    
    for (int i = 0; i < header.accel_samples; i++) {
        accelSamples.push_back({data_ptr[0], data_ptr[1], data_ptr[2], accelTimestamp_});
        accelTimestamp_ += accelDeltaT;
        data_ptr += 3;
    }
    
    // Extract gyro data
    struct GyroSample {
        float x, y, z;
        float timestamp;
    };
    std::vector<GyroSample> gyroSamples;
    
    for (int i = 0; i < header.gyro_samples; i++) {
        gyroSamples.push_back({data_ptr[0], data_ptr[1], data_ptr[2], gyroTimestamp_});
        gyroTimestamp_ += gyroDeltaT;
        data_ptr += 3;
    }
    
    // Create a combined sorted list of all samples by timestamp
    struct SensorSample {
        enum Type { MAG, ACCEL, GYRO } type;
        float x, y, z;
        float timestamp;
        int index; // Index within the specific sensor type
    };
    
    std::vector<SensorSample> allSamples;
    
    // Add all samples to the combined list
    for (size_t i = 0; i < magSamples.size(); i++) {
        allSamples.push_back({SensorSample::MAG, magSamples[i].x, magSamples[i].y, magSamples[i].z, magSamples[i].timestamp, (int)i});
    }
    for (size_t i = 0; i < accelSamples.size(); i++) {
        allSamples.push_back({SensorSample::ACCEL, accelSamples[i].x, accelSamples[i].y, accelSamples[i].z, accelSamples[i].timestamp, (int)i});
    }
    for (size_t i = 0; i < gyroSamples.size(); i++) {
        allSamples.push_back({SensorSample::GYRO, gyroSamples[i].x, gyroSamples[i].y, gyroSamples[i].z, gyroSamples[i].timestamp, (int)i});
    }
    
    // Sort by timestamp to restore temporal order
    std::sort(allSamples.begin(), allSamples.end(), 
              [](const SensorSample& a, const SensorSample& b) {
                  return a.timestamp < b.timestamp;
              });
    
    // Process samples in temporal order for proper filtering
    for (const auto& sample : allSamples) {
        switch (sample.type) {
            case SensorSample::MAG:
                magDataBuffer_.append(sample.x, sample.y, sample.z);
                magTimesBuffer_.append(sample.timestamp);
                complementaryFilter_.updateWithMag(sample.x, sample.y, sample.z);
                if (sample.index == 0) { // Only print first sample to avoid spam
                    std::cout << "[USB] Mag: " << sample.x << ", " << sample.y << ", " << sample.z << std::endl;
                }
                break;
                
            case SensorSample::ACCEL:
                accelDataBuffer_.append(sample.x, sample.y, sample.z);
                accelTimesBuffer_.append(sample.timestamp);
                complementaryFilter_.updateWithAccel(sample.x, sample.y, sample.z);
                if (sample.index == 0) { // Only print first sample to avoid spam
                    std::cout << "[USB] Accel: " << sample.x << ", " << sample.y << ", " << sample.z << std::endl;
                }
                break;
                
            case SensorSample::GYRO:
                gyroDataBuffer_.append(sample.x, sample.y, sample.z);
                gyroTimesBuffer_.append(sample.timestamp);
                complementaryFilter_.updateWithGyro(sample.x, sample.y, sample.z);
                if (sample.index == 0) { // Only print first sample to avoid spam
                    std::cout << "[USB] Gyro: " << sample.x << ", " << sample.y << ", " << sample.z << std::endl;
                }
                break;
        }
    }
}

void USBSession::readPacketHeader() {
    auto self(shared_from_this());
    // Read fixed-size header (3 bytes after sync) 
    boost::asio::async_read(serial_port_, 
        boost::asio::buffer(binary_buffer_.data(), 3),
        [this, self](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                std::cerr << "[USB] Header read error: " << ec.message() << std::endl;
                read_state_ = ReadState::SYNC;
                startReading();
                return;
            }
            
            // Parse header (order: mag, accel, gyro)
            uint8_t mag_samples = binary_buffer_[0];
            uint8_t accel_samples = binary_buffer_[1];
            uint8_t gyro_samples = binary_buffer_[2];
            
            // Simple validation
            if (gyro_samples > 7 || accel_samples > 7 || mag_samples > 7) {
                std::cerr << "[USB] Invalid header: mag=" << (int)mag_samples 
                          << ", accel=" << (int)accel_samples 
                          << ", gyro=" << (int)gyro_samples << ". Resyncing..." << std::endl;
                read_state_ = ReadState::SYNC;
                startReading();
                return;
            }
            
            // Store header info for data reading
            current_header_ = {
                .mag_samples = mag_samples,
                .accel_samples = accel_samples,
                .gyro_samples = gyro_samples
            };
            
            // Calculate data size: (gyro + accel + mag) * 3 floats each * 4 bytes per float
            bytes_needed_ = (gyro_samples * 3 + accel_samples * 3 + mag_samples * 3) * sizeof(float);
            
            if (bytes_needed_ > 0) {
                read_state_ = ReadState::DATA;
                readPacketData();
            } else {
                // No data, process empty packet
                processBatch(current_header_, nullptr);
                read_state_ = ReadState::SYNC;
                startReading();
            }
        });
}

void USBSession::readPacketData() {
    auto self(shared_from_this());
    boost::asio::async_read(serial_port_, 
        boost::asio::buffer(binary_buffer_.data(), bytes_needed_),
        [this, self](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                std::cerr << "[USB] Data read error: " << ec.message() << std::endl;
                read_state_ = ReadState::SYNC;
                startReading();
                return;
            }
            
            // Verify we got the expected number of bytes
            if (bytes_transferred != bytes_needed_) {
                std::cerr << "[USB] Data size mismatch: expected " << bytes_needed_ 
                          << ", got " << bytes_transferred << std::endl;
                read_state_ = ReadState::SYNC;
                startReading();
                return;
            }
            
            // Process the received data
            const float* float_data = reinterpret_cast<const float*>(binary_buffer_.data());
            processBatch(current_header_, float_data);
            
            // Return to sync state
            read_state_ = ReadState::SYNC;
            startReading();
        });
}