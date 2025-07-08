#include <iostream>
#include <regex>
#include <sstream>
#include <iomanip>
#include <cstring>

#include "communication/WebSocketSession.h"
#include "Config.h"

#include "ComplementaryFilter.h"

WebSocketSession::WebSocketSession(net::io_context& ioc, unsigned short port, 
            GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
            GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
            ComplementaryFilter& complementaryFilter)
    : 
    acceptor_(ioc, {tcp::v4(), port}),
    gyroDataBuffer_(gyroDataBuffer), accelDataBuffer_(accelDataBuffer), magDataBuffer_(magDataBuffer),
    gyroTimesBuffer_(gyroTimesBuffer), accelTimesBuffer_(accelTimesBuffer), magTimesBuffer_(magTimesBuffer),
    complementaryFilter_(complementaryFilter) {
    std::cout << "[Server] WebSocket server started on port " << port << std::endl;
    run();
}

/**
 * Start accepting incoming connections. This function will block until a
 * connection is accepted, then it will call handleConnection() to handle the
 * connection. If handleConnection() returns, this function will be called again
 * to continue listening for connections.
 */
void WebSocketSession::run() {
    acceptor_.async_accept(
        [this](beast::error_code ec, tcp::socket socket) {
            if (!ec) handleConnection(std::move(socket));
            run();  // Keep listening for connections
        });
}

void WebSocketSession::handleConnection(tcp::socket socket) {
    std::cout << "[Server] New connection attempt" << std::endl;
    
    //if (ws_) {
    //    std::cerr << "[Server] Rejecting connection - already connected" << std::endl;
    ///    return;
    //}
    
    std::cout << "[Server] Connection accepted" << std::endl;
    ws_.emplace(std::move(socket));
    ws_->async_accept([this](beast::error_code ec) {
        if (!ec) {
            std::cout << "[Server] WebSocket handshake successful" << std::endl;
            readLoop();
        } else {
            std::cerr << "[Server] Handshake error: " << ec.message() << std::endl;
            ws_.reset();
        }
    });
}

void WebSocketSession::readLoop() {
    ws_->async_read(buffer_,
        [this](beast::error_code ec, size_t bytes) {
            if (ec) return ws_.reset();
            
            processMessage(bytes);
            buffer_.consume(bytes);
            readLoop();
        });
}

void WebSocketSession::processMessage(size_t bytes) {
    // Get raw binary data
    const uint8_t* data = static_cast<const uint8_t*>(buffer_.data().data());
    
    // Verify minimum message length (sync + flags)
    if (bytes < 2) {
        std::cerr << "[Server] Message too short" << std::endl;
        return;
    }
    
    // Check sync byte
    if (data[0] != 0xAA) {
        std::cerr << "[Server] Invalid sync byte: 0x" << std::hex << (int)data[0] << std::endl;
        return;
    }
    
    // Parse flags
    uint8_t flags = data[1];
    bool hasMag = (flags & 0x04) != 0;
    bool hasAccel = (flags & 0x02) != 0;
    bool hasGyro = (flags & 0x01) != 0;
    
    // Calculate expected payload size
    size_t numFloats = (hasMag ? 3 : 0) + (hasAccel ? 3 : 0) + (hasGyro ? 3 : 0);
    size_t expectedSize = 2 + (numFloats * 4); // 2 header bytes + float data
    
    if (bytes != expectedSize) {
        std::cerr << "[Server] Size mismatch. Expected " << expectedSize 
                  << " bytes, got " << bytes << std::endl;
        return;
    }
    
    // Extract float values from binary data
    std::vector<float> values;
    size_t offset = 2; // Skip sync and flags
    
    for (size_t i = 0; i < numFloats; i++) {
        float val;
        memcpy(&val, &data[offset], 4);
        values.push_back(val);
        offset += 4;
    }
    
    // Process sensor data in order: mag, accel, gyro
    size_t index = 0;
    
    if (hasMag) {
        // std::cout << std::setprecision(6) << "Mag: " << values[index] << ", " 
        //          << values[index+1] << ", " << values[index+2] << std::endl;
        complementaryFilter_.updateWithMag(values[index], values[index+1], values[index+2]);
        magDataBuffer_.append(values[index], values[index+1], values[index+2]);
        magTimestamp_ += magDeltaT;
        magTimesBuffer_.append(magTimestamp_);
        index += 3;
    }
    
    if (hasAccel) {
        // std::cout << std::setprecision(6) << "Accel: " << values[index] << ", " 
        //          << values[index+1] << ", " << values[index+2] << std::endl;
        complementaryFilter_.updateWithAccel(values[index], values[index+1], values[index+2]);
        accelDataBuffer_.append(values[index], values[index+1], values[index+2]);
        accelTimestamp_ += accelDeltaT;
        accelTimesBuffer_.append(accelTimestamp_);
        index += 3;
    }
    
    if (hasGyro) {
        // std::cout << std::setprecision(6) << "Gyro: " << values[index] << ", " 
        //          << values[index+1] << ", " << values[index+2] << std::endl;
        complementaryFilter_.updateWithGyro(values[index], values[index+1], values[index+2]);
        gyroDataBuffer_.append(values[index], values[index+1], values[index+2]);
        gyroTimestamp_ += gyroDeltaT;
        gyroTimesBuffer_.append(gyroTimestamp_);
        index += 3;
    }
}