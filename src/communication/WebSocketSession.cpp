#include <iostream>
#include <regex>
#include <sstream>
#include <iomanip>

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
    
    if (ws_) {
        std::cerr << "[Server] Rejecting connection - already connected" << std::endl;
        return;
    }
    
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
    std::string msg(static_cast<char*>(buffer_.data().data()), bytes);
    // std::cout << "[Server] Received message: " << msg << std::endl;
    // Verify minimum message length (flags + space)
    if (msg.size() < 4 || msg[3] != ' ') {
        std::cerr << "[Server] Invalid message format" << std::endl;
        return;
    }

    // Parse sensor flags
    bool hasMag = (msg[0] == '1');
    bool hasAccel = (msg[1] == '1');
    bool hasGyro = (msg[2] == '1');
    std::string dataStr = msg.substr(4);
    
    // Split comma-separated values
    std::vector<float> values;
    std::istringstream iss(dataStr);
    std::string token;
    while (std::getline(iss, token, ',')) {
        try {
            values.push_back(std::stof(token));
        } catch (const std::exception& e) {
            std::cerr << "[Server] Invalid float value: " << token << std::endl;
            return;
        }
    }

    // Validate data length
    size_t expectedCount = (hasGyro + hasAccel + hasMag) * 3;
    if (values.size() != expectedCount) {
        std::cerr << "[Server] Data count mismatch. Expected " 
                  << expectedCount << " values, got " << values.size() << std::endl;
        return;
    }

    // Process sensor data 
    size_t index = 0;  
    if (hasMag) {
        std::cout << std::setprecision(6) << "Mag: " << values[index] << ", " << values[index+1] << ", " << values[index+2] << std::endl;
        complementaryFilter_.updateWithMag(values[index], values[index+1], values[index+2]);
        magDataBuffer_.append(values[index], values[index+1], values[index+2]);
        magTimestamp_ += magDeltaT;
        magTimesBuffer_.append(magTimestamp_);
        index += 3;
    }
    if (hasAccel) {
        std::cout << std::setprecision(6) << "Accel: " << values[index] << ", " << values[index+1] << ", " << values[index+2] << std::endl;
        complementaryFilter_.updateWithAccel(values[index], values[index+1], values[index+2]);
        accelDataBuffer_.append(values[index], values[index+1], values[index+2]);
        accelTimestamp_ += accelDeltaT;
        accelTimesBuffer_.append(accelTimestamp_);
        index += 3;
    }
    if (hasGyro) {
        std::cout << std::setprecision(6) << "Gyro: " << values[index] << ", " << values[index+1] << ", " << values[index+2] << std::endl;
        complementaryFilter_.updateWithGyro(values[index], values[index+1], values[index+2]);
        gyroDataBuffer_.append(values[index], values[index+1], values[index+2]);
        gyroTimestamp_ += gyroDeltaT;
        gyroTimesBuffer_.append(gyroTimestamp_);
        index += 3;
    }
}
