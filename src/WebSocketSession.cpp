#include <iostream>
#include <regex>
#include <sstream>

#include "WebSocketSession.h"
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
    bool foundData = false;
    
    // Current timestamp
    auto now = std::chrono::steady_clock::now();
    
    // Set first timestamp if this is the first data
    if (!firstDataReceived_) {
        firstTimestamp_ = now;
        firstDataReceived_ = true;
        std::cout << "[Server] First data received - timestamp set" << std::endl;
    }
    
    // Calculate time in seconds since first data
    auto duration = now - firstTimestamp_;
    float timeInSeconds = std::chrono::duration<float>(duration).count();

    // Check if complementary filter needs to be updated
    if (timeInSeconds > complementaryFilter_.currentTime_ + complementaryFilter_.deltaTime_) {
        complementaryFilter_.update();
    }

    // Verify minimum message length (flags + space)
    if (msg.size() < 4 || msg[3] != ' ') {
        std::cerr << "[Server] Invalid message format" << std::endl;
        return;
    }

    // Parse sensor flags
    bool hasGyro = (msg[0] == '1');
    bool hasAccel = (msg[1] == '1');
    bool hasMag = (msg[2] == '1');
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

    // Process sensor data in order
    size_t index = 0;
    if (hasGyro) {
        gyroDataBuffer_.append(&values[index], &values[index+1], &values[index+2], 1);
        gyroTimesBuffer_.append(timeInSeconds);
        index += 3;
        foundData = true;
    }
    if (hasAccel) {
        accelDataBuffer_.append(&values[index], &values[index+1], &values[index+2], 1);
        accelTimesBuffer_.append(timeInSeconds);
        index += 3;
        foundData = true;
    }
    if (hasMag) {
        magDataBuffer_.append(&values[index], &values[index+1], &values[index+2], 1);
        magTimesBuffer_.append(timeInSeconds);
        foundData = true;
    }

    if (!foundData) {
        std::cerr << "[Server] No valid sensor data found in message" << std::endl;
    }
}