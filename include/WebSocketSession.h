#pragma once
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <atomic>

#include "Config.h"
#include "ComplementaryFilter.h"

namespace beast = boost::beast;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class WebSocketSession {
public:
    WebSocketSession(net::io_context& ioc, unsigned short port, 
                     GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
                     GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
                     ComplementaryFilter& complementaryFilter);
    
    void run();
    
private:
    void handleConnection(tcp::socket socket);
    void readLoop();
    void processMessage(size_t bytes);

    tcp::acceptor acceptor_;
    std::optional<beast::websocket::stream<tcp::socket>> ws_;
    beast::flat_buffer buffer_;

    GyroBuffer& gyroDataBuffer_;
    AccelBuffer& accelDataBuffer_;
    MagBuffer& magDataBuffer_;

    GyroTimesBuffer& gyroTimesBuffer_;
    AccelTimesBuffer& accelTimesBuffer_;
    MagTimesBuffer& magTimesBuffer_;

    ComplementaryFilter& complementaryFilter_;
  
    std::chrono::steady_clock::time_point firstTimestamp_;
    bool firstDataReceived_;
};