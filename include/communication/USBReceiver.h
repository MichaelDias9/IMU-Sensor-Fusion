#pragma once

#include <libusb-1.0/libusb.h>
#include <string>
#include <atomic>
#include <chrono>

#include "Config.h"

// Forward declaration
class ComplementaryFilter;

class USBReceiver {
private:
    libusb_context* ctx;
    libusb_device_handle* dev_handle;
    std::atomic<bool> running;
    
    // Buffer references
    GyroBuffer& gyroDataBuffer_;
    AccelBuffer& accelDataBuffer_;
    MagBuffer& magDataBuffer_;
    GyroTimesBuffer& gyroTimesBuffer_;
    AccelTimesBuffer& accelTimesBuffer_;
    MagTimesBuffer& magTimesBuffer_;
    ComplementaryFilter& complementaryFilter_;
    
    // Timing
    std::chrono::steady_clock::time_point firstTimestamp_;
    bool firstDataReceived_;
    
    // iPhone USB identifiers (may vary by model)
    static constexpr uint16_t APPLE_VENDOR_ID = 0x05ac;
    static constexpr uint16_t IPHONE_PRODUCT_ID = 0x12a8; // iPhone (adjust as needed)
    
    // USB endpoints
    static constexpr uint8_t BULK_IN_ENDPOINT = 0x82;
    
    // Private helper methods
    void processMessage(const std::string& msg);
    
public:
    USBReceiver(GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
                GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
                ComplementaryFilter& complementaryFilter);
    ~USBReceiver();
    
    // Disable copy constructor and assignment operator
    USBReceiver(const USBReceiver&) = delete;
    USBReceiver& operator=(const USBReceiver&) = delete;
    
    /**
     * Initialize the USB receiver
     * @return true if initialization successful, false otherwise
     */
    bool initialize();
    
    /**
     * Start receiving data from the iPhone
     * This function blocks until stop() is called
     */
    void startReceiving();
    

    
    /**
     * Stop the receiving process
     */
    void stop();
    
    /**
     * Clean up resources
     */
    void cleanup();
    
    /**
     * List all available USB devices for debugging
     */
    void listDevices();
    
    /**
     * Check if the receiver is currently running
     * @return true if receiving, false otherwise
     */
    bool isRunning() const;
    
    /**
     * Check if the device is connected and initialized
     * @return true if connected, false otherwise
     */
    bool isConnected() const;
};