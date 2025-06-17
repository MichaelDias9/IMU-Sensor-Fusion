#include "communication/USBReceiver.h"
#include "ComplementaryFilter.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>
#include <sstream>

USBReceiver::USBReceiver(GyroBuffer& gyroDataBuffer, AccelBuffer& accelDataBuffer, MagBuffer& magDataBuffer,
                         GyroTimesBuffer& gyroTimesBuffer, AccelTimesBuffer& accelTimesBuffer, MagTimesBuffer& magTimesBuffer,
                         ComplementaryFilter& complementaryFilter)
    : ctx(nullptr), dev_handle(nullptr), running(false),
      gyroDataBuffer_(gyroDataBuffer), accelDataBuffer_(accelDataBuffer), magDataBuffer_(magDataBuffer),
      gyroTimesBuffer_(gyroTimesBuffer), accelTimesBuffer_(accelTimesBuffer), magTimesBuffer_(magTimesBuffer),
      complementaryFilter_(complementaryFilter), firstDataReceived_(false) {
}

USBReceiver::~USBReceiver() {
    cleanup();
}

bool USBReceiver::initialize() {
    // Initialize libusb
    int r = libusb_init(&ctx);
    if (r < 0) {
        std::cerr << "Failed to initialize libusb: " << libusb_error_name(r) << std::endl;
        return false;
    }
    
    // Set debug level
    libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    
    // Find and open the iPhone device
    dev_handle = libusb_open_device_with_vid_pid(ctx, APPLE_VENDOR_ID, IPHONE_PRODUCT_ID);
    if (!dev_handle) {
        std::cerr << "Cannot find iPhone device. Make sure it's connected and recognized." << std::endl;
        return false;
    }
    
    // Detach kernel driver if active
    if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
        std::cout << "Kernel driver active, detaching..." << std::endl;
        if (libusb_detach_kernel_driver(dev_handle, 0) != 0) {
            std::cerr << "Could not detach kernel driver" << std::endl;
            return false;
        }
    }
    
    // Claim interface
    r = libusb_claim_interface(dev_handle, 0);
    if (r < 0) {
        std::cerr << "Cannot claim interface: " << libusb_error_name(r) << std::endl;
        return false;
    }
    
    std::cout << "[USB] Device initialized successfully" << std::endl;
    return true;
}

void USBReceiver::startReceiving() {
    if (!dev_handle) {
        std::cerr << "[USB] Device not initialized" << std::endl;
        return;
    }
    
    running = true;
    std::cout << "[USB] Starting to receive data..." << std::endl;
    
    const int BUFFER_SIZE = 1024;
    unsigned char buffer[BUFFER_SIZE];
    int actual_length;
    
    while (running) {
        // Receive data from iPhone
        int r = libusb_bulk_transfer(dev_handle, BULK_IN_ENDPOINT, 
                                   buffer, BUFFER_SIZE, &actual_length, 1000);
        
        if (r == 0 && actual_length > 0) {
            // Data received successfully
            std::string received_data(reinterpret_cast<char*>(buffer), actual_length);
            
            // Process the sensor data (same format as WebSocket)
            processMessage(received_data);
            
        } else if (r == LIBUSB_ERROR_TIMEOUT) {
            // Timeout - continue waiting
            continue;
        } else {
            std::cerr << "[USB] Transfer error: " << libusb_error_name(r) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::cout << "[USB] Stopped receiving data" << std::endl;
}

void USBReceiver::processMessage(const std::string& msg) {
    bool foundData = false;
    
    // Current timestamp
    auto now = std::chrono::steady_clock::now();
    
    // Set first timestamp if this is the first data
    if (!firstDataReceived_) {
        firstTimestamp_ = now;
        firstDataReceived_ = true;
        std::cout << "[USB] First data received - timestamp set" << std::endl;
    }
    
    // Calculate time in seconds since first data
    auto duration = now - firstTimestamp_;
    float timeInSeconds = std::chrono::duration<float>(duration).count();

    // Verify minimum message length (flags + space)
    if (msg.size() < 4 || msg[3] != ' ') {
        std::cerr << "[USB] Invalid message format" << std::endl;
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
            std::cerr << "[USB] Invalid float value: " << token << std::endl;
            return;
        }
    }

    // Validate data length
    size_t expectedCount = (hasGyro + hasAccel + hasMag) * 3;
    if (values.size() != expectedCount) {
        std::cerr << "[USB] Data count mismatch. Expected " 
                  << expectedCount << " values, got " << values.size() << std::endl;
        return;
    }

    // Process sensor data in order accel, mag, gyro
    size_t index = 3;
    if (hasAccel) {
        // Update complementary filter
        complementaryFilter_.updateWithAccel(values[index], values[index+1], values[index+2]);
        accelDataBuffer_.append(values[index], values[index+1], values[index+2]);
        accelTimesBuffer_.append(timeInSeconds);
        index += 3;
        foundData = true;
    }
    if (hasMag) {
        magDataBuffer_.append(values[index], values[index+1], values[index+2]);
        magTimesBuffer_.append(timeInSeconds);
        index = 0;
        foundData = true;
    }
    if (hasGyro) {
        // Update complementary filter
        complementaryFilter_.updateWithGyro(values[index], values[index+1], values[index+2]);
        gyroDataBuffer_.append(values[index], values[index+1], values[index+2]);
        gyroTimesBuffer_.append(timeInSeconds);
        foundData = true;
    }

    if (!foundData) {
        std::cerr << "[USB] No valid sensor data found in message" << std::endl;
    } else {
        std::cout << "[USB] Processed sensor data at time: " << timeInSeconds << "s" << std::endl;
    }
}



void USBReceiver::stop() {
    running = false;
    std::cout << "[USB] Stop signal sent" << std::endl;
}

void USBReceiver::cleanup() {
    // Stop receiving first
    running = false;
    
    if (dev_handle) {
        libusb_release_interface(dev_handle, 0);
        libusb_close(dev_handle);
        dev_handle = nullptr;
        std::cout << "[USB] Device handle closed" << std::endl;
    }
    
    if (ctx) {
        libusb_exit(ctx);
        ctx = nullptr;
        std::cout << "[USB] libusb context cleaned up" << std::endl;
    }
}

void USBReceiver::listDevices() {
    if (!ctx) {
        std::cerr << "[USB] libusb context not initialized" << std::endl;
        return;
    }
    
    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);
    
    if (cnt < 0) {
        std::cerr << "[USB] Failed to get device list" << std::endl;
        return;
    }
    
    std::cout << "[USB] Available USB devices:" << std::endl;
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) continue;
        
        std::cout << "Device " << i << ": VID=0x" << std::hex << desc.idVendor 
                 << " PID=0x" << desc.idProduct << std::dec;
        
        // Check if this is an Apple device
        if (desc.idVendor == APPLE_VENDOR_ID) {
            std::cout << " (Apple Device)";
        }
        
        std::cout << std::endl;
    }
    
    libusb_free_device_list(devs, 1);
}

bool USBReceiver::isRunning() const {
    return running;
}

bool USBReceiver::isConnected() const {
    return dev_handle != nullptr;
}