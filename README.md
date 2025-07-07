# IMU Sensor Data Fusion 

A C++ project for visualizing and fusing real time gyroscope, accelerometer, and magnetometer sensor data using a PI complementary filter.

## Features 
  ### Sensors
  * Use config.h file to program any mix of sensor frequencies (I suggest gyroFreq >= accelFreq >= magFreq).
  ### Filter 
  * UI sliders for run-time tuning of proportional and integral (PI) terms of both accelerometer and magnetometer corrections.
  ### Plots 
  * Adjust plot sizes and zoom levels in both axes at run-time.  
  * Leverages downsampling to plot a large history of sensor data simultaneously (play with bufferSeconds and MAX_PLOT_POINTS).

## Demo 
![Demo](media/demo.gif?raw=true)

Note: For this example I only used accelerometer and gyroscope as the magnetometer is optional

## Usage
  * Uses CMake build generator. To build, run the following commands from the project root directory:
    * cmake -S . -B build
    * cmake --build build
  * Note: all libraries are included except for 'boost'. If you don't already have boost, then install it and add the boost home environment variable so that cmake can find it with find_package()
    
  * Run program with "build/IMUTool" from the project root.
    * Set sensor frequencies in Config.h for correct timing behaviour.
    * Decrease MAX_PLOT_POINTS in Config.h if plots or data aren't displaying properly

## Sensor Data Message Format
This program accepts sensor data messages in a specific format over USB serial or WebSocket connections.

### For Websockets
Format: Char Array - [mag_flag][accel_flag][gyro_flag] [sensor_values]

  * Flags: 3 characters ('0' or '1') indicating which sensors are included
  * Space: Required after flags
  * Data: Comma-separated float values in order: Magnetometer (x,y,z), Accelerometer (x,y,z), Gyroscope (x,y,z)

Examples:
  * 111 1.2,3.4,5.6,0.1,0.2,0.3,45.1,23.2,67.8 - All sensors
  * 010 0.1,0.2,0.3 - Accelerometer only
  * 101 1.2,3.4,5.6,45.1,23.2,67.8 - Magnetometer + Gyroscope

### For USB 
The USB protocol uses data buffering/batching to reduce overhead from repeated USB system calls.

Format: Raw Bytes - [SYNC_BYTE] [MAG_COUNT] [ACCEL_COUNT] [GYRO_COUNT] [MAG_DATA] [ACCEL_DATA] [GYRO_DATA]

Where: 
  * SYNC_BYTE = 0xAA
  * MAG_COUNT, ACCEL_COUNT, and GYRO_COUNT are uint8_t types that represent the number of samples in the batch for the respective sensor 

Example:
```cpp
  // Define struct to hold batched data
  typedef struct {
    uint8_t gyro_count;
    uint8_t accel_count;
    uint8_t mag_count;
    float gyro[8][3];     // Max 8 samples (200Hz/25Hz)
    float accel[8][3];    // Max 8 samples (200Hz/25Hz)
    float mag[2][3];      // Max 2 samples (50Hz/25Hz)    
  } SensorBatch;

  // -- Add data to struct here --

  // Build packet using stored data
    uint8_t packet[256];
    uint8_t *ptr = packet;

    // Add sync byte and packet header (order: mag, accel, gyro to match data)
    *ptr++ = SYNC_BYTE;
    *ptr++ = send_batch.mag_count;
    *ptr++ = send_batch.accel_count;
    *ptr++ = send_batch.gyro_count; 
    
    // Add mag data first
    for (int i = 0; i < send_batch.mag_count; i++) {
        memcpy(ptr, send_batch.mag[i], 3 * sizeof(float));
        ptr += 3 * sizeof(float);
    }

    // Add accel data second
    for (int i = 0; i < send_batch.accel_count; i++) {
        memcpy(ptr, send_batch.accel[i], 3 * sizeof(float));
        ptr += 3 * sizeof(float);
    }

    // Add gyro data last
    for (int i = 0; i < send_batch.gyro_count; i++) {
        memcpy(ptr, send_batch.gyro[i], 3 * sizeof(float));
        ptr += 3 * sizeof(float);
    }

    // Send packet
    size_t packet_size = ptr - packet;
    usb_send_data((const char*)packet, packet_size);
```
#### Connection Details
  * USB Serial: 115200 baud, 8N1 (8 data bits, no parity, 1 stop bit)
  * WebSocket: Standard WebSocket protocol  
