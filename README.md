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
![Demo](media/sensor-fusion-demo.gif)

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
In your program to forward your sensor data, you must send messages with this following format:

Format: [mag_flag][accel_flag][gyro_flag] [sensor_values]

  * Flags: 3 characters ('0' or '1') indicating which sensors are included
  * Space: Required after flags
  * Data: Comma-separated float values in order: Magnetometer (x,y,z), Accelerometer (x,y,z), Gyroscope (x,y,z)

Examples:
  * 111 1.2,3.4,5.6,0.1,0.2,0.3,45.1,23.2,67.8 - All sensors
  * 010 0.1,0.2,0.3 - Accelerometer only
  * 101 1.2,3.4,5.6,45.1,23.2,67.8 - Magnetometer + Gyroscope

### Connection Details
  * USB Serial: 115200 baud, 8N1 (8 data bits, no parity, 1 stop bit)
  * WebSocket: Standard WebSocket protocol
  * Message Termination: End messages with newline character (\n) for USB serial
  

