#pragma once
#include <cstddef>

#include "util/ThreadSafeRingBuffer3D.h"
#include "util/ThreadSafeRingBuffer.h"

// Screen settings
const int screenWidth = 1280;
const int screenHeight = 800;
const int targetFPS = 60;

// Sensor frequencies
constexpr int gyroFreq = 150;
constexpr int accelFreq = 150;
constexpr int magFreq = 30;
constexpr float gyroDeltaT = 1.0f / gyroFreq;
constexpr float accelDeltaT = 1.0f / accelFreq;
constexpr float magDeltaT = 1.0f / magFreq;

// Complementary filter settings
const float KpRollPitch = 6.0f;
const float KiRollPitch = 0.1f;
const float KpYaw = 4.0f;
const float KiYaw = 0.05f;

// Plot settings
static constexpr size_t MAX_PLOT_POINTS = 1000; // ImPlot downsampling threshold
constexpr int bufferSeconds = 5;                // Length of data history to keep

// Set buffer sizes for aliases based on sensor frequencies and data history length
constexpr std::size_t gyroBufferSize = gyroFreq * bufferSeconds;
constexpr std::size_t accelBufferSize = accelFreq * bufferSeconds;
constexpr std::size_t magBufferSize = magFreq * bufferSeconds;
using GyroBuffer = ThreadSafeRingBuffer3D<gyroBufferSize>;
using AccelBuffer = ThreadSafeRingBuffer3D<accelBufferSize>;
using MagBuffer = ThreadSafeRingBuffer3D<magBufferSize>;
using GyroTimesBuffer = ThreadSafeRingBuffer<gyroBufferSize>;
using AccelTimesBuffer = ThreadSafeRingBuffer<accelBufferSize>;
using MagTimesBuffer = ThreadSafeRingBuffer<magBufferSize>;