#pragma once
#include <cstddef>

#include "ThreadSafeRingBuffer3D.h"
#include "ThreadSafeRingBuffer.h"

const int screenWidth = 1280;
const int screenHeight = 800;
const int targetFPS = 60;

// Downsampling threshold (max points per plot)
static constexpr size_t MAX_PLOT_POINTS = 1000; 

constexpr int gyroFreq = 150;
constexpr int accelFreq = 150;
constexpr int magFreq = 30;
constexpr int bufferSeconds = 5;

const int compFilterFrequency = 100;
const float compFilterAlpha = 0.88;

constexpr std::size_t gyroBufferSize = gyroFreq * bufferSeconds;
constexpr std::size_t accelBufferSize = accelFreq * bufferSeconds;
constexpr std::size_t magBufferSize = magFreq * bufferSeconds;

using GyroBuffer = ThreadSafeRingBuffer3D<gyroBufferSize>;
using AccelBuffer = ThreadSafeRingBuffer3D<accelBufferSize>;
using MagBuffer = ThreadSafeRingBuffer3D<magBufferSize>;

using GyroTimesBuffer = ThreadSafeRingBuffer<gyroBufferSize>;
using AccelTimesBuffer = ThreadSafeRingBuffer<accelBufferSize>;
using MagTimesBuffer = ThreadSafeRingBuffer<magBufferSize>;