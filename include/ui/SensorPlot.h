#pragma once
#include "ThreadSafeRingBuffer3D.h"
#include "Config.h"
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <functional>
#include <algorithm>
#include "implot.h"

template <size_t Capacity>
class SensorPlot {
private:
    std::string m_name;
    const ThreadSafeRingBuffer3D<Capacity>& m_data_buffer_ref;
    const ThreadSafeRingBuffer<Capacity>& m_time_buffer_ref;
    mutable float m_y_min; 
    mutable float m_y_max; 

public:
    SensorPlot(const std::string name, const ThreadSafeRingBuffer3D<Capacity>& dataBuffer, const ThreadSafeRingBuffer<Capacity>& timeBuffer, float yMin=-1.1f, float yMax=1.1f)
        : m_name(name), m_data_buffer_ref(dataBuffer), m_time_buffer_ref(timeBuffer),
          m_y_min(yMin), m_y_max(yMax) {}

    void Draw(float height, float XZoom) const {
        if (ImPlot::BeginPlot(m_name.c_str(), ImVec2(-1, height))) {
            const float *xDataPtr, *yDataPtr, *zDataPtr, *timestampsPtr;
            m_data_buffer_ref.getRecentPointers(Capacity, &xDataPtr, &yDataPtr, &zDataPtr);
            m_time_buffer_ref.getRecentPointer(Capacity, &timestampsPtr);

            if (xDataPtr && yDataPtr && zDataPtr && timestampsPtr) {
                // Get the newest timestamp
                float max_time = timestampsPtr[Capacity - 1];

                // Set time range to show bufferSeconds behind the newest timestamp
                float x_min = max_time - bufferSeconds / XZoom;
                float x_max = max_time;

                // Configure axes
                ImPlot::SetupAxes("Time (s)", "Value");
                ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, m_y_min, m_y_max, ImGuiCond_Once);

                // Handle Y-axis zoom 
                if (ImPlot::IsPlotHovered() && !ImGui::GetIO().KeyCtrl && !ImGui::GetIO().KeyShift) {
                    float wheel = ImGui::GetIO().MouseWheel;
                    if (wheel != 0.0f) {
                        float delta = wheel * 0.1f; // Sensitivity adjustment
                        float center = (m_y_min + m_y_max) * 0.5f;
                        float range = m_y_max - m_y_min;
                        float new_range = range * (1.0f - delta);
                        new_range = std::clamp(new_range, 0.1f, 100.0f);
                        m_y_min = center - new_range * 0.5f;
                        m_y_max = center + new_range * 0.5f;
                    }
                }

                // Plot data 
                if (Capacity <= MAX_PLOT_POINTS) {
                    ImPlot::PlotLine("X", timestampsPtr, xDataPtr, Capacity);
                    ImPlot::PlotLine("Y", timestampsPtr, yDataPtr, Capacity);
                    ImPlot::PlotLine("Z", timestampsPtr, zDataPtr, Capacity);
                } else {
                    // Downsample by taking every nth point
                    size_t step = Capacity / MAX_PLOT_POINTS;
                    std::vector<float> sampledTime, sampledX, sampledY, sampledZ;
                    sampledTime.reserve(MAX_PLOT_POINTS);
                    sampledX.reserve(MAX_PLOT_POINTS);
                    sampledY.reserve(MAX_PLOT_POINTS);
                    sampledZ.reserve(MAX_PLOT_POINTS);

                    for (size_t i = 0; i < Capacity; i += step) {
                        sampledTime.push_back(timestampsPtr[i]);
                        sampledX.push_back(xDataPtr[i]);
                        sampledY.push_back(yDataPtr[i]);
                        sampledZ.push_back(zDataPtr[i]);
                    }

                    ImPlot::PlotLine("X", sampledTime.data(), sampledX.data(), sampledTime.size());
                    ImPlot::PlotLine("Y", sampledTime.data(), sampledY.data(), sampledTime.size());
                    ImPlot::PlotLine("Z", sampledTime.data(), sampledZ.data(), sampledTime.size());                    
                }
            }
            ImPlot::EndPlot();
        }
    }
};