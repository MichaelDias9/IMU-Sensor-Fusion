#pragma once
#include "implot.h"
#include "imgui.h"
#include "ThreadSafeRingBuffer3D.h"
#include "Config.h"
#include "SensorPlot.h"

class ImPlotPanel {
private:
    const int m_posX;
    const int m_posY;
    const int m_width;
    const int m_height;
    
    float m_vertical_zoom;   // Overall panel zoom (affects height)
    float m_horizontal_zoom; // X-axis zoom (shared across plots)
    
    SensorPlot<gyroBufferSize> m_gyroPlot;
    SensorPlot<accelBufferSize> m_accelPlot;
    SensorPlot<magBufferSize> m_magPlot;

    static constexpr float min_zoom = 0.1f;
    static constexpr float max_zoom = 10.0f;

public:
    ImPlotPanel(int posX, int posY, int width, int height, 
                const GyroBuffer& gyroBuffer_ref,
                const AccelBuffer& accelBuffer_ref, 
                const MagBuffer& magBuffer_ref,
                const GyroTimesBuffer& gyroTimeBuffer_ref,
                const AccelTimesBuffer& accelTimeBuffer_ref,
                const MagTimesBuffer& magTimeBuffer_ref
                );

    void Draw();
};