#include "ImPlotPanel.h"

ImPlotPanel::ImPlotPanel(int posX, int posY, int width, int height, 
                         const GyroBuffer& gyroDataBuffer_ref,
                         const AccelBuffer& accelDataBuffer_ref, 
                         const MagBuffer& magDataBuffer_ref,
                         const GyroTimesBuffer& gyroTimeBuffer_ref,
                         const AccelTimesBuffer& accelTimeBuffer_ref,
                         const MagTimesBuffer& magTimeBuffer_ref
                         )
                        :
                         m_posX(posX), m_posY(posY), m_width(width), m_height(height),
                         m_vertical_zoom(1.0f), m_horizontal_zoom(1.0f),
                         m_gyroPlot("Gyro", gyroDataBuffer_ref, gyroTimeBuffer_ref, -5.1f, 5.1f),
                         m_accelPlot("Accel", accelDataBuffer_ref, accelTimeBuffer_ref, -2.1f, 1.1f),
                         m_magPlot("Mag", magDataBuffer_ref, magTimeBuffer_ref, -90.0f, 90.0f) 
{}

void ImPlotPanel::Draw() {
    ImGui::SetNextWindowPos(ImVec2(m_posX, m_posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(m_width, m_height), ImGuiCond_Always);
    ImGui::Begin("Sensor Data", nullptr, 
          ImGuiWindowFlags_NoMove | 
          ImGuiWindowFlags_NoResize | 
          ImGuiWindowFlags_NoCollapse |
          ImGuiWindowFlags_AlwaysVerticalScrollbar
    );

    // ------ New Zoom Control Section ------
    ImGui::BeginGroup();  // Group for zoom controls
    
    // Panel Zoom Slider
    ImGui::Text("Plots Height:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
    ImGui::SliderFloat("##PanelZoom", &m_vertical_zoom, min_zoom, max_zoom, "%.1fx");

    // Time Axis Zoom Slider
    ImGui::Text("Time Scale:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
    ImGui::SliderFloat("##TimeZoom", &m_horizontal_zoom, min_zoom, max_zoom, "%.1fx");
    
    // Reset buttons
    if (ImGui::Button("Reset Plot Heights")) {
        m_vertical_zoom = 1.0f;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Time Zoom")) {
        m_horizontal_zoom = 1.0f;
    }
    
    ImGui::EndGroup();
    ImGui::Separator();
    // ------ End Zoom Controls ------

    // Calculate plot heights with vertical zoom
    const float content_height = ImGui::GetContentRegionAvail().y;
    const float total_plots_height = content_height * m_vertical_zoom;
    const float plot_height = total_plots_height / 3.0f;

    // Draw plots
    m_gyroPlot.Draw(plot_height, m_horizontal_zoom);
    ImGui::Dummy(ImVec2(0, ImGui::GetStyle().ItemSpacing.y));
    m_accelPlot.Draw(plot_height, m_horizontal_zoom);
    ImGui::Dummy(ImVec2(0, ImGui::GetStyle().ItemSpacing.y));
    m_magPlot.Draw(plot_height, m_horizontal_zoom);

    ImGui::End();
}