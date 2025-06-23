#include "ui/ImGuiPanel.h"
#include "ComplementaryFilter.h"

ImGuiPanel::ImGuiPanel(int posX, int posY, int width, int height, 
                       Structs3D::QuaternionF& attitude, 
                       ComplementaryFilter& complementaryFilter)
    : m_posX(posX), m_posY(posY), m_width(width), m_height(height), 
      attitude_(attitude), filter_(complementaryFilter) {}

void ImGuiPanel::Draw() {
    ImGui::SetNextWindowPos(ImVec2(m_posX, m_posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(m_width, m_height), ImGuiCond_Always);
    
    if (ImGui::Begin("Attitude Control Panel", nullptr, 
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        
    // Quaternion Section
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(12, 18));
    
    if (ImGui::CollapsingHeader("Quaternion Values", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent();
        ImGui::Text("W: %.3f X: %.3f Y: %.3f Z: %.3f", attitude_.w, attitude_.x, attitude_.y, attitude_.z);
        ImGui::Unindent();
    }
            
    // Complementary Filter Section
    if (ImGui::CollapsingHeader("Complementary Filter", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent();
        
        // P and I Terms in columns
        if (ImGui::BeginTable("FilterTerms", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("Proportional Terms", ImGuiTableColumnFlags_WidthFixed, 200.0f);
            ImGui::TableSetupColumn("Integral Terms", ImGuiTableColumnFlags_WidthFixed, 200.0f);
            ImGui::TableHeadersRow();
            
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Pitch: %.4f", filter_.PTermPitch_);
            ImGui::Text("Roll:  %.4f", filter_.PTermRoll_);
            ImGui::Text("Yaw:   %.4f", filter_.PTermYaw_);
            
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("Pitch: %.4f", filter_.ITermPitch_);
            ImGui::Text("Roll:  %.4f", filter_.ITermRoll_);
            ImGui::Text("Yaw:   %.4f", filter_.ITermYaw_);
            
            ImGui::EndTable();
        }
        
        
        // Control Sliders
        ImGui::Text("Control Parameters:");
        
        ImGui::SliderFloat("Kp Roll/Pitch", &filter_.KpRollPitch_, 0.0f, 10.0f, "%.2f");
        ImGui::SliderFloat("Kp Yaw", &filter_.KpYaw_, 0.0f, 10.0f, "%.2f");
        
        ImGui::Spacing();
        
        ImGui::SliderFloat("Ki Roll/Pitch", &filter_.KiRollPitch_, 0.0f, 2.0f, "%.3f");
        ImGui::SliderFloat("Ki Yaw", &filter_.KiYaw_, 0.0f, 2.0f, "%.3f");
        
        ImGui::Unindent();
    }
        
    ImGui::PopStyleVar();
    }
    ImGui::End();
}