#include "ui/ImGuiPanel.h"

ImGuiPanel::ImGuiPanel(int posX, int posY, int width, int height, Attitude& attitude, float& KpRollPitch, float& KpYaw, float& KiRollPitch, float& KiYaw)
    : m_posX(posX), m_posY(posY), m_width(width), m_height(height), attitude_(attitude), KpRollPitch_(KpRollPitch), KpYaw_(KpYaw), KiRollPitch_(KiRollPitch), KiYaw_(KiYaw) {}

void ImGuiPanel::Draw() {
    ImGui::SetNextWindowPos(ImVec2(m_posX, m_posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(m_width, m_height), ImGuiCond_Always);
    ImGui::Begin("INFO:");
    ImGui::Text("Roll:  %.2f", attitude_.roll);
    ImGui::Text("Pitch: %.2f", attitude_.pitch);
    ImGui::Text("Yaw:   %.2f", attitude_.yaw);
    ImGui::Text("Quaternion W: %.2f", attitude_.w);
    ImGui::Text("Quaternion X: %.2f", attitude_.x);
    ImGui::Text("Quaternion Y: %.2f", attitude_.y);
    ImGui::Text("Quaternion Z: %.2f", attitude_.z);
    ImGui::SliderFloat("Kp Roll/Pitch", &KpRollPitch_, 0.0f, 100.0f);
    ImGui::SliderFloat("Kp Yaw", &KpYaw_, 0.0f, 10.0f);
    ImGui::SliderFloat("Ki Roll/Pitch", &KiRollPitch_, 0.0f, 100.0f);
    ImGui::SliderFloat("Ki Yaw", &KiYaw_, 0.0f, 10.0f);
    ImGui::End();
}