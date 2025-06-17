#pragma once
#include "imgui.h"
#include "util/Attitude.h"

class ImGuiPanel {
private:
    int m_posX;
    int m_posY;
    int m_width;
    int m_height;
    Attitude& attitude_;
    float& KpRollPitch_;
    float& KpYaw_;
    float& KiRollPitch_;
    float& KiYaw_;

public:
    ImGuiPanel(int posX, int posY, int width, int height, Attitude& attitude, float& KpRollPitch, float& KpYaw, float& KiRollPitch, float& KiYaw);
    void Draw();
};