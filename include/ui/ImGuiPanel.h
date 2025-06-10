#pragma once
#include "imgui.h"
#include "Attitude.h"

class ImGuiPanel {
private:
    int m_posX;
    int m_posY;
    int m_width;
    int m_height;
    Attitude& attitude_;
    float& alpha_;

public:
    ImGuiPanel(int posX, int posY, int width, int height, Attitude& attitude, float& alpha);
    void Draw();
};