#pragma once
#include "imgui.h"
#include "util/Structs3D.h"
#include "ComplementaryFilter.h"

class ImGuiPanel {
private:
    int m_posX;
    int m_posY;
    int m_width;
    int m_height;
    Structs3D::QuaternionF& attitude_;
    ComplementaryFilter& filter_;
    

public:
    ImGuiPanel(int posX, int posY, int width, int height, Structs3D::QuaternionF& attitude, ComplementaryFilter& complementaryFilter);
    void Draw();
};