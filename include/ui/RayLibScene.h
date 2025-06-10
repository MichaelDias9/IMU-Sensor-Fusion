#pragma once
#include "raylib.h"

#include "Attitude.h"

class RaylibScene {
private:
    const int m_sceneOriginX;
    const int m_sceneOriginY;
    const int m_sceneWidth;
    const int m_sceneHeight;
    const Attitude& attitude_;
    Camera m_camera;
    RenderTexture2D m_renderTarget;

public:
    RaylibScene(int originX, int originY, int width, int height, const Attitude& attitude);
    ~RaylibScene();
    void Init();
    void Draw();
};