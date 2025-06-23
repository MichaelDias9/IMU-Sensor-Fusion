#pragma once
#include "raylib.h"
#include "util/Structs3D.h"

class RaylibScene {
private:
    const int m_sceneOriginX;
    const int m_sceneOriginY;
    const int m_sceneWidth;
    const int m_sceneHeight;
    Camera m_camera;
    RenderTexture2D m_renderTarget;
    const Structs3D::QuaternionF& attitude_;
    const Structs3D::Vector3F& accelVector_;

public:
    RaylibScene(int originX, int originY, int width, int height, const Structs3D::QuaternionF& attitude, const Structs3D::Vector3F& accelVector);
    ~RaylibScene();
    void Init();
    void Draw();
};