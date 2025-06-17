#include "ui/RunApp.h"

#include "Config.h"
#include "util/Attitude.h"

#include "ui/RayLibScene.h"
#include "ui/ImPlotPanel.h"
#include "ui/ImGuiPanel.h"
#include "util/ThreadSafeRingBuffer3D.h"

#include "rlImGui.h"
#include "imgui.h"
#include "implot.h"

void runApp(const GyroBuffer &gyroDataBuffer, 
            const AccelBuffer& accelDataBuffer, 
            const MagBuffer& magDataBuffer, 
            const GyroTimesBuffer &gyroTimeBuffer,
            const AccelTimesBuffer &accelTimeBuffer,
            const MagTimesBuffer &magTimeBuffer,
            Attitude &estimatedAttitude,
            float& KpRollPitch, float& KpYaw, float& KiRollPitch, float& KiYaw) 
{ 
  // Initialize window with config values
  InitWindow(screenWidth, screenHeight, "IMU + Attitude Estimation");
  SetTargetFPS(targetFPS);

  // Initialize Raylib Scene
  RaylibScene raylibScene(screenWidth/2, screenHeight/2, screenWidth/2, screenHeight/2, estimatedAttitude);
  raylibScene.Init();
  
  // Initialize Plots Context
  rlImGuiSetup(true);
  ImPlot::CreateContext();
  // Enable 32 bit vertex indices for more vertices
  ImGuiIO& io = ImGui::GetIO();
  io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
  // Initialize Plots
  ImPlotPanel plotPanel(0, 0, screenWidth/2, screenHeight, 
                        gyroDataBuffer, accelDataBuffer, magDataBuffer, gyroTimeBuffer, accelTimeBuffer, magTimeBuffer);

  // Initialize GUI
  ImGuiPanel guiPanel(screenWidth/2, 0, screenWidth/2, screenHeight/2, estimatedAttitude, KpRollPitch, KpYaw, KiRollPitch, KiYaw);
  
  // Run Main Loop
  while (!WindowShouldClose()) {
    // Draw frame
    BeginDrawing();
    ClearBackground(RAYWHITE);
    
    raylibScene.Draw();
      
    rlImGuiBegin();
    plotPanel.Draw();
    guiPanel.Draw();
    rlImGuiEnd();
    
    EndDrawing();
  }
  // Exit Gracefully
  ImPlot::DestroyContext();
  rlImGuiShutdown();
  CloseWindow();
}