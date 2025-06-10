cmake_minimum_required(VERSION 3.20)
project(IMUTool LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_policy(SET CMP0167 NEW)   # Supress warning
find_package(Boost 1.87.0 REQUIRED COMPONENTS system thread)

# Link GoogleTest
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# --- Fetch raylib --- # 
include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG master
    CMAKE_ARGS 
        -DBUILD_SHARED_LIBS=OFF
        -DUSE_EXTERNAL_GLFW=OFF
        -DSUPPORT_WINMM_HIGH_RES_TIMER=OFF
)
FetchContent_MakeAvailable(raylib)

# --- ImGui --- #
file(GLOB IMGUI_SOURCES third_party/imgui/*.cpp)
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC third_party/imgui)

# --- ImPlot --- #
file(GLOB IMPLOT_SOURCES third_party/implot/*.cpp)
add_library(implot STATIC ${IMPLOT_SOURCES})
target_include_directories(implot PUBLIC third_party/implot)
target_link_libraries(implot PRIVATE imgui)

# --- rlImGui --- #
add_library(rlImGui STATIC third_party/rlImGui/rlImGui.cpp)
target_include_directories(rlImGui PUBLIC
    third_party/rlImGui
    ${raylib_SOURCE_DIR}/src  # Needed for raylib internal headers
)
target_link_libraries(rlImGui PUBLIC raylib imgui)

# --- Main executable --- #
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    include
    include/util
    include/ui
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Boost::system
    Boost::thread
    rlImGui # Links raylib transitively
    implot  # Links imgui transitively
    # GTest::gtest GTest::gtest_main 
)

# --- Resources --- #
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/resources
        $<TARGET_FILE_DIR:IMUTool>/resources
)