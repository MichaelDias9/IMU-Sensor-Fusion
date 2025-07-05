cmake_minimum_required(VERSION 3.20)
project(IMUTool LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Platform-specific configuration --- #
if(APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "Minimum macOS deployment version")
    # Find frameworks needed for serial communication on macOS
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    if(NOT IOKIT_FRAMEWORK OR NOT COREFOUNDATION_FRAMEWORK)
        message(FATAL_ERROR "IOKit or CoreFoundation framework not found")
    endif()
endif()

# --- Find boost --- #
if (WIN32)
    set(Boost_NO_BOOST_CMAKE ON)
    set(BOOST_ROOT "H:/Libraries/boost_1_87_0")
    set(BOOST_INCLUDEDIR "H:/Libraries/boost_1_87_0")
    set(BOOST_LIBRARYDIR "H:/Libraries/boost_1_87_0/stage/lib")
    cmake_policy(SET CMP0167 OLD)   # Use legacy FindBoost
    cmake_policy(SET CMP0144 OLD)   # Use old-style _ROOT variables
else()
    cmake_policy(SET CMP0167 NEW)   # Suppress warning, use modern behavior
    # On macOS/Linux, try to find Boost using pkg-config or system paths
    if(APPLE)
        # Common Homebrew installation paths
        set(BOOST_ROOT "/opt/homebrew" "/usr/local")
        set(Boost_NO_BOOST_CMAKE ON)
    endif()
endif()

find_package(Boost 1.70.0 REQUIRED COMPONENTS system thread)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_VERSION}")
    message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
else()
    message(FATAL_ERROR "Boost not found. Please install Boost development libraries.")
endif()

# --- Raylib --- # 
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
    ${Boost_INCLUDE_DIRS}
)

# --- Link libraries --- #
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Boost::system
    Boost::thread
    rlImGui # Links raylib transitively
    implot  # Links imgui transitively
)

# Platform-specific linking
if(APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        ${IOKIT_FRAMEWORK}
        ${COREFOUNDATION_FRAMEWORK}
    )
elseif(UNIX AND NOT APPLE)
    # Linux - might need additional libraries for serial communication
    target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
endif()

# --- Compiler-specific options --- #
if(APPLE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE __APPLE__)
elseif(UNIX)
    target_compile_definitions(${PROJECT_NAME} PRIVATE __linux__)
endif()

# --- Resources --- #
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/resources
        $<TARGET_FILE_DIR:IMUTool>/resources
)