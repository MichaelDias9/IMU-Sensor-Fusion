#pragma once

namespace Structs3D {
    
    struct Vector3F {
        float x, y, z;
        Vector3F(float x, float y, float z) : x(x), y(y), z(z) {}
    };

    struct QuaternionF {
        float w, x, y, z;
    };
}