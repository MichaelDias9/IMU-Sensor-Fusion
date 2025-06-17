#pragma once
#include <math.h>

struct Vector3 {
    float x, y, z;
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct Quaternion {
    float w, x, y, z;
};

inline Quaternion addQuaternions(Quaternion q1, Quaternion q2){
    Quaternion result;
    result.w = q1.w + q2.w;
    result.x = q1.x + q2.x;
    result.y = q1.y + q2.y;
    result.z = q1.z + q2.z;
    return result;
}

inline Quaternion multiplyQuaternions(Quaternion q1, Quaternion q2){
    Quaternion result;
    result.w = (q1.w*q2.w) - (q1.x*q2.x) - (q1.y*q2.y) - (q1.z*q2.z);
    result.x = (q1.w*q2.x) + (q1.x*q2.w) + (q1.y*q2.z) - (q1.z*q2.y);
    result.y = (q1.w*q2.y) + (q2.w*q1.y) + (q1.z*q2.x) - (q1.x*q2.z);
    result.z = (q1.w*q2.z) + (q2.w*q1.z) + (q1.x*q2.y) - (q1.y*q2.x);
    return result;
}

inline Quaternion normalizeQuaternion(Quaternion q1){
    Quaternion result;
    float magnitude = sqrt(q1.w*q1.w + q1.x*q1.x + q1.y*q1.y + q1.z*q1.z);
    result.w = q1.w / magnitude;
    result.x = q1.x / magnitude;
    result.y = q1.y / magnitude;
    result.z = q1.z / magnitude;
    return result;
}

inline Quaternion multiplyQuaternionByScalar(Quaternion q1, float scalar){
    Quaternion result;
    result.w = q1.w * scalar;
    result.x = q1.x * scalar;
    result.y = q1.y * scalar;
    result.z = q1.z * scalar;
    return result;
}

inline Quaternion conjugateQuaternion(Quaternion q1){
    Quaternion result;
    result.w = q1.w;
    result.x = -q1.x;
    result.y = -q1.y;
    result.z = -q1.z;
    return result;
}

inline Quaternion updateQuaternionWithAngularVelocity(Quaternion q, float pitch, float yaw, float roll, float dt){
    Quaternion angularVelocity;
    angularVelocity.w = 0.0f;
    angularVelocity.x = pitch * 0.5f;
    angularVelocity.y = yaw * 0.5f;
    angularVelocity.z = roll * 0.5f;
    Quaternion quaternionDerivative = multiplyQuaternionByScalar(multiplyQuaternions(angularVelocity, q), 0.5f * dt);
    return addQuaternions(q, quaternionDerivative);
} 

inline Vector3 rotateVectorByQuaternion(Vector3 vector, Quaternion q) {
    // Convert vector to quaternion (w = 0, x,y,z = vector components)
    Quaternion vectorQuat;
    vectorQuat.w = 0.0f;
    vectorQuat.x = vector.x;
    vectorQuat.y = vector.y;
    vectorQuat.z = vector.z;
    
    // Get quaternion conjugate
    Quaternion qConj = conjugateQuaternion(q);
    
    // Perform rotation: q * vector * q_conjugate
    Quaternion temp = multiplyQuaternions(q, vectorQuat);
    Quaternion rotatedQuat = multiplyQuaternions(temp, qConj);
    
    // Extract the rotated vector from the quaternion result
    Vector3 result(0.0f, 0.0f, 0.0f);
    result.x = rotatedQuat.x;
    result.y = rotatedQuat.y;
    result.z = rotatedQuat.z;
    
    return result;
}

inline Vector3 crossProduct(Vector3 a, Vector3 b) {
    Vector3 result(0.0f, 0.0f, 0.0f);
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}