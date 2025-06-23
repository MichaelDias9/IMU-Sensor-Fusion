#pragma once
#include <math.h>
#include "Structs3D.h"

using namespace Structs3D;

namespace Math3D {

    inline QuaternionF addQuaternions(QuaternionF q1, QuaternionF q2){
        QuaternionF result;
        result.w = q1.w + q2.w;
        result.x = q1.x + q2.x;
        result.y = q1.y + q2.y;
        result.z = q1.z + q2.z;
        return result;
    }

    inline QuaternionF multiplyQuaternions(QuaternionF q1, QuaternionF q2){
        QuaternionF result;
        result.w = (q1.w*q2.w) - (q1.x*q2.x) - (q1.y*q2.y) - (q1.z*q2.z);
        result.x = (q1.w*q2.x) + (q1.x*q2.w) + (q1.y*q2.z) - (q1.z*q2.y);
        result.y = (q1.w*q2.y) + (q2.w*q1.y) + (q1.z*q2.x) - (q1.x*q2.z);
        result.z = (q1.w*q2.z) + (q2.w*q1.z) + (q1.x*q2.y) - (q1.y*q2.x);
        return result;
    }

    inline QuaternionF normalizeQuaternion(QuaternionF q1){
        QuaternionF result;
        float magnitude = sqrt(q1.w*q1.w + q1.x*q1.x + q1.y*q1.y + q1.z*q1.z);
        result.w = q1.w / magnitude;
        result.x = q1.x / magnitude;
        result.y = q1.y / magnitude;
        result.z = q1.z / magnitude;
        return result;
    }

    inline QuaternionF multiplyQuaternionByScalar(QuaternionF q1, float scalar){
        QuaternionF result;
        result.w = q1.w * scalar;
        result.x = q1.x * scalar;
        result.y = q1.y * scalar;
        result.z = q1.z * scalar;
        return result;
    }

    inline QuaternionF conjugateQuaternion(QuaternionF q1){
        QuaternionF result;
        result.w = q1.w;
        result.x = -q1.x;
        result.y = -q1.y;
        result.z = -q1.z;
        return result;
    }

    inline QuaternionF updateQuaternionWithAngularVelocity(QuaternionF q, float wx, float wy, float wz, float dt){
        QuaternionF angularVelocity;
        angularVelocity.w = 0.0f;
        angularVelocity.x = wx;
        angularVelocity.y = wy;
        angularVelocity.z = wz;
        QuaternionF quaternionDerivative = multiplyQuaternionByScalar(multiplyQuaternions(q, angularVelocity), 0.5f * dt);
        return addQuaternions(q, quaternionDerivative);
    } 

    inline Vector3F rotateVectorByQuaternion(Vector3F vector, QuaternionF q) {
        // Convert vector to QuaternionF (w = 0, x,y,z = vector components)
        QuaternionF vectorQuat;
        vectorQuat.w = 0.0f;
        vectorQuat.x = vector.x;
        vectorQuat.y = vector.y;
        vectorQuat.z = vector.z;
        
        // Get QuaternionF conjugate
        QuaternionF qConj = conjugateQuaternion(q);
        
        // Perform rotation: q * vector * q_conjugate
        QuaternionF temp = multiplyQuaternions(q, vectorQuat);
        QuaternionF rotatedQuat = multiplyQuaternions(temp, qConj);
        
        // Extract the rotated vector from the QuaternionF result
        Vector3F result(0.0f, 0.0f, 0.0f);
        result.x = rotatedQuat.x;
        result.y = rotatedQuat.y;
        result.z = rotatedQuat.z;
        
        return result;
    }

    inline Vector3F crossProduct(Vector3F a, Vector3F b) {
        Vector3F result(0.0f, 0.0f, 0.0f);
        result.x = a.y * b.z - a.z * b.y;
        result.y = a.z * b.x - a.x * b.z;
        result.z = a.x * b.y - a.y * b.x;
        return result;
    }
    
    inline Vector3F normalizeVector(Vector3F v) {
        float magnitude = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        Vector3F result(0.0f, 0.0f, 0.0f);
        result.x = v.x / magnitude;
        result.y = v.y / magnitude;
        result.z = v.z / magnitude;
        return result;
    }
}
