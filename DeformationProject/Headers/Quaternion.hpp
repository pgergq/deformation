//--------------------------------------------------------------------------------------
// File: Quaternion.hpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Quaternion class (header+impl)
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_

#include <cmath>
#include <DirectXMath.h>
#include "../Headers/Constants.h"
#include "../../Assimp/include/assimp/quaternion.h"


/// Class representing a quaternion
class quaternion
{

public:

    // vector part
    float x;
    // vector part
    float y;
    // vector part
    float z;
    // real part
    float w;


    /// special functions

    // default ctor
    quaternion(float _x = 0, float _y = 0, float _z = 0, float _w = 1) : x(_x), y(_y), z(_z), w(_w) {}
    // vector ctor
    quaternion(const XMFLOAT3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
    // vector ctor 2
    quaternion(const XMFLOAT4& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}


    /// conversion operators
    explicit operator aiQuaternion() const
    {
        return aiQuaternion(w, x, y, z);
    }


    /// operators

    // operator+
    quaternion operator+(const quaternion& rhs) const
    {
        return quaternion(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
    }
    // operator-
    quaternion operator-(const quaternion& rhs) const
    {
        return quaternion(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
    }
    // operator-
    quaternion operator-() const
    {
        return quaternion(-x, -y, -z, -w);
    }
    // operator* (scale)
    quaternion operator*(float s) const
    {
        return quaternion(x*s, y*s, z*s, w*s);
    }
    // operator/ (scale)
    quaternion operator/(float s) const
    {
        if (abs(s) < 1e-10)
        {
            return quaternion();
        }
        else
        {
            return quaternion(x / s, y / s, z / s, w / s);
        }
    }
    // operator* (cross product)
    quaternion operator*(const quaternion& rhs) const
    {
        XMVECTOR ls = XMVectorSet(this->x, this->y, this->z, 0);
        XMVECTOR rs = XMVectorSet(rhs.x, rhs.y, rhs.z, 0);
        XMFLOAT3 res;
        XMStoreFloat3(&res, ls * rhs.w + rs * w + XMVector3Cross(ls, rs));
        return quaternion(res, (float)(w * rhs.w - XMVectorGetX(XMVector3Dot(ls, rs))));
    }
    // conjugate
    quaternion conjugate() const
    {
        return quaternion(-x, -y, -z, w);
    }
    // norm (length)
    float norm() const
    {
        return sqrt(x*x + y*y + z*z + w*w);
    }
    // inverse
    quaternion inverse() const
    {
        float l = sqrt(x*x + y*y + z*z + w*w);
        return this->conjugate() / (l*l);
    }
    // normalized
    quaternion normalized() const
    {
        float len = sqrt(x*x + y*y + z*z + w*w);
        return (*this) / len;
    }


    /// misc functions

    // rotation angle
    quaternion angle() const
    {
        float cosa2 = w;
        float sina2 = sqrt(x*x + y*y + z*z);
        return (atan2(sina2, cosa2) * 2) * 180 / XM_PI;
    }
    // create quaternion from rotation angle and axis
    static quaternion fromAngleRotation(const XMFLOAT3& axis, float angle)
    {
        XMFLOAT3 ret;
        auto v = XMVector3Normalize(XMLoadFloat3(&axis)) * sin(0.5f * angle);
        XMStoreFloat3(&ret, v);
        return quaternion(ret, cos(0.5f * angle));
    }
    // convert to angle and rotation
    static void toAngleRotation(const quaternion& q, XMFLOAT3& axis, float& angle)
    {
        XMVECTOR tmp = XMVectorSet(q.x, q.y, q.z, q.w);
        XMVECTOR tmpax;
        XMQuaternionToAxisAngle(&tmpax, &angle, tmp);
        XMStoreFloat3(&axis, tmpax);
    }
    // rotate vector with this quaternion
    XMFLOAT3 rotateVector(const XMFLOAT3& v) const
    {
        auto res = (*this) * quaternion(v, 0) * (*this).inverse();
        return XMFLOAT3(res.x, res.y, res.z);
    }
    // convert to rotation matrix
    static XMFLOAT3X3 toRotationMatrix(const quaternion& q)
    {
        return XMFLOAT3X3(
            1 - 2 * q.y*q.y - 2 * q.z*q.z, 2 * q.x*q.y - 2 * q.z*q.w, 2 * q.x*q.z + 2 * q.y*q.w,
            2 * q.x*q.y + 2 * q.z*q.w, 1 - 2 * q.x*q.x - 2 * q.z*q.z, 2 * q.y*q.z - 2 * q.x*q.w,
            2 * q.x*q.z - 2 * q.y*q.w, 2 * q.y*q.z + 2 * q.x*q.w, 1 - 2 * q.x*q.x - 2 * q.y*q.y
            );
    }
    // convert to rotation matrix
    XMFLOAT3X3 toRotationMatrix() const
    {
        return toRotationMatrix(*this);
    }
    // SLERP
    static quaternion slerp(const quaternion& q0, const quaternion& q1, float t)
    {
        quaternion qstart = q0.normalized(), qend = q1.normalized();
        float dot = qstart.x*qend.x + qstart.y*qend.y + qstart.z*qend.z + qstart.w*qend.w;
        if (dot < 0)
        {
            dot = -dot;
            qend = qend * -1;
        }
        if (dot < 0.95f)
        {
            float omega = acos(dot);
            float somega = sin(omega);
            float w1 = sin((1.0f - t) * omega) / somega;
            float w2 = sin(t * omega) / somega;
            return qstart*w1 + qend*w2;
        }
        else
        {
            return (qstart * (1 - t) + qend * t).normalized();
        }
    }
    // SLERP
    quaternion slerp(const quaternion& q1, float t)
    {
        return slerp(*this, q1, t);
    }


    // dtor
    ~quaternion() {}
};


#endif