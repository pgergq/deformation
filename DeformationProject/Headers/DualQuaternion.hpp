//--------------------------------------------------------------------------------------
// File: DualQuaternion.hpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// DualQuaternion class (header+impl)
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _DUALQUATERNION_HPP_
#define _DUALQUATERNION_HPP_

#include <cmath>
#include <DirectXMath.h>
#include "../Headers/Constants.h"
#include "../Headers/Quaternion.hpp"

/// Class representing a dual quaternion
class dualquaternion
{

public:
    
    //real part
    quaternion qreal;
    // dual part
    quaternion qdual;


    /// special functions

    // default ctor
    dualquaternion() : qreal(quaternion(0, 0, 0, 1)), qdual(quaternion(0, 0, 0, 0)) {}
    // quaternion constructor
    dualquaternion(quaternion r, quaternion d) : qreal(r.normalized()), qdual(d) {}
    

    /// operators

    // operator+
    dualquaternion operator+(const dualquaternion& rhs) const
    {
        return dualquaternion(qreal + rhs.qreal, qdual + rhs.qdual);
    }
    // operator-
    dualquaternion operator-(const dualquaternion& rhs) const
    {
        return dualquaternion(qreal - rhs.qreal, qdual - rhs.qdual);
    }
    // operator* (scale)
    dualquaternion operator*(float s) const
    {
        return dualquaternion(qreal*s, qdual*s);
    }
    // operator* ('cross' product)
    dualquaternion operator*(const dualquaternion& rhs) const
    {
        return dualquaternion(qreal*rhs.qreal, qreal*rhs.qdual + qdual*rhs.qreal);
    }

    
    /// misc functions

    // conjugate
    dualquaternion conjugate() const
    {
        return dualquaternion(qreal.conjugate(), qdual.conjugate());
    }
    // normalized form (!)
    dualquaternion normalized() const
    {
        float mag = qreal.x*qreal.x + qreal.y*qreal.y + qreal.z*qreal.z + qreal.w*qreal.w;
        return (*this) * (float)(1.0f / mag);
    }
    // norm (!)
    dualquaternion norm() const
    {
        throw std::exception("not implemented");
    }
    // inverse (!)
    dualquaternion inverse() const
    {
        std::exception("not implemented");
    }
    // get rotation
    quaternion rotation() const
    {
        return qreal;
    }
    // get translation
    quaternion translation() const
    {
        return qdual * qreal.conjugate() * 2;
    }
    // create dual quaternion from rotation and translation
    // input: r = [cos(theta/2), nx * sin(theta/2), ny * sin(theta/2), nz * sin(theta/2)]
    //        t = [0, tx, ty, tz]
    static dualquaternion fromRotTrans(const quaternion& rot, const quaternion& tr)
    {
        return dualquaternion(rot, tr * rot * 0.5f);
    }

};

#endif