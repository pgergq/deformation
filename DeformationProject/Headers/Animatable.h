//--------------------------------------------------------------------------------------
// File: Animatable.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Animatable class declaration and helper typedefs
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _ANIMATABLE_H_
#define _ANIMATABLE_H_

#include <string>
#include <vector>
#include <array>
#include <DirectXMath.h>
#include "Constants.h"
#include "Collision.h"
#include "DeformableOBJ.h"

using namespace DirectX;


/// Class representing an animatable deformable object (deformable structure, animation properties)
class Animatable final
{
private:
    // inner deformable object 
    /// !!!TO FBX!!!
    DeformableOBJ dObject;


public:
    // translate inner deformable object
    void translateDeformable(int, int, int);

    // no default constructor
    Animatable() = delete;
    // default destructor
    ~Animatable();
    // constructor to use
    Animatable(std::string, int);

};

#endif