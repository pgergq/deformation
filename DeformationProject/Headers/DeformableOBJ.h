//--------------------------------------------------------------------------------------
// File: DeformableOBJ.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Deformable class declaration and helper typedefs
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _DEFORMABLEOBJ_H_
#define _DEFORMABLEOBJ_H_

#include <string>
#include <vector>
#include <array>
#include <DirectXMath.h>
#include "Constants.h"
#include "Collision.h"
#include "DeformableBase.h"

using namespace DirectX;


/// Class representing a deformable object model (vertices, masscubes, helper structures)
/// Specific, use to import .OBJ models
class DeformableOBJ : public DeformableBase
{
public:
    // no default constructor
    DeformableOBJ() = delete;
    // default destructor
    ~DeformableOBJ() {};
    // construct with file name
    DeformableOBJ(std::string s, int i) : DeformableBase(s, i) {};

protected:
    // import .OBJ file
    void importFile() override;

};

#endif