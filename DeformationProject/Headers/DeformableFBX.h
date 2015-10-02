//--------------------------------------------------------------------------------------
// File: DeformableFBX.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Deformable class declaration and helper typedefs
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _DEFORMABLEFBX_H_
#define _DEFORMABLEFBX_H_

#include <string>
#include <vector>
#include <array>
#include <DirectXMath.h>
#include <assimp\Importer.hpp>
#include <assimp\scene.h>
#include <assimp\postprocess.h>
#include "Constants.h"
#include "Collision.h"
#include "DeformableBase.h"

using namespace DirectX;


/// Class representing a deformable object model (vertices, masscubes, helper structures)
/// Specific, use to import .FBX models with Assimp
class DeformableFBX : public DeformableBase
{
public:
    // no default constructor
    DeformableFBX() = delete;
    // default destructor
    ~DeformableFBX() {};
    // construct with file name
    DeformableFBX(std::string s, int i) : DeformableBase(s, i) {};

protected:
    // import .OBJ file
    void importFile() override;

};

#endif