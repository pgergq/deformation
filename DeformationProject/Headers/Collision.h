//--------------------------------------------------------------------------------------
// File: Collision.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Collision detection functions (header)
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _COLLISION_H_
#define _COLLISION_H_

#include <cfloat>
#include <vector>
#include "DeformableBase.h"
#include "Constants.h"


/// Class representing a Bounding Volume Hierarchy
class BVHierarchy final{

public:
    // BVHierarchy in its final array-representation
    BVBoxVector bvh;

    // no default constructor
    BVHierarchy() = delete;
    // construct BVBox from given masspoints+IDs
    BVHierarchy(MassIDTypeVector masspoints);
    // destructor
    ~BVHierarchy();
    // sort masspoints in order, add IDs
    BVBoxVector sort(MassIDTypeVector masspoints, uint mode);
    // merge two binary trees into one array
    BVBoxVector merge(BVBoxVector a, BVBoxVector b);
};


#endif