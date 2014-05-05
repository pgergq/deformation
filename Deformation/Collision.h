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
#include "Deformable.h"
#include "Constants.h"


/// Class representing a Bounding Volume Hierarchy
class BVHierarchy final{
private:
    // variables

    //functions

public:
    //variables
    BVBoxVector bvh;                                // BVHierarchy in its final array-representation

    //functions
    BVHierarchy() = delete;                         // no default constructor
    BVHierarchy(MassVector masspoints);             // construct BVBox from given masspoints
    BVHierarchy(MassIDVector masspoints);           // construct BVBox from given masspoints+IDs
    ~BVHierarchy();                                 // destructor
    BVBoxVector sort(MassIDVector masspoints, uint mode);       // sort masspoints in order, add IDs
    BVBoxVector merge(BVBoxVector a, BVBoxVector b);            // merge two binary trees into one array
};


#endif