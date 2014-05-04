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

#include <vector>
#include <cfloat>
#include "Deformable.h"


typedef unsigned int uint;

/// Structure representing BVBox data
/// isLeaf -> the box contains 2 masspoints with given bounding box (<-- valid ID(s))
struct BVBOX {

    int leftID;            // ID of left child masspoint
    int rightID;           // ID of right child masspoint

    float minX;             // coordinates
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;

    BVBOX() : leftID(-1), rightID(-1), minX(FLT_MAX), maxX(FLT_MIN), minY(FLT_MAX), maxY(FLT_MIN), minZ(FLT_MAX), maxZ(FLT_MIN) {}
};

typedef std::vector<MASSPOINT> MassVector;
typedef std::vector<BVBOX> BVBoxVector;
typedef std::tuple<int, MASSPOINT> MassID;
typedef std::vector<MassID> MassIDVector;

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
    ~BVHierarchy();                                 // destructor
    BVBoxVector sort(MassIDVector masspoints, uint mode);       // sort masspoints in order, add IDs
    BVBoxVector merge(BVBoxVector a, BVBoxVector b);            // merge two binary trees into one array
};

















#endif