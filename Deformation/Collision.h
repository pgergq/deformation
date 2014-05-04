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
#include "Deformable.h"


typedef unsigned int uint;

/// Structure representing BVBox data
/// if(isLeaf) -> the box contains 2 masspoints with given bounding box
/// if(!isLeaf) -> the box contains only min and max coords for children objects
struct BVBOX {

    uint leftID;            // ID of left child masspoint
    uint rightID;           // ID of right child masspoint

    float minX;             // coordinates
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;

    bool isLeaf;            // isLeaf==true -> valid left and right ID with min and max coords
                            // isLeaf==false -> valid min and max coords
};

typedef std::vector<MASSPOINT> MassVector;
typedef std::vector<BVBOX> BVBoxVector;
typedef std::tuple<uint, MASSPOINT> MassID;
typedef std::vector<MassID> MassIDVector;

/// Class representing a Bounding Volume Hierarchy
class BVHierarchy final{
private:
    // variables

    //functions

public:
    //variables
    BVBOX* bvh;                                     // BVHierarchy in its final array-representation
    uint bvhsize;                                   // size(bvh)

    //functions
    BVHierarchy() = delete;                         // no default constructor
    BVHierarchy(MassVector masspoints);             // construct BVBox from given masspoints
    ~BVHierarchy();                                 // destructor
    BVBoxVector sort(MassIDVector masspoints, uint mode);       // sort masspoints in order, add IDs
    //BVBoxVector hierarchize(MassIDVector masspoints, uint mode);  // build hierarchy from vector<MASSPOINT>

};

















#endif