//--------------------------------------------------------------------------------------
// File: Collision.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Collision detection functions (impl)
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <tuple>
#include "Collision.h"
#include "Constants.h"



//--------------------------------------------------------------------------------------
// Constructor: malloc tree memory, build tree, sort tree (<- USE THIS)
//--------------------------------------------------------------------------------------
BVHierarchy::BVHierarchy(MassIDTypeVector masspoints){

    // Mass[] -> Mass+ID[]
    MassIDTypeVector x;
    uint size = masspoints.size();
    // extend array to contain power-of-2 number elements
    uint ext = std::exp2(std::ceil(std::log2(size))) - size;
    x.reserve(size + ext);
    for (uint i = 0; i < size; i++){
        x.push_back(masspoints[i]);
    }
    for (uint i = 0; i < ext; i++){
        x.push_back(MassIDType(-1, -1, MASSPOINT{}));
    }

    // sort and structurise input
    bvh = sort(x, 0);

}


//--------------------------------------------------------------------------------------
// Destructor: delete hierarchy tree data
//--------------------------------------------------------------------------------------
BVHierarchy::~BVHierarchy(){}


//--------------------------------------------------------------------------------------
// Sort helpers
//--------------------------------------------------------------------------------------
bool sortX(MassIDType a, MassIDType b){
    return (std::get<2>(a)).newpos.x < (std::get<2>(b)).newpos.x;
}

bool sortY(MassIDType a, MassIDType b){
    return (std::get<2>(a)).newpos.y < (std::get<2>(b)).newpos.y;
}

bool sortZ(MassIDType a, MassIDType b){
    return (std::get<2>(a)).newpos.z < (std::get<2>(b)).newpos.z;
}


//--------------------------------------------------------------------------------------
// Sort: sort masspoint+ID pairs: repeatedly by X-Y-Z coordinates
//       mode: 0=X, 1=Y, 2=Z (sort)
//--------------------------------------------------------------------------------------
BVBoxVector BVHierarchy::sort(MassIDTypeVector masspoints, uint mode){

    // sort 'em!

    // two masspoints -> leaf
    if (masspoints.size() == 2){

        BVBOX tmp;
        BVBoxVector ret;

        // two invalid leaves, signal with two '-1's
        if (std::get<0>(masspoints[0]) == -1 && std::get<0>(masspoints[1]) == -1){
            tmp.leftType = -1;
            tmp.rightType = -1;
        }

        // one valid and one invalid masspoint -> yippie, edge of the 'valid tree'
        else if (std::get<0>(masspoints[0]) != -1 && std::get<0>(masspoints[1]) == -1){
            tmp.leftID = std::get<0>(masspoints[0]);
            tmp.leftType = std::get<1>(masspoints[0]);
            tmp.rightID = -1;
            tmp.rightType = -1;         // -1 is for empty leaves
            tmp.minX = (std::get<2>(masspoints[0])).newpos.x - collisionRangeConstant;
            tmp.maxX = (std::get<2>(masspoints[0])).newpos.x + collisionRangeConstant;
            tmp.minY = (std::get<2>(masspoints[0])).newpos.y - collisionRangeConstant;
            tmp.maxY = (std::get<2>(masspoints[0])).newpos.y + collisionRangeConstant;
            tmp.minZ = (std::get<2>(masspoints[0])).newpos.z - collisionRangeConstant;
            tmp.maxZ = (std::get<2>(masspoints[0])).newpos.z + collisionRangeConstant;
        }

        // two valid masspoints
        else if (std::get<0>(masspoints[0]) != -1 && std::get<0>(masspoints[1]) != -1){
            tmp.leftID = std::get<0>(masspoints[0]);
            tmp.leftType = std::get<1>(masspoints[0]);
            tmp.rightID = std::get<0>(masspoints[1]);
            tmp.rightType = std::get<1>(masspoints[1]);
            tmp.minX = std::min((std::get<2>(masspoints[0])).newpos.x, (std::get<2>(masspoints[1])).newpos.x) - collisionRangeConstant;
            tmp.maxX = std::max((std::get<2>(masspoints[0])).newpos.x, (std::get<2>(masspoints[1])).newpos.x) + collisionRangeConstant;
            tmp.minY = std::min((std::get<2>(masspoints[0])).newpos.y, (std::get<2>(masspoints[1])).newpos.y) - collisionRangeConstant;
            tmp.maxY = std::max((std::get<2>(masspoints[0])).newpos.y, (std::get<2>(masspoints[1])).newpos.y) + collisionRangeConstant;
            tmp.minZ = std::min((std::get<2>(masspoints[0])).newpos.z, (std::get<2>(masspoints[1])).newpos.z) - collisionRangeConstant;
            tmp.maxZ = std::max((std::get<2>(masspoints[0])).newpos.z, (std::get<2>(masspoints[1])).newpos.z) + collisionRangeConstant;
            
        }

        ret.push_back(tmp);
        return ret;
    }

    // three or more masspoints -> sort, divide, recursion
    else {

        // only sort valid masspoints (no -1 ID)
        MassIDTypeVector tmp;
        uint valid = 0;
        while (valid < masspoints.size() && std::get<0>(masspoints[valid]) != (-1)){
            valid++;
        }

        // sort vector elements by the appropriate coordinate
        switch (mode)
        {
            case 0: std::sort(masspoints.begin(), masspoints.begin() + valid, sortX); break;
            case 1: std::sort(masspoints.begin(), masspoints.begin() + valid, sortY); break;
            case 2: std::sort(masspoints.begin(), masspoints.begin() + valid, sortZ); break;
            default: break;
        }

        // valid masspoints must stay at the beginning
        // divide vector and recurse
        BVBoxVector x, y, xy;
        int div = std::ceil((float)masspoints.size() / 2);
        x = sort(MassIDTypeVector(masspoints.begin(), masspoints.begin() + div), (mode + 1) % 3);
        y = sort(MassIDTypeVector(masspoints.begin() + div, masspoints.end()), (mode + 1) % 3);

        BVBOX tmp2;                                     // tmp <- bounding box for each masspoint in masspoints

        if (x.size() != 0){                             // (check for empty subtrees)
            tmp2.leftType = (x[0].leftType == -1 && x[0].rightType == -1)? -1 : 0;   // 0: valid tree node, -1: invalid (dead) tree node
            tmp2.rightType = (y.size() != 0 && y[0].leftType == -1 && y[0].rightType == -1) ? -1 : 0;
            tmp2.minX = (y.size() != 0 ? std::min(x[0].minX, y[0].minX) : x[0].minX);
            tmp2.maxX = (y.size() != 0 ? std::max(x[0].maxX, y[0].maxX) : x[0].maxX);
            tmp2.minY = (y.size() != 0 ? std::min(x[0].minY, y[0].minY) : x[0].minY);
            tmp2.maxY = (y.size() != 0 ? std::max(x[0].maxY, y[0].maxY) : x[0].maxY);
            tmp2.minZ = (y.size() != 0 ? std::min(x[0].minZ, y[0].minZ) : x[0].minZ);
            tmp2.maxZ = (y.size() != 0 ? std::max(x[0].maxZ, y[0].maxZ) : x[0].maxZ);
        }


        //if (!(x.size() == 0 && y.size() == 0)){           // only push if node has any children
            xy.reserve(x.size() + y.size() + 1);
            xy = merge(x, y);                               // merge two children
            xy.insert(xy.begin(), tmp2);                    // vector[] <- {container, leftchildren, rightchildren}
        //}

        return xy;

    }
}


//--------------------------------------------------------------------------------------
// Merge: get two binary trees in an array -> merge them -> convert back to array
//--------------------------------------------------------------------------------------
BVBoxVector BVHierarchy::merge(BVBoxVector a, BVBoxVector b){

    // two nonempty tree
    if (a.size() != 0 && b.size() != 0){

        uint level = std::log2(a.size() + 1);       // num of levels in each tree

        BVBoxVector ret;

        BVBoxVector::iterator ai = a.begin();
        BVBoxVector::iterator bi = b.begin();

        for (uint i = 1; i <= level; i++){          // get 2^level elements from the array ('treeize')
            int inc = std::pow(2, i - 1);
            ret.insert(ret.end(), ai, ai + inc);
            ai = ai + inc;
            ret.insert(ret.end(), bi, bi + inc);
            bi = bi + inc;
        }
        return ret;
    }

    // one empty tree, must be the right side
    else if (a.size() != 0 && b.size() == 0){
        return a;
    }

    // both trees are empty, return empty array
    else{
        return BVBoxVector();
    }
}