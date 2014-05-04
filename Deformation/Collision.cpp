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
// Constructor: malloc tree memory, build tree, sort tree
//--------------------------------------------------------------------------------------
BVHierarchy::BVHierarchy(MassVector masspoints){
    
    // Mass[] -> Mass+ID[]
    MassIDVector x;
    uint size = masspoints.size();
    // extend array to contain power-of-2 number elements
    uint ext = std::exp2(std::ceil(std::log2(size))) - size;
    x.reserve(size+ext);
    for (uint i = 0; i < size; i++){
        x.push_back(std::tuple<uint, MASSPOINT>(i, masspoints[i]));
    }
    for (uint i = 0; i < ext; i++){
        x.push_back(std::tuple<uint, MASSPOINT>(-1, MASSPOINT{}));
    }

    // sort and structurise input
    BVBoxVector v = sort(x, 0);

    // vector(BVBOX) -> BVBOX[]
    bvh = new BVBOX[v.size()];
    for (uint i = 0; i < v.size(); i++)
        bvh[i] = v[i];

}


//--------------------------------------------------------------------------------------
// Destructor: delete hierarchy tree data
//--------------------------------------------------------------------------------------
BVHierarchy::~BVHierarchy(){
    delete[] bvh;
    bvh = nullptr;
    bvhsize = 0;
}


//--------------------------------------------------------------------------------------
// Sort helpers
//--------------------------------------------------------------------------------------
bool sortX(MassID a, MassID b){
    return (std::get<1>(a)).newpos.x < (std::get<1>(b)).newpos.x;
}

bool sortY(MassID a, MassID b){
    return (std::get<1>(a)).newpos.y < (std::get<1>(b)).newpos.y;
}

bool sortZ(MassID a, MassID b){
    return (std::get<1>(a)).newpos.z < (std::get<1>(b)).newpos.z;
}

//--------------------------------------------------------------------------------------
// Sort: sort masspoint+ID pairs: repeatedly by X-Y-Z coordinates
//       mode: 0=X, 1=Y, 2=Z (sort
//--------------------------------------------------------------------------------------
BVBoxVector BVHierarchy::sort(MassIDVector masspoints, uint mode){

    // sort 'em!

    // two masspoints -> leaf
    if (masspoints.size() == 2){

        BVBOX tmp;
        BVBoxVector ret;

        // two invalid leaves, return empty vector

        // one valid and one invalid masspoint -> yippie, edge of the 'valid tree'
        if (std::get<0>(masspoints[0]) != -1 && std::get<0>(masspoints[1]) == -1){
            tmp.isLeaf = true;
            tmp.leftID = std::get<0>(masspoints[0]);
            tmp.minX = (std::get<1>(masspoints[0])).newpos.x - g_fCollisionRange;
            tmp.maxX = (std::get<1>(masspoints[0])).newpos.x + g_fCollisionRange;
            tmp.minY = (std::get<1>(masspoints[0])).newpos.y - g_fCollisionRange;
            tmp.maxY = (std::get<1>(masspoints[0])).newpos.y + g_fCollisionRange;
            tmp.minZ = (std::get<1>(masspoints[0])).newpos.z - g_fCollisionRange;
            tmp.maxZ = (std::get<1>(masspoints[0])).newpos.z + g_fCollisionRange;
            ret.push_back(tmp);
        }

        // two valid masspoints
        else if (std::get<0>(masspoints[0]) != -1 && std::get<0>(masspoints[1]) != -1){
            tmp.isLeaf = true;
            tmp.leftID = std::get<0>(masspoints[0]);
            tmp.rightID = std::get<0>(masspoints[1]);
            tmp.minX = std::min((std::get<1>(masspoints[0])).newpos.x, (std::get<1>(masspoints[1])).newpos.x) - g_fCollisionRange;
            tmp.maxX = std::max((std::get<1>(masspoints[0])).newpos.x, (std::get<1>(masspoints[1])).newpos.x) + g_fCollisionRange;
            tmp.minY = std::min((std::get<1>(masspoints[0])).newpos.y, (std::get<1>(masspoints[1])).newpos.y) - g_fCollisionRange;
            tmp.maxY = std::max((std::get<1>(masspoints[0])).newpos.y, (std::get<1>(masspoints[1])).newpos.y) + g_fCollisionRange;
            tmp.minZ = std::min((std::get<1>(masspoints[0])).newpos.z, (std::get<1>(masspoints[1])).newpos.z) - g_fCollisionRange;
            tmp.maxZ = std::max((std::get<1>(masspoints[0])).newpos.z, (std::get<1>(masspoints[1])).newpos.z) + g_fCollisionRange;
            ret.push_back(tmp);
        }

        return ret;
    }

    // three or more masspoints -> sort, divide, recursion
    else {

        // only sort valid masspoints (no -1 ID)
        MassIDVector tmp;
        uint valid = 0;
        while(std::get<0>(masspoints[valid]) != (-1) && valid < masspoints.size()){
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
        x = sort(MassIDVector(masspoints.begin(), masspoints.begin() + div), (mode + 1) % 3);
        y = sort(MassIDVector(masspoints.begin() + div, masspoints.end()), (mode + 1) % 3);
        xy.reserve(x.size() + y.size() + 1);

        BVBOX tmp2;                                     // tmp <- bounding box for each masspoint in masspoints
        tmp2.isLeaf = false;                            // tmp is container, not leaf
        tmp2.minX = std::min(x[0].minX, y[0].minX);
        tmp2.maxX = std::max(x[0].maxX, y[0].maxX);
        tmp2.minY = std::min(x[0].minY, y[0].minY);
        tmp2.maxY = std::max(x[0].maxY, y[0].maxY);
        tmp2.minZ = std::min(x[0].minZ, y[0].minZ);
        tmp2.maxZ = std::max(x[0].maxZ, y[0].maxZ);

        xy.push_back(tmp2);                             // vector[] <- {container, leftchildren, rightchildren}
        xy.insert(xy.end(), x.begin(), x.end());
        xy.insert(xy.end(), y.begin(), y.end());
        return xy;

    }
}