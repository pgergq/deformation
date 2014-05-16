//--------------------------------------------------------------------------------------
// File: Structures.hlsl
//
// Structure definitions for Deformation
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------



// masspoint structure
struct MassPoint
{
    float4 oldpos;          // masspoint position (t-1)
    float4 newpos;          // masspoint position (t)
    float4 acc;             // masspoint acceleration
    uint neighbour_same;    // masspoint neighbours in the same masscube
    uint neighbour_other;   // masspoint neighbours in the other masscube
};

// object vertex structure
struct Particle
{
    float4 pos;             // vertex position
    float4 npos;            // vertex normal position (end of vector)
    float4 mpid1;           // masspoint ID in first masscube
    float4 mpid2;           // masspoint ID in second masscube
};

// indexer entry structure
struct Indexer
{
    float3 vc1index;        // index in first volumetric cube
    float3 vc2index;        // index in second volumetric cube
    float w1[8];            // neighbour weights in first volumetric cube
    float w2[8];            // neighbour weights in second volumetric cube
    float nw1[8];           // neighbour weights for normal
    float nw2[8];           //			--||--
};

// BVHData entry (bounding box)
struct BVBox {
    int left_id;            // masspoint ID of left child (-1: not leaf)
    int left_type;          // masspoint type (1st or second masscube)
    int right_id;           // masspoint ID of right child
    int right_type;         // masspoint type
    float min_x;            // bounding box coordinates
    float max_x;            // bounding box coordinates
    float min_y;            // bounding box coordinates
    float max_y;            // bounding box coordinates
    float min_z;            // bounding box coordinates
    float max_z;            // bounding box coordinates
};

// BVHCatalogue entry (for each object)
struct BVHDesc {
    uint array_offset;      // BVHTree offset in global array (for the entry's BVHTree)
    uint masspoint_count;   // masspoint count in BVHTree
    float min_x;            // bounding box coordinates
    float max_x;            // bounding box coordinates
    float min_y;            // bounding box coordinates
    float max_y;            // bounding box coordinates
    float min_z;            // bounding box coordinates
    float max_z;            // bounding box coordinates
};
