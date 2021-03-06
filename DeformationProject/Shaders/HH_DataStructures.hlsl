//--------------------------------------------------------------------------------------
// File: HH_DataStructures.hlsl
//
// Structure definitions for Deformation
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------
// PHYSICS
//--------------------------------------------------------------------------------------

// volcube neighbouring data
#define NB_SAME_LEFT			0x20
#define NB_SAME_RIGHT			0x10
#define NB_SAME_DOWN			0x08
#define NB_SAME_UP				0x04
#define NB_SAME_FRONT			0x02
#define NB_SAME_BACK			0x01
#define NB_OTHER_NEAR_BOT_LEFT	0x80
#define NB_OTHER_NEAR_BOT_RIGHT	0x40
#define NB_OTHER_NEAR_TOP_LEFT	0x20
#define NB_OTHER_NEAR_TOP_RIGHT	0x10
#define NB_OTHER_FAR_BOT_LEFT	0x08
#define NB_OTHER_FAR_BOT_RIGHT	0x04
#define NB_OTHER_FAR_TOP_LEFT	0x02
#define NB_OTHER_FAR_TOP_RIGHT	0x01

// constants
#define exp_mul                 0.05f           // default: 0.05f
#define exp_max                 1000000         // default: 1000000
#define masspoint_tgsize        256
#define particle_tgsize         256


// masspoint structure (defined in RenderObjects as well)
struct MassPoint
{
    float4 oldpos;          // masspoint position (t-1)
    float4 newpos;          // masspoint position (t)
    float4 acc;             // masspoint acceleration
    float4 color;           // masspoint colour
    uint neighbour_same;    // masspoint neighbours in the same masscube
    uint neighbour_other;   // masspoint neighbours in the other masscube
    uint local_id;          // id in masscube
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
    float nw2[8];           //          --||--
};

// BVHData entry (bounding box)
struct BVBox {
    int left_id;            // masspoint ID of left child (-1: not leaf), index in masscube!
    int left_type;          // masspoint type (1st or second masscube, 0: treenode, -1: tree leaf)
    int right_id;           // masspoint ID of right child, index in masscube!
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
    uint array_offset;      // BVHTree offset in global array (for the objects's BVHTree)
    uint masspoint_count;   // masspoint count in BVHTree
    float min_x;            // bounding box coordinates
    float max_x;            // bounding box coordinates
    float min_y;            // bounding box coordinates
    float max_y;            // bounding box coordinates
    float min_z;            // bounding box coordinates
    float max_z;            // bounding box coordinates
};

// Bone structure
struct Bone {
    uint4 bone_indices;
    float4 bone_weights;
};


//--------------------------------------------------------------------------------------
// GRAPHICS
//--------------------------------------------------------------------------------------

// face structure
struct Face
{
    uint4 vertices      : FACEVERTS;
};

// object vertex structure
struct BufferVertex
{
    float4 pos          : WPOS;
    float4 npos         : NPOS;
    float4 mpid1        : MPIDONE;
    float4 mpid2        : MPIDTWO;
};

// particle + light position
struct ObjectVertex
{
    float4 pos          : SV_Position;
    float4 lpos         : TEXCOORD0;
    float4 color        : COLOR;
};

// picking helper structure
struct PickingVertex
{
    float4 pos          : SV_Position;
    float3 mpid1        : MPIDONE;
    float3 mpid2        : MPIDTWO;
};

// shadow mapping structure
struct ShadowVertex
{
    float4 pos          : SV_Position;
};
