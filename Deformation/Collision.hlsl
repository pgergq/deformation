//--------------------------------------------------------------------------------------
// File: Collision.hlsl
//
// Collision detection compute shader
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------


struct BVBox {
    int left_id;
    int left_type;
    int right_id;
    int right_type;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
};

struct BVHDesc {
    uint array_offset;
    uint masspoint_count;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
};


