//--------------------------------------------------------------------------------------
// File: HH_CSConstantBuffer.hlsl
//
// Constant buffer structure definition for Deformation
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------



// constant buffer for compute shaders
cbuffer cbCS : register(b0)
{
    uint cube_width;
    uint cube_cell_size;
    uint object_count;

    uint is_picking;
    uint pick_origin_x;
    uint pick_origin_y;

    float stiffness;
    float damping;
    float dt;
    float im;
    float gravity;
    float table_pos;
    float collision_range;

    float3 dummy;

    float4 pick_dir;
    float4 eye_pos;
};