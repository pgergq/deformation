//--------------------------------------------------------------------------------------
// File: Collision.hlsl
//
// Collision detection compute shader
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------


// include structure definitions
#include "Structures.hlsl"

StructuredBuffer<MassPoint> volcube1    : register(t0);
StructuredBuffer<MassPoint> volcube2    : register(t1);
StructuredBuffer<BVHDesc> obvhdesc      : register(t2);
StructuredBuffer<BVBox> obvhdata        : register(t3);
RWStructuredBuffer<BVHDesc> bvhdesc     : register(u0);
RWStructuredBuffer<BVBox> bvhdata       : register(u1);


[numthreads(1, 1, 1)]
void BVHUpdate(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex)
{
    /// Helper variables
    uint objnum = Gid.x;                                        // object line number
    uint offset = obvhdesc[objnum].array_offset;                // offset in bvhdata array
    uint maxlevel = log2(obvhdesc[objnum].masspoint_count + 1); // num of tree levels
    uint level = maxlevel;

    /// Update tree data
    while (level > 0){

        uint leveloffset = exp2(level - 1) - 1;                 // leveloffset = 2^(level-1)-1, number of nodes higher in the tree

        // nodes level, update BVBoxes
        if (level == maxlevel){
            // For every node on the current level, get min and max from masspoint data
            for (uint i = 0; i < leveloffset + 1; i++){

                BVBox tmp = bvhdata[offset + leveloffset + i];
                MassPoint ml, mr;
                if (tmp.left_id != (-1) && tmp.left_type == 1){ // left child valid, read from masscube data
                    // index: objnum * masscube1_size + index_in_cube
                    ml = volcube1[objnum*cube_width*cube_width*cube_width + tmp.left_id];
                }
                else if (tmp.left_id != (-1) && tmp.left_type == 2){
                    ml = volcube2[objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + tmp.left_id];
                }

                if (tmp.right_id != (-1) && tmp.right_type == 1){ // right child valid, read from masscube data
                    // index: objnum * masscube(1|2)_size + index_in_cube
                    mr = volcube1[objnum*cube_width*cube_width*cube_width + tmp.right_id];
                }
                else if (tmp.right_id != (-1) && tmp.right_type == 2){
                    mr = volcube2[objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + tmp.right_id];
                }

                bvhdata[offset + leveloffset + i].min_x = min(ml.newpos.x, mr.newpos.x) - collision_range;
                bvhdata[offset + leveloffset + i].max_x = max(ml.newpos.x, mr.newpos.x) + collision_range;
                bvhdata[offset + leveloffset + i].min_y = min(ml.newpos.y, mr.newpos.y) - collision_range;
                bvhdata[offset + leveloffset + i].max_y = max(ml.newpos.y, mr.newpos.y) + collision_range;
                bvhdata[offset + leveloffset + i].min_y = min(ml.newpos.z, mr.newpos.z) - collision_range;
                bvhdata[offset + leveloffset + i].max_y = max(ml.newpos.z, mr.newpos.z) + collision_range;
            }
        }

        // upper in the tree, update from two children
        else {
            // For every node on the current level, get min and max from left and right children
            for (uint i = 0; i < leveloffset + 1; i++){
                bvhdata[offset + leveloffset + i].max_x = max(bvhdata[offset + 2 * (leveloffset + i) + 1].max_x, bvhdata[offset + 2 * (leveloffset + i) + 2].max_x);
                bvhdata[offset + leveloffset + i].min_x = min(bvhdata[offset + 2 * (leveloffset + i) + 1].min_x, bvhdata[offset + 2 * (leveloffset + i) + 2].min_x);
                bvhdata[offset + leveloffset + i].min_y = min(bvhdata[offset + 2 * (leveloffset + i) + 1].min_y, bvhdata[offset + 2 * (leveloffset + i) + 2].min_y);
                bvhdata[offset + leveloffset + i].max_y = max(bvhdata[offset + 2 * (leveloffset + i) + 1].max_y, bvhdata[offset + 2 * (leveloffset + i) + 2].max_y);
                bvhdata[offset + leveloffset + i].min_z = min(bvhdata[offset + 2 * (leveloffset + i) + 1].min_z, bvhdata[offset + 2 * (leveloffset + i) + 2].min_z);
                bvhdata[offset + leveloffset + i].max_z = max(bvhdata[offset + 2 * (leveloffset + i) + 1].max_z, bvhdata[offset + 2 * (leveloffset + i) + 2].max_z);
            }
        }

        level--;
    }

    /// Update catalogue
    bvhdesc[objnum].min_x = bvhdata[offset].min_x;      // offset = index of tree root in global array
    bvhdesc[objnum].max_x = bvhdata[offset].max_x;
    bvhdesc[objnum].min_y = bvhdata[offset].min_y;
    bvhdesc[objnum].max_y = bvhdata[offset].max_y;
    bvhdesc[objnum].min_z = bvhdata[offset].min_z;
    bvhdesc[objnum].max_z = bvhdata[offset].max_z;
}