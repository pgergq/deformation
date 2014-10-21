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
    uint offset = obvhdesc[objnum].array_offset;                // offset in bvhdata array, index of tree root in global array
    uint maxlevel = log2(obvhdesc[objnum].masspoint_count + 1); // num of tree levels
    uint level = maxlevel;
    BVHDesc olddesc = bvhdesc[objnum];                          // old descriptor entry
    BVBox olddata = bvhdata[offset];                            // old tree root entry

    /// Update tree data
    while (level > 0){

        uint leveloffset = exp2(level - 1) - 1;                 // leveloffset = 2^(level-1)-1, number of nodes higher in the tree

        // leaf level, update BVBoxes
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

                BVBox equ = { tmp.left_id, tmp.left_type, tmp.right_id, tmp.right_type,
                              min(ml.newpos.x, mr.newpos.x) - collision_range,
                              max(ml.newpos.x, mr.newpos.x) + collision_range,
                              min(ml.newpos.y, mr.newpos.y) - collision_range,
                              max(ml.newpos.y, mr.newpos.y) + collision_range,
                              min(ml.newpos.z, mr.newpos.z) - collision_range,
                              max(ml.newpos.z, mr.newpos.z) + collision_range };
                bvhdata[offset + leveloffset + i] = equ;
            }
        }

        // upper in the tree, update from two children
        else {
            // For every node on the current level, get min and max from left and right children
            for (uint i = 0; i < leveloffset + 1; i++){
                BVBox tmp = bvhdata[offset + leveloffset + i];
                BVBox a = bvhdata[offset + 2 * (leveloffset + i) + 1];
                BVBox b = bvhdata[offset + 2 * (leveloffset + i) + 2];
                BVBox equ = { tmp.left_id, tmp.left_type, tmp.right_id, tmp.right_type,     
                              min(a.min_x, b.min_x), max(a.max_x, b.max_x),
                              min(a.min_y, b.min_y), max(a.max_y, b.max_y),
                              min(a.min_z, b.min_z), max(a.max_z, b.max_z) };
                bvhdata[offset + leveloffset + i] = equ;
            }
        }

        level--;
    }

    /// Update catalogue
    BVHDesc eqv = { olddesc.array_offset, olddesc.masspoint_count, 
                    olddata.min_x, olddata.max_x, olddata.min_y, 
                    olddata.max_y, olddata.min_z, olddata.max_z };
    bvhdesc[objnum] = eqv;
}