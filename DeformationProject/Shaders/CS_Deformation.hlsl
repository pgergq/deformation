//--------------------------------------------------------------------------------------
// File: CS_Deformation.hlsl
//
// Update volumetric model based on previous states and affecting forces
// Picking physics added
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------


// include structure definitions
#include "HH_DataStructures.hlsl"
#include "HH_CSConstantBuffer.hlsl"

// attached buffers
StructuredBuffer<MassPoint> ovolcube1   : register(t0);
StructuredBuffer<MassPoint> ovolcube2   : register(t1);
Texture2D<float4> vertexID1             : register(t2);
Texture2D<float4> vertexID2             : register(t3);
StructuredBuffer<BVHDesc> bvhdesc       : register(t4);
StructuredBuffer<BVBox> bvhdata         : register(t5);
RWStructuredBuffer<MassPoint> volcube1  : register(u0);
RWStructuredBuffer<MassPoint> volcube2  : register(u1);


// Return force/acceleration affecting the first input vertex (mass spring system, spring between the two vertices)
float3 acceleration(MassPoint a, MassPoint b, uint mode)
{
    float invlen = 1.0f / length(a.newpos - b.newpos);
    float3 v = (a.newpos - a.oldpos - b.newpos + b.oldpos).xyz / dt;// + ab*dt - aa*dt;
    float len;
    // 0 == same cube, 1 == other cube, 2 == same cube+second neighbour
    switch (mode){
        case 0: len = cube_cell_size; break;
        case 1: len = cube_cell_size * 0.5f * sqrt(3); break;
        case 2: len = cube_cell_size * 2; break;
        default: break;
    }

    // F(stiff) = ks * (xj-xi)/|xj-xi| * (|xj-xi| - l0)
    // F(damp) = kd * (vj-vi) * (xj-xi)/|xj-xi|
    // return F/m
    return (stiffness * normalize((b.newpos - a.newpos).xyz) * (length(a.newpos - b.newpos) - len) + damping * v) * im;
}


// collide to points in space (cpos = base point, xpos = colliding neighbour
float3 collide(float3 cpos, float3 xpos){

    // may the Force here be calculated
    float dist = length(cpos - xpos);
    float3 dir = normalize(cpos - xpos);
    float3 ret = float3(0, 0, 0);

    // repulsive force = direction * weight_from_distance
    ret = dir * min(exp_max, 1000 * exp2(collision_range - dist));      // exponential
    //ret = dir * min(exp_max, ((float)1000/collision_range)*(exp2((float)collision_range / dist)));     // hybrid
    //ret = dir * min(100000, 10 * (float)collision_range / dist);     // fractional

    return ret;
}


// x is between a and b values
bool between(float x, float a, float b){
    return (a <= x) && (x <= b);
}


// return collision detection result (colliding forces affecting the current masspoint)
float3 collision_detection(MassPoint old, uint objnum){

    float3 accel = float3(0, 0, 0);
    float4 cpos = old.newpos;

    // for every object in the simulation
    for (uint o = 0; o < object_count; o++){
        // colliding(?) object's desc
        BVHDesc colldesc = bvhdesc[o];
        // BVHTree offset in bvhdata[]
        uint arr_off = colldesc.array_offset;

        // o == objnum => self-collision
        if (o == objnum){
            // ***TODO***: self collision
        }

        // collision with another object
        else {
            // check for collision
            if (between(cpos.x, colldesc.min_x, colldesc.max_x) && between(cpos.y, colldesc.min_y, colldesc.max_y) && between(cpos.z, colldesc.min_z, colldesc.max_z))
            {
                // collision with the other object, compute forces (traverse tree)
                uint stack[32];
                stack[0] = 0;
                uint stacks = 1;
                uint maxlevel = log2(colldesc.masspoint_count + 1);

                // DFS in collision tree (max size: 2^32 tree)
                while (stacks > 0){
                    // get node index to check
                    stacks--;
                    uint index = stack[stacks];
                    // level index (0..n-1)
                    uint level = log2(index + 1);

                    // leaf level, bvboxes with two leaf-children
                    if (level == maxlevel - 1){
                        BVBox node = bvhdata[arr_off + index];
                        // collide left leaf
                        if (node.left_type == 1){
                            accel += collide(cpos.xyz, ovolcube1[o * cube_width * cube_width * cube_width + node.left_id].newpos.xyz);
                        }
                        else if (node.left_type == 2){
                            accel += collide(cpos.xyz, ovolcube2[o * (cube_width + 1) * (cube_width + 1) * (cube_width + 1) + node.left_id].newpos.xyz);
                        }
                        // collide right leaf
                        if (node.right_type == 1){
                            accel += collide(cpos.xyz, ovolcube1[o * cube_width * cube_width * cube_width + node.right_id].newpos.xyz);
                        }
                        else if (node.right_type == 2){
                            accel += collide(cpos.xyz, ovolcube2[o * (cube_width + 1) * (cube_width + 1) * (cube_width + 1) + node.right_id].newpos.xyz);
                        }
                    }
                    // node level, check children
                    else {
                        BVBox lnode = bvhdata[arr_off + index * 2 + 1];
                        BVBox rnode = bvhdata[arr_off + index * 2 + 2];
                        // colliding right children
                        if (between(cpos.x, rnode.min_x, rnode.max_x) && between(cpos.y, rnode.min_y, rnode.max_y) && between(cpos.z, rnode.min_z, rnode.max_z)){
                            stack[stacks] = index * 2 + 2;
                            stacks++;
                        }
                        // colliding left children
                        if (between(cpos.x, lnode.min_x, lnode.max_x) && between(cpos.y, lnode.min_y, lnode.max_y) && between(cpos.z, lnode.min_z, lnode.max_z)){
                            stack[stacks] = index * 2 + 1;
                            stacks++;
                        }
                    }
                }
            }
        }
    }
    return accel;
}



[numthreads(masspoint_tgsize, 1, 1)]
void CSMain1(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex)
{

    /// Helper variables
    uint full = DTid.x;
    // object masscube's index in masscube buffer
    uint objnum = full / (cube_width*cube_width*cube_width);
    // masspoint index in cube
    uint cube = full % (cube_width*cube_width*cube_width);
    uint z = cube / cube_width / cube_width;
    uint y = (cube - z*cube_width*cube_width) / cube_width;
    uint x = (cube - z*cube_width*cube_width - y*cube_width);

    // full indices in first and second masscube buffer
    uint ind = objnum*cube_width*cube_width*cube_width + z*cube_width*cube_width + y*cube_width + x;
    uint ind2 = objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + z*(cube_width + 1)*(cube_width + 1) + y*(cube_width + 1) + x;


    // old masspoint data
    MassPoint old = ovolcube1[ind];

    /// Picking
    // ID of picked masscube's lower left masspoint
    float4 pickID = vertexID1[uint2(pick_origin_x, pick_origin_y)];

    // if picking mode is on AND the pick didn't happen over a black pixel AND the picked vertex is adjacent to the current masspoint
    if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && (z + objnum * cube_width) == pickID.z)){
        float len = length(old.newpos.xyz - eye_pos.xyz);
        float3 v_curr = pick_dir.xyz * len + eye_pos.xyz;

        volcube1[ind].acc.xyz = float3(0, 0, 0);
        volcube1[ind].oldpos = old.newpos;
        volcube1[ind].newpos.xyz = v_curr;
    }

    /// Normal mode
    else {

        uint same = old.neighbour_same;
        uint other = old.neighbour_other;
        //0 = static masspoint, no neighbours
        int notstaticmass = (same | other) == 0 ? 0 : 1;
        if (notstaticmass == 0){
            return;
        }

        /// Sum neighbouring forces
        // init with gravity
        float3 accel = float3(0, notstaticmass*gravity*im, 0);

        // Get neighbours (immediate and second), set acceleration, with index checking
        // left neighbour
        if (same & NB_SAME_LEFT){
            accel += acceleration(old, ovolcube1[ind - 1], 0);
            //second to left
            if (x > 1 && (ovolcube1[ind - 1].neighbour_same & NB_SAME_LEFT))
                accel += acceleration(old, ovolcube1[ind - 2], 2);
        }
        // right neighbour
        if (same & NB_SAME_RIGHT){
            accel += acceleration(old, ovolcube1[ind + 1], 0);
            //second to right
            if (x < cube_width - 2 && (ovolcube1[ind + 1].neighbour_same & NB_SAME_RIGHT))
                accel += acceleration(old, ovolcube1[ind + 2], 2);
        }
        // lower neighbour
        if (same & NB_SAME_DOWN){
            accel += acceleration(old, ovolcube1[ind - cube_width], 0);
            //second down
            if (y > 1 && (ovolcube1[ind - cube_width].neighbour_same & NB_SAME_DOWN))
                accel += acceleration(old, ovolcube1[ind - 2 * cube_width], 2);
        }
        // upper neighbour
        if (same & NB_SAME_UP){
            accel += acceleration(old, ovolcube1[ind + cube_width], 0);
            //second up
            if (y < cube_width - 2 && (ovolcube1[ind + cube_width].neighbour_same & NB_SAME_UP))
                accel += acceleration(old, ovolcube1[ind + 2 * cube_width], 2);
        }
        // nearer neighbour
        if (same & NB_SAME_FRONT){
            accel += acceleration(old, ovolcube1[ind - cube_width*cube_width], 0);
            //second front
            if (z > 1 && (ovolcube1[ind - cube_width*cube_width].neighbour_same & NB_SAME_FRONT))
                accel += acceleration(old, ovolcube1[ind - 2 * cube_width * cube_width], 2);
        }
        // farther neighbour
        if (same & NB_SAME_BACK){
            accel += acceleration(old, ovolcube1[ind + cube_width*cube_width], 0);
            //second back
            if (z < cube_width - 2 && (ovolcube1[ind + cube_width*cube_width].neighbour_same & NB_SAME_BACK))
                accel += acceleration(old, ovolcube1[ind + 2 * cube_width * cube_width], 2);
        }

        // neighbours in second volcube
        if (other & NB_OTHER_NEAR_BOT_LEFT)
            accel += acceleration(old, ovolcube2[ind2], 1);
        if (other & NB_OTHER_NEAR_BOT_RIGHT)
            accel += acceleration(old, ovolcube2[ind2 + 1], 1);
        if (other & NB_OTHER_NEAR_TOP_LEFT)
            accel += acceleration(old, ovolcube2[ind2 + cube_width + 1], 1);
        if (other & NB_OTHER_NEAR_TOP_RIGHT)
            accel += acceleration(old, ovolcube2[ind2 + cube_width + 2], 1);
        if (other & NB_OTHER_FAR_BOT_LEFT)
            accel += acceleration(old, ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1)], 1);
        if (other & NB_OTHER_FAR_BOT_RIGHT)
            accel += acceleration(old, ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1) + 1], 1);
        if (other & NB_OTHER_FAR_TOP_LEFT)
            accel += acceleration(old, ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1) + cube_width + 1], 1);
        if (other & NB_OTHER_FAR_TOP_RIGHT)
            accel += acceleration(old, ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1) + cube_width + 2], 1);

        // collision detection
        accel += collision_detection(old, objnum);


        // Verlet + Acceleration
        // table
        if (old.newpos.y < table_pos && notstaticmass){
            //old: min(exp_max, 1000 * exp2(-old.newpos.y)*exp_mul)
            accel += float3(0, min(exp_max, 1000 * exp2(abs(old.newpos.y - table_pos))*exp_mul), 0);
        }
        volcube1[ind].acc.xyz = accel;
        volcube1[ind].oldpos = old.newpos;
        volcube1[ind].newpos.xyz = old.newpos.xyz * 2 - old.oldpos.xyz + accel*dt*dt;
    }
}

[numthreads(masspoint_tgsize, 1, 1)]
void CSMain2(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex)
{

    /// Helper variables
    uint full = DTid.x;
    uint objnum = full / ((cube_width + 1)*(cube_width + 1)*(cube_width + 1));
    uint cube = full % ((cube_width + 1)*(cube_width + 1)*(cube_width + 1));
    uint z = cube / (cube_width + 1) / (cube_width + 1);
    uint y = (cube - z*(cube_width + 1)*(cube_width + 1)) / (cube_width + 1);
    uint x = (cube - z*(cube_width + 1)*(cube_width + 1) - y*(cube_width + 1));
    uint ind = objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + z*(cube_width + 1)*(cube_width + 1) + y*(cube_width + 1) + x;
    uint ind1 = objnum*cube_width*cube_width*cube_width + z*cube_width*cube_width + y*cube_width + x;

    MassPoint old = ovolcube2[ind];

    /// Picking
    float4 pickID = vertexID2[uint2(pick_origin_x, pick_origin_y)];

    if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && (z + objnum*(cube_width + 1)) == pickID.z)){
        float len = length(old.newpos.xyz - eye_pos.xyz);
        float3 v_curr = pick_dir.xyz * len + eye_pos.xyz;

        volcube2[ind].acc.xyz = float3(0, 0, 0);
        volcube2[ind].oldpos = old.newpos;
        volcube2[ind].newpos.xyz = v_curr;
    }

    /// Normal mode
    else {

        uint same = old.neighbour_same;
        uint other = old.neighbour_other;
        int notstaticmass = (same | other) == 0 ? 0 : 1;
        if (notstaticmass == 0){
            return;
        }

        /// Sum neighbouring forces
        float3 accel = float3(0, notstaticmass*gravity*im, 0);

        // Get neighbours, set acceleration, with index checking
        if (same & NB_SAME_LEFT){
            accel += acceleration(old, ovolcube2[ind - 1], 0);
            if (x > 1 && (ovolcube2[ind - 1].neighbour_same & NB_SAME_LEFT))
                accel += acceleration(old, ovolcube2[ind - 2], 2);
        }
        if (same & NB_SAME_RIGHT){
            accel += acceleration(old, ovolcube2[ind + 1], 0);
            if (x < cube_width - 1 && (ovolcube2[ind + 1].neighbour_same & NB_SAME_RIGHT))
                accel += acceleration(old, ovolcube2[ind + 2], 2);
        }
        if (same & NB_SAME_DOWN){
            accel += acceleration(old, ovolcube2[ind - cube_width - 1], 0);
            if (y > 1 && (ovolcube2[ind - (cube_width + 1)].neighbour_same & NB_SAME_DOWN))
                accel += acceleration(old, ovolcube2[ind - 2 * (cube_width + 1)], 2);
        }
        if (same & NB_SAME_UP){
            accel += acceleration(old, ovolcube2[ind + cube_width + 1], 0);
            if (y < cube_width - 1 && (ovolcube2[ind + cube_width + 1].neighbour_same & NB_SAME_UP))
                accel += acceleration(old, ovolcube2[ind + 2 * (cube_width + 1)], 2);
        }
        if (same & NB_SAME_FRONT){
            accel += acceleration(old, ovolcube2[ind - (cube_width + 1)*(cube_width + 1)], 0);
            if (z > 1 && (ovolcube2[ind - (cube_width + 1)*(cube_width + 1)].neighbour_same & NB_SAME_FRONT))
                accel += acceleration(old, ovolcube2[ind - 2 * (cube_width + 1) * (cube_width + 1)], 2);
        }
        if (same & NB_SAME_BACK){
            accel += acceleration(old, ovolcube2[ind + (cube_width + 1)*(cube_width + 1)], 0);
            if (z < cube_width - 1 && (ovolcube2[ind + (cube_width + 1)*(cube_width + 1)].neighbour_same & NB_SAME_BACK))
                accel += acceleration(old, ovolcube2[ind + 2 * (cube_width + 1) * (cube_width + 1)], 2);
        }

        // neighbours in second volcube
        if (other & NB_OTHER_NEAR_BOT_LEFT)
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width - cube_width - 1], 1);
        if (other & NB_OTHER_NEAR_BOT_RIGHT)
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width - cube_width], 1);
        if (other & NB_OTHER_NEAR_TOP_LEFT)
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width - 1], 1);
        if (other & NB_OTHER_NEAR_TOP_RIGHT)
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width], 1);
        if (other & NB_OTHER_FAR_BOT_LEFT)
            accel += acceleration(old, ovolcube1[ind1 - cube_width - 1], 1);
        if (other & NB_OTHER_FAR_BOT_RIGHT)
            accel += acceleration(old, ovolcube1[ind1 - cube_width], 1);
        if (other & NB_OTHER_FAR_TOP_LEFT)
            accel += acceleration(old, ovolcube1[ind1 - 1], 1);
        if (other & NB_OTHER_FAR_TOP_RIGHT)
            accel += acceleration(old, ovolcube1[ind1], 1);

        // collision detection
        accel += collision_detection(old, objnum);

        // Verlet + Acceleration
        if (old.newpos.y < table_pos && notstaticmass){
            accel += float3(0, min(exp_max, 1000 * exp2(abs(old.newpos.y - table_pos))*exp_mul), 0);
        }
        volcube2[ind].acc.xyz = accel;
        volcube2[ind].oldpos = old.newpos;
        volcube2[ind].newpos.xyz = old.newpos.xyz * 2 - old.oldpos.xyz + accel*dt*dt;
    }
}