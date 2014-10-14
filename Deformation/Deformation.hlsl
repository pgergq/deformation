//--------------------------------------------------------------------------------------
// File: Deformation.hlsl
//
// Update volumetric model based on previous states and affecting forces
// Picking physics added
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------


// include structure definitions
#include "Structures.hlsl"

// attached buffers
StructuredBuffer<MassPoint> ovolcube1	: register(t0);
StructuredBuffer<MassPoint> ovolcube2	: register(t1);
Texture2D<float4> vertexID1				: register(t2);
Texture2D<float4> vertexID2				: register(t3);
StructuredBuffer<BVHDesc> bvhdesc       : register(t4);
StructuredBuffer<BVBox> bvhdata         : register(t5);
RWStructuredBuffer<MassPoint> volcube1	: register(u0);
RWStructuredBuffer<MassPoint> volcube2	: register(u1);


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

// generate index in first volcube from coordinates
uint index_1(uint x, uint y, uint z){
    return z*cube_width*cube_width + y*cube_width + x;
}

// generate index in second volcube from coordinates
uint index_2(uint x, uint y, uint z){
    return z*(cube_width + 1)*(cube_width + 1) + y*(cube_width + 1) + x;
}

// x is between a and b values
bool between(float x, float a, float b){
    return (a <= x) && (x <= b);
}



[numthreads(masspoint_tgsize, 1, 1)]
void CSMain1(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex)
{

    /// Helper variables
    //uint full = Gid.z*cube_width*cube_width + Gid.y*cube_width + Gid.x; // old call
    uint full = DTid.x;
    uint objnum = full / (cube_width*cube_width*cube_width);    // object masscube's line number in masscube buffer
    uint cube = full % (cube_width*cube_width*cube_width);      // masspoint index in cube
    uint z = cube / cube_width / cube_width;                    // cube z coord
    uint y = (cube - z*cube_width*cube_width) / cube_width;     // cube y coord
    uint x = (cube - z*cube_width*cube_width - y*cube_width);   // cube x coord

    // full indices in first and second masscube buffer
    uint ind = objnum*cube_width*cube_width*cube_width + z*cube_width*cube_width + y*cube_width + x;
    uint ind2 = objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + z*(cube_width + 1)*(cube_width + 1) + y*(cube_width + 1) + x;
    
    // old masspoint data
    MassPoint old = ovolcube1[ind];

	/// Picking
	float4 pickID = vertexID1[uint2(pick_origin_x, pick_origin_y)];				// ID of picked masscube's lower left masspoint

	// if picking mode is on AND the pick didn't happen over a black pixel AND the picked vertex is adjacent to the current masspoint
	if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && (z + objnum * cube_width) == pickID.z)){
		float len = length(old.newpos.xyz - eye_pos.xyz);			// current
		float3 v_curr = pick_dir.xyz * len + eye_pos.xyz;							// current position

		volcube1[ind].acc.xyz = float3(0, 0, 0);
		volcube1[ind].oldpos = old.newpos;
		volcube1[ind].newpos.xyz = v_curr;
	}

    /// Normal mode
	else {

		uint same = old.neighbour_same;
		uint other = old.neighbour_other;
		int notstaticmass = (same | other) == 0 ? 0 : 1;	//0 = static masspoint, no neighbours

		/// Sum neighbouring forces
		float3 accel = float3(0, notstaticmass*gravity*im, 0);		// gravity

		// Get neighbours, set acceleration, with index checking
		// neighbours in the same volcube + second neighbours
		if (same & NB_SAME_LEFT){				// left neighbour
			accel += acceleration(old, ovolcube1[ind - 1], 0);
			if (x > 1 && (ovolcube1[ind - 1].neighbour_same & NB_SAME_LEFT))				//second to left
				accel += acceleration(old, ovolcube1[ind - 2], 2);
		}
		if (same & NB_SAME_RIGHT){				// right neighbour 
            accel += acceleration(old, ovolcube1[ind + 1], 0);
			if (x < cube_width - 2 && (ovolcube1[ind + 1].neighbour_same & NB_SAME_RIGHT))		//second to right
                accel += acceleration(old, ovolcube1[ind + 2], 2);
		}
		if (same & NB_SAME_DOWN){				// lower neighbour
            accel += acceleration(old, ovolcube1[ind - cube_width], 0);
			if (y > 1 && (ovolcube1[ind - cube_width].neighbour_same & NB_SAME_DOWN))			//second down
                accel += acceleration(old, ovolcube1[ind - 2 * cube_width], 2);
		}
		if (same & NB_SAME_UP){				// upper neighbour
            accel += acceleration(old, ovolcube1[ind + cube_width], 0);
			if (y < cube_width - 2 && (ovolcube1[ind + cube_width].neighbour_same & NB_SAME_UP))	//second up
                accel += acceleration(old, ovolcube1[ind + 2 * cube_width], 2);
		}
		if (same & NB_SAME_FRONT){				// nearer neighbour
            accel += acceleration(old, ovolcube1[ind - cube_width*cube_width], 0);
			if (z > 1 && (ovolcube1[ind - cube_width*cube_width].neighbour_same & NB_SAME_FRONT))			//second front
                accel += acceleration(old, ovolcube1[ind - 2 * cube_width * cube_width], 2);
		}
		if (same & NB_SAME_BACK){				// farther neighbour
            accel += acceleration(old, ovolcube1[ind + cube_width*cube_width], 0);
			if (z < cube_width - 2 && (ovolcube1[ind + cube_width*cube_width].neighbour_same & NB_SAME_BACK))	//second back
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

        
        // Collision detection
        float4 cpos = old.newpos;                   // current masspoint position
        for (uint o = 0; o < object_count; o++){    // for every object in the simulation

            BVHDesc colldesc = bvhdesc[o];          // colliding? object's desc
            
            // o == objnum => self-collision
            if (o == objnum){
                // *TODO*
            }

            // collision with another object
            else {
                // do we have collision?
                if (between(cpos.x, colldesc.min_x, colldesc.max_x) && between(cpos.y, colldesc.min_y, colldesc.max_y) && between(cpos.z, colldesc.min_z, colldesc.max_z)){
                    
                    // collision with the other object, compute forces (traverse tree)
                    bool tree[2048];    // *TODO* ???
                    tree[0] = tree[1] = tree[2] = true;
                    uint maxlevel = log2(colldesc.masspoint_count + 1);
                    for (uint i = 1; i < maxlevel; i++){                                    // i = level number, root level already checked (collision)
                        uint leveloffset = exp2(i) - 1;                                     // first node index of current level

                        for (uint j = leveloffset; j < leveloffset * 2 + 1; j++){           // j = node index int the whole tree (for every node on the current level)
                            BVBox node = bvhdata[j];
                            // not leaf level -> go below
                            if (i != maxlevel - 1){
                                // colliding parent box AND inside bounding box -> check children nodes
                                // *TODO*: optimize axes
                                if (tree[j] && between(cpos.x, node.min_x, node.max_x) && between(cpos.y, node.min_x, node.max_y) && between(cpos.z, node.min_z, node.max_z)){
                                    tree[j * 2 + 1] = true;
                                    tree[j * 2 + 2] = true;
                                }
                                else {
                                    tree[j * 2 + 1] = false;
                                    tree[j * 2 + 2] = false;
                                }
                            }
                            // leaf level -> calculate force
                            else {
                                if (tree[j] && node.left_type != -1){                        // valid left leaf, calculate force
                                    float4 xpos = (node.left_type == 1) ? ovolcube1[o*cube_width*cube_width*cube_width + node.left_id].newpos 
                                                                        : ovolcube2[o*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + node.left_id].newpos;
                                    float3 dir = normalize((cpos - xpos).xyz);
                                    // *TODO*: may the Force here be calculated

                                }
                                if (tree[j] && node.right_type != -1){                       // valid right leaf, calculate force
                                    float4 xpos = (node.right_type == 1) ? ovolcube1[o*cube_width*cube_width*cube_width + node.right_id].newpos
                                                                         : ovolcube2[o*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + node.right_id].newpos;
                                    float3 dir = normalize((cpos - xpos).xyz);
                                    // *TODO*: may the Force here be calculated
                                }
                            }
                        }
                    }
                }
            }
        }
        //////////////////////

		// Verlet + Acceleration
        if (old.newpos.y < table_pos && notstaticmass){				// table
            accel += float3(0, min(exp_max, 1000 * exp2(-old.newpos.y)*exp_mul), 0);
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
    //uint full = Gid.z*(cube_width + 1)*(cube_width + 1) + Gid.y*(cube_width + 1) + Gid.x; // old call
    uint full = DTid.x;
    uint objnum = full / ((cube_width + 1)*(cube_width + 1)*(cube_width + 1));       // object masscube's line number in masscube buffer
    uint cube = full % ((cube_width + 1)*(cube_width + 1)*(cube_width + 1));         // masspoint index in cube
    uint z = cube / (cube_width + 1) / (cube_width + 1);                          // cube z coord
    uint y = (cube - z*(cube_width + 1)*(cube_width + 1)) / (cube_width + 1);        // cube y coord
    uint x = (cube - z*(cube_width + 1)*(cube_width + 1) - y*(cube_width + 1));      // cube x coord

    // full indices in first and second masscube buffer
    uint ind = objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + z*(cube_width + 1)*(cube_width + 1) + y*(cube_width + 1) + x;
    uint ind1 = objnum*cube_width*cube_width*cube_width + z*cube_width*cube_width + y*cube_width + x;

    // old masspoint
    MassPoint old = ovolcube2[ind];

	/// Picking
	float4 pickID = vertexID2[uint2(pick_origin_x, pick_origin_y)];				    // ID of picked masscube's lower left masspoint

	// if picking mode is on AND the pick didn't happen over a black pixel AND the picked vertex is adjacent to the current masspoint
    if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && (z + objnum*(cube_width + 1)) == pickID.z)){
        float len = length(old.newpos.xyz - eye_pos.xyz);			// current 
		float3 v_curr = pick_dir.xyz * len + eye_pos.xyz;							// current position

		volcube2[ind].acc.xyz = float3(0, 0, 0);
        volcube2[ind].oldpos = old.newpos;
		volcube2[ind].newpos.xyz = v_curr;
	}

    /// Normal mode
	else {

        uint same = old.neighbour_same;
        uint other = old.neighbour_other;
		int notstaticmass = (same | other) == 0 ? 0 : 1;	//0 = static masspoint, no neighbours

		/// Sum neighbouring forces
		float3 accel = float3(0, notstaticmass*gravity*im, 0);		// gravity

		// Get neighbours, set acceleration, with index checking
		// neighbours in the same volcube + second neighbours
		if (same & NB_SAME_LEFT){				// left neighbour
            accel += acceleration(old, ovolcube2[ind - 1], 0);
			if (x > 1 && (ovolcube2[ind - 1].neighbour_same & NB_SAME_LEFT))				//second to left
                accel += acceleration(old, ovolcube2[ind - 2], 2);
		}
		if (same & NB_SAME_RIGHT){				// right neighbour 
            accel += acceleration(old, ovolcube2[ind + 1], 0);
			if (x < cube_width - 1 && (ovolcube2[ind + 1].neighbour_same & NB_SAME_RIGHT))		//second to right
                accel += acceleration(old, ovolcube2[ind + 2], 2);
		}
		if (same & NB_SAME_DOWN){				// lower neighbour
            accel += acceleration(old, ovolcube2[ind - cube_width - 1], 0);
			if (y > 1 && (ovolcube2[ind - (cube_width + 1)].neighbour_same & NB_SAME_DOWN))	    //second down
                accel += acceleration(old, ovolcube2[ind - 2 * (cube_width + 1)], 2);
		}
		if (same & NB_SAME_UP){					// upper neighbour
            accel += acceleration(old, ovolcube2[ind + cube_width + 1], 0);
			if (y < cube_width - 1 && (ovolcube2[ind + cube_width + 1].neighbour_same & NB_SAME_UP))	//second up
                accel += acceleration(old, ovolcube2[ind + 2 * (cube_width + 1)], 2);
		}
		if (same & NB_SAME_FRONT){				// nearer neighbour
            accel += acceleration(old, ovolcube2[ind - (cube_width + 1)*(cube_width + 1)], 0);
			if (z > 1 && (ovolcube2[ind - (cube_width + 1)*(cube_width + 1)].neighbour_same & NB_SAME_FRONT))				//second front
                accel += acceleration(old, ovolcube2[ind - 2 * (cube_width + 1) * (cube_width + 1)], 2);
		}
		if (same & NB_SAME_BACK){				// farther neighbour
            accel += acceleration(old, ovolcube2[ind + (cube_width + 1)*(cube_width + 1)], 0);
			if (z < cube_width - 1 && (ovolcube2[ind + (cube_width + 1)*(cube_width + 1)].neighbour_same & NB_SAME_BACK))	//second back
                accel += acceleration(old, ovolcube2[ind + 2 * (cube_width + 1) * (cube_width + 1)], 2);
		}

		// neighbours in second volcube
		if (other & NB_OTHER_NEAR_BOT_LEFT)	// front, bottomleft
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width - cube_width - 1], 1);
		if (other & NB_OTHER_NEAR_BOT_RIGHT)	// front, bottomright
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width - cube_width], 1);
		if (other & NB_OTHER_NEAR_TOP_LEFT)	// front, topleft
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width - 1], 1);
		if (other & NB_OTHER_NEAR_TOP_RIGHT)	// front, topright
            accel += acceleration(old, ovolcube1[ind1 - cube_width*cube_width], 1);
		if (other & NB_OTHER_FAR_BOT_LEFT)		// back, bottomleft
            accel += acceleration(old, ovolcube1[ind1 - cube_width - 1], 1);
		if (other & NB_OTHER_FAR_BOT_RIGHT)	// back, bottomright
            accel += acceleration(old, ovolcube1[ind1 - cube_width], 1);
		if (other & NB_OTHER_FAR_TOP_LEFT)		// back, topleft
            accel += acceleration(old, ovolcube1[ind1 - 1], 1);
		if (other & NB_OTHER_FAR_TOP_RIGHT)	// back, topright
            accel += acceleration(old, ovolcube1[ind1], 1);


        // Collision detection








        //////////////////////

		// Verlet + Acceleration
        if (old.newpos.y < table_pos && notstaticmass){  // table
            accel += float3(0, min(exp_max, 1000 * exp2(-old.newpos.y)*exp_mul), 0);
		}
		volcube2[ind].acc.xyz = accel;
        volcube2[ind].oldpos = old.newpos;
        volcube2[ind].newpos.xyz = old.newpos.xyz * 2 - old.oldpos.xyz + accel*dt*dt;
	}
}