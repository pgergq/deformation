//--------------------------------------------------------------------------------------
// File: Deformation.hlsl
//
// Update volumetric model based on previous states and affecting forces
// Picking physics added
//
// @Copyright (c) pgq
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
#define exp_mul					0.05f		// default: 0.05f
#define exp_max					1000000		// default: 1000000

// include structure definitions
#include "Structures.hlsl"

// attached buffers
StructuredBuffer<MassPoint> ovolcube1	: register(t0);
StructuredBuffer<MassPoint> ovolcube2	: register(t1);
Texture2D<float4> vertexID1				: register(t2);
Texture2D<float4> vertexID2				: register(t3);
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



[numthreads(1, 1, 1)]
void CSMain1(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex)
{

    /// Helper variables
    uint full = Gid.z*cube_width*cube_width + Gid.y*cube_width + Gid.x;
    uint objnum = full / (cube_width*cube_width*cube_width);    // object masscube's line number in masscube buffer
    uint cube = full % (cube_width*cube_width*cube_width);      // masspoint index in cube
    uint z = cube / cube_width / cube_width;                  // cube z coord
    uint y = (cube - z*cube_width*cube_width) / cube_width;      // cube y coord
    uint x = (cube - z*cube_width*cube_width - y*cube_width);    // cube x coord
    // full indeces in first and second masscube buffer
    uint ind = objnum*cube_width*cube_width*cube_width + z*cube_width*cube_width + y*cube_width + x;
    uint ind2 = objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + z*(cube_width + 1)*(cube_width + 1) + y*(cube_width + 1) + x;
    

	/// Picking
	float4 pickID = vertexID1[uint2(pick_origin_x, pick_origin_y)];				// ID of picked masscube's lower left masspoint

	// if picking mode is on AND the pick didn't happen over a black pixel AND the picked vertex is adjacent to the current masspoint
	if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && (z+objnum*cube_width) == pickID.z)){
		float len = length(ovolcube1[ind].newpos.xyz - eye_pos.xyz);			// current
		float3 v_curr = pick_dir.xyz * len + eye_pos.xyz;							// current position

		volcube1[ind].acc.xyz = float3(0, 0, 0);
		volcube1[ind].oldpos = ovolcube1[ind].newpos;
		volcube1[ind].newpos.xyz = v_curr;
	}

    /// Normal mode
	else {

		uint same = ovolcube1[ind].neighbour_same;
		uint other = ovolcube1[ind].neighbour_other;
		int notstaticmass = (same | other) == 0 ? 0 : 1;	//0 = static masspoint, no neighbours

		/// Sum neighbouring forces
		float3 accel = float3(0, notstaticmass*gravity*im, 0);		// gravity

		// Get neighbours, set acceleration, with index checking
		// neighbours in the same volcube + second neighbours
		if (same & NB_SAME_LEFT){				// left neighbour
			accel += acceleration(ovolcube1[ind], ovolcube1[ind - 1], 0);
			if (x > 1 && (ovolcube1[ind - 1].neighbour_same & NB_SAME_LEFT))				//second to left
				accel += acceleration(ovolcube1[ind], ovolcube1[ind - 2], 2);
		}
		if (same & NB_SAME_RIGHT){				// right neighbour 
			accel += acceleration(ovolcube1[ind], ovolcube1[ind + 1], 0);
			if (x < cube_width - 2 && (ovolcube1[ind + 1].neighbour_same & NB_SAME_RIGHT))		//second to right
				accel += acceleration(ovolcube1[ind], ovolcube1[ind + 2], 2);
		}
		if (same & NB_SAME_DOWN){				// lower neighbour
			accel += acceleration(ovolcube1[ind], ovolcube1[ind - cube_width], 0);
			if (y > 1 && (ovolcube1[ind - cube_width].neighbour_same & NB_SAME_DOWN))			//second down
				accel += acceleration(ovolcube1[ind], ovolcube1[ind - 2 * cube_width], 2);
		}
		if (same & NB_SAME_UP){				// upper neighbour
			accel += acceleration(ovolcube1[ind], ovolcube1[ind + cube_width], 0);
			if (y < cube_width - 2 && (ovolcube1[ind + cube_width].neighbour_same & NB_SAME_UP))	//second up
				accel += acceleration(ovolcube1[ind], ovolcube1[ind + 2 * cube_width], 2);
		}
		if (same & NB_SAME_FRONT){				// nearer neighbour
			accel += acceleration(ovolcube1[ind], ovolcube1[ind - cube_width*cube_width], 0);
			if (z > 1 && (ovolcube1[ind - cube_width*cube_width].neighbour_same & NB_SAME_FRONT))			//second front
				accel += acceleration(ovolcube1[ind], ovolcube1[ind - 2 * cube_width * cube_width], 2);
		}
		if (same & NB_SAME_BACK){				// farther neighbour
			accel += acceleration(ovolcube1[ind], ovolcube1[ind + cube_width*cube_width], 0);
			if (z < cube_width - 2 && (ovolcube1[ind + cube_width*cube_width].neighbour_same & NB_SAME_BACK))	//second back
				accel += acceleration(ovolcube1[ind], ovolcube1[ind + 2 * cube_width * cube_width], 2);
		}

		// neighbours in second volcube
		if (other & NB_OTHER_NEAR_BOT_LEFT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2], 1);
		if (other & NB_OTHER_NEAR_BOT_RIGHT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + 1], 1);
		if (other & NB_OTHER_NEAR_TOP_LEFT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + cube_width + 1], 1);
		if (other & NB_OTHER_NEAR_TOP_RIGHT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + cube_width + 2], 1);
		if (other & NB_OTHER_FAR_BOT_LEFT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1)], 1);
		if (other & NB_OTHER_FAR_BOT_RIGHT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1) + 1], 1);
		if (other & NB_OTHER_FAR_TOP_LEFT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1) + cube_width + 1], 1);
		if (other & NB_OTHER_FAR_TOP_RIGHT)
			accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (cube_width + 1)*(cube_width + 1) + cube_width + 2], 1);


		//Verlet + Acceleration
		if (ovolcube1[ind].newpos.y < table_pos && notstaticmass){				// table
			accel += float3(0, min(exp_max, 1000 * exp2(-ovolcube1[ind].newpos.y)*exp_mul), 0);
		}
		volcube1[ind].acc.xyz = accel;
		volcube1[ind].oldpos = ovolcube1[ind].newpos;
		volcube1[ind].newpos.xyz = ovolcube1[ind].newpos.xyz * 2 - ovolcube1[ind].oldpos.xyz + accel*dt*dt;
	}
}

[numthreads(1, 1, 1)]
void CSMain2(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex)
{

    /// Helper variables
    uint full = Gid.z*(cube_width + 1)*(cube_width + 1) + Gid.y*(cube_width + 1) + Gid.x;
    uint objnum = full / ((cube_width + 1)*(cube_width + 1)*(cube_width + 1));       // object masscube's line number in masscube buffer
    uint cube = full % ((cube_width + 1)*(cube_width + 1)*(cube_width + 1));         // masspoint index in cube
    uint z = cube / (cube_width + 1) / (cube_width + 1);                          // cube z coord
    uint y = (cube - z*(cube_width + 1)*(cube_width + 1)) / (cube_width + 1);        // cube y coord
    uint x = (cube - z*(cube_width + 1)*(cube_width + 1) - y*(cube_width + 1));      // cube x coord
    // full indeces in first and second masscube buffer
    uint ind = objnum*(cube_width + 1)*(cube_width + 1)*(cube_width + 1) + z*(cube_width + 1)*(cube_width + 1) + y*(cube_width + 1) + x;
    uint ind1 = objnum*cube_width*cube_width*cube_width + z*cube_width*cube_width + y*cube_width + x;


	/// Picking
	float4 pickID = vertexID2[uint2(pick_origin_x, pick_origin_y)];				    // ID of picked masscube's lower left masspoint

	// if picking mode is on AND the pick didn't happen over a black pixel AND the picked vertex is adjacent to the current masspoint
    if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && (z + objnum*(cube_width + 1)) == pickID.z)){
		float len = length(ovolcube2[ind].newpos.xyz - eye_pos.xyz);			// current 
		float3 v_curr = pick_dir.xyz * len + eye_pos.xyz;							// current position

		volcube2[ind].acc.xyz = float3(0, 0, 0);
		volcube2[ind].oldpos = ovolcube2[ind].newpos;
		volcube2[ind].newpos.xyz = v_curr;
	}

    /// Normal mode
	else {

		uint same = ovolcube2[ind].neighbour_same;
		uint other = ovolcube2[ind].neighbour_other;
		int notstaticmass = (same | other) == 0 ? 0 : 1;	//0 = static masspoint, no neighbours

		/// Sum neighbouring forces
		float3 accel = float3(0, notstaticmass*gravity*im, 0);		// gravity

		// Get neighbours, set acceleration, with index checking
		// neighbours in the same volcube + second neighbours
		if (same & NB_SAME_LEFT){				// left neighbour
			accel += acceleration(ovolcube2[ind], ovolcube2[ind - 1], 0);
			if (x > 1 && (ovolcube2[ind - 1].neighbour_same & NB_SAME_LEFT))				//second to left
				accel += acceleration(ovolcube2[ind], ovolcube2[ind - 2], 2);
		}
		if (same & NB_SAME_RIGHT){				// right neighbour 
			accel += acceleration(ovolcube2[ind], ovolcube2[ind + 1], 0);
			if (x < cube_width - 1 && (ovolcube2[ind + 1].neighbour_same & NB_SAME_RIGHT))		//second to right
				accel += acceleration(ovolcube2[ind], ovolcube2[ind + 2], 2);
		}
		if (same & NB_SAME_DOWN){				// lower neighbour
			accel += acceleration(ovolcube2[ind], ovolcube2[ind - cube_width - 1], 0);
			if (y > 1 && (ovolcube2[ind - (cube_width + 1)].neighbour_same & NB_SAME_DOWN))	    //second down
				accel += acceleration(ovolcube2[ind], ovolcube2[ind - 2 * (cube_width + 1)], 2);
		}
		if (same & NB_SAME_UP){					// upper neighbour
			accel += acceleration(ovolcube2[ind], ovolcube2[ind + cube_width + 1], 0);
			if (y < cube_width - 1 && (ovolcube2[ind + cube_width + 1].neighbour_same & NB_SAME_UP))	//second up
				accel += acceleration(ovolcube2[ind], ovolcube2[ind + 2 * (cube_width + 1)], 2);
		}
		if (same & NB_SAME_FRONT){				// nearer neighbour
			accel += acceleration(ovolcube2[ind], ovolcube2[ind - (cube_width + 1)*(cube_width + 1)], 0);
			if (z > 1 && (ovolcube2[ind - (cube_width + 1)*(cube_width + 1)].neighbour_same & NB_SAME_FRONT))				//second front
				accel += acceleration(ovolcube2[ind], ovolcube2[ind - 2 * (cube_width + 1) * (cube_width + 1)], 2);
		}
		if (same & NB_SAME_BACK){				// farther neighbour
			accel += acceleration(ovolcube2[ind], ovolcube2[ind + (cube_width + 1)*(cube_width + 1)], 0);
			if (z < cube_width - 1 && (ovolcube2[ind + (cube_width + 1)*(cube_width + 1)].neighbour_same & NB_SAME_BACK))	//second back
				accel += acceleration(ovolcube2[ind], ovolcube2[ind + 2 * (cube_width + 1) * (cube_width + 1)], 2);
		}

		// neighbours in second volcube
		if (other & NB_OTHER_NEAR_BOT_LEFT)	// front, bottomleft
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - cube_width*cube_width - cube_width - 1], 1);
		if (other & NB_OTHER_NEAR_BOT_RIGHT)	// front, bottomright
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - cube_width*cube_width - cube_width], 1);
		if (other & NB_OTHER_NEAR_TOP_LEFT)	// front, topleft
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - cube_width*cube_width - 1], 1);
		if (other & NB_OTHER_NEAR_TOP_RIGHT)	// front, topright
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - cube_width*cube_width], 1);
		if (other & NB_OTHER_FAR_BOT_LEFT)		// back, bottomleft
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - cube_width - 1], 1);
		if (other & NB_OTHER_FAR_BOT_RIGHT)	// back, bottomright
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - cube_width], 1);
		if (other & NB_OTHER_FAR_TOP_LEFT)		// back, topleft
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - 1], 1);
		if (other & NB_OTHER_FAR_TOP_RIGHT)	// back, topright
			accel += acceleration(ovolcube2[ind], ovolcube1[ind1], 1);

		//Verlet + Acceleration
		if (ovolcube2[ind].newpos.y < table_pos && notstaticmass){  // table
			accel += float3(0, min(exp_max, 1000 * exp2(-ovolcube2[ind].newpos.y)*exp_mul), 0);
		}
		volcube2[ind].acc.xyz = accel;
		volcube2[ind].oldpos = ovolcube2[ind].newpos;
		volcube2[ind].newpos.xyz = ovolcube2[ind].newpos.xyz * 2 - ovolcube2[ind].oldpos.xyz + accel*dt*dt;
	}
}