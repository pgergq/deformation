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

#define exp_mul					0.05f		//0.05f
#define exp_max					1000000		//1000000
#define gravity					-1000		//-1000
#define tablepos				-1000


cbuffer cbCS : register(b0)
{
	uint vcwidth;			// number of masspoint in the smaller volcube in one row
	uint vccell;			// size of one volcube cell
	uint numparticles;		// total count of model vertices

	uint is_picking;		// bool for mouse picking
	uint pickOriginX;		// pick origin X
	uint pickOriginY;		// pick origin Y
	uint dummy1;			// dummy
	uint dummy2;			// dummy
	float4 pickDir;			// picking vector direction
	float4 eyePos;			// eye position

	float stiffness;		// stiffness
	float damping;			// damping, negative!
	float dt;				// delta time
	float im;				// inverse mass of masspoints
};

struct MassPoint		// vertex in the input volumetric cubes
{
	float4 oldpos;
	float4 newpos;
	float4 acc;
	uint neighbour_same;
	uint neighbour_other;
};

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
	float3 v = (a.newpos - a.oldpos - b.newpos + b.oldpos) / dt;// + ab*dt - aa*dt;
	float len;
	// 0 == same cube, 1 == other cube, 2 == same cube+second neighbour
	switch (mode){
        case 0: len = vccell; break;
        case 1: len = vccell * 0.5f * sqrt(3); break;
        case 2: len = vccell * 2; break;
        default: break;
	}

	// F(stiff) = ks * (xj-xi)/|xj-xi| * (|xj-xi| - l0)
	// F(damp) = kd * (vj-vi) * (xj-xi)/|xj-xi|
	// return F/m
	return (stiffness * normalize(b.newpos - a.newpos) * (length(a.newpos - b.newpos) - len) + damping * v) * im;
}


[numthreads(1, 1, 1)]
void CSMain1(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex)
{
	/// Helper variables
	uint z = Gid.z; uint y = Gid.y; uint x = Gid.x;
	uint ind = z*vcwidth*vcwidth + y*vcwidth + x;
	uint ind2 = z*(vcwidth + 1)*(vcwidth + 1) + y*(vcwidth + 1) + x;
	//uint max = vcwidth*vcwidth*vcwidth;

	/// Picking
	float4 pickID = vertexID1[uint2(pickOriginX, pickOriginY)];				// ID of picked masscube's lower left masspoint
		// if picking mode is on AND the pick didn't happen over a black pixel AND the picked vertex is adjacent to the current masspoint
		if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && z == pickID.z)){
			float len = length(ovolcube1[ind].newpos.xyz - eyePos.xyz);			// current
			float3 v_curr = pickDir * len + eyePos.xyz;							// current position

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
				if (x < vcwidth - 2 && (ovolcube1[ind + 1].neighbour_same & NB_SAME_RIGHT))		//second to right
					accel += acceleration(ovolcube1[ind], ovolcube1[ind + 2], 2);
			}
			if (same & NB_SAME_DOWN){				// lower neighbour
				accel += acceleration(ovolcube1[ind], ovolcube1[ind - vcwidth], 0);
				if (y > 1 && (ovolcube1[ind - vcwidth].neighbour_same & NB_SAME_DOWN))			//second down
					accel += acceleration(ovolcube1[ind], ovolcube1[ind - 2 * vcwidth], 2);
			}
			if (same & NB_SAME_UP){				// upper neighbour
				accel += acceleration(ovolcube1[ind], ovolcube1[ind + vcwidth], 0);
				if (y < vcwidth - 2 && (ovolcube1[ind + vcwidth].neighbour_same & NB_SAME_UP))	//second up
					accel += acceleration(ovolcube1[ind], ovolcube1[ind + 2 * vcwidth], 2);
			}
			if (same & NB_SAME_FRONT){				// nearer neighbour
				accel += acceleration(ovolcube1[ind], ovolcube1[ind - vcwidth*vcwidth], 0);
				if (z > 1 && (ovolcube1[ind - vcwidth*vcwidth].neighbour_same & NB_SAME_FRONT))			//second front
					accel += acceleration(ovolcube1[ind], ovolcube1[ind - 2 * vcwidth * vcwidth], 2);
			}
			if (same & NB_SAME_BACK){				// farther neighbour
				accel += acceleration(ovolcube1[ind], ovolcube1[ind + vcwidth*vcwidth], 0);
				if (z < vcwidth - 2 && (ovolcube1[ind + vcwidth*vcwidth].neighbour_same & NB_SAME_BACK))	//second back
					accel += acceleration(ovolcube1[ind], ovolcube1[ind + 2 * vcwidth * vcwidth], 2);
			}

			// neighbours in second volcube
			if (other & NB_OTHER_NEAR_BOT_LEFT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2], 1);
			if (other & NB_OTHER_NEAR_BOT_RIGHT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + 1], 1);
			if (other & NB_OTHER_NEAR_TOP_LEFT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + vcwidth + 1], 1);
			if (other & NB_OTHER_NEAR_TOP_RIGHT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + vcwidth + 2], 1);
			if (other & NB_OTHER_FAR_BOT_LEFT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (vcwidth + 1)*(vcwidth + 1)], 1);
			if (other & NB_OTHER_FAR_BOT_RIGHT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (vcwidth + 1)*(vcwidth + 1) + 1], 1);
			if (other & NB_OTHER_FAR_TOP_LEFT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (vcwidth + 1)*(vcwidth + 1) + vcwidth + 1], 1);
			if (other & NB_OTHER_FAR_TOP_RIGHT)
				accel += acceleration(ovolcube1[ind], ovolcube2[ind2 + (vcwidth + 1)*(vcwidth + 1) + vcwidth + 2], 1);


			//Verlet + Acceleration
			if (ovolcube1[ind].newpos.y < tablepos && notstaticmass){				// table
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
	uint z = Gid.z; uint y = Gid.y; uint x = Gid.x;
	uint ind = z*(vcwidth + 1)*(vcwidth + 1) + y*(vcwidth + 1) + x;
	uint ind1 = z*vcwidth*vcwidth + y*vcwidth + x;

	/// Picking
	float4 pickID = vertexID2[uint2(pickOriginX, pickOriginY)];				    // ID of picked masscube's lower left masspoint
		// if picking mode is on AND the pick didn't happen over a black pixel AND the picked vertex is adjacent to the current masspoint
		if (is_picking && pickID.w != 0 && (x == pickID.x && y == pickID.y && z == pickID.z)){
			float len = length(ovolcube2[ind].newpos.xyz - eyePos.xyz);			// current 
			float3 v_curr = pickDir * len + eyePos.xyz;							// current position

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
				if (x < vcwidth - 1 && (ovolcube2[ind + 1].neighbour_same & NB_SAME_RIGHT))		//second to right
					accel += acceleration(ovolcube2[ind], ovolcube2[ind + 2], 2);
			}
			if (same & NB_SAME_DOWN){				// lower neighbour
				accel += acceleration(ovolcube2[ind], ovolcube2[ind - vcwidth - 1], 0);
				if (y > 1 && (ovolcube2[ind - (vcwidth + 1)].neighbour_same & NB_SAME_DOWN))	    //second down
					accel += acceleration(ovolcube2[ind], ovolcube2[ind - 2 * (vcwidth + 1)], 2);
			}
			if (same & NB_SAME_UP){					// upper neighbour
				accel += acceleration(ovolcube2[ind], ovolcube2[ind + vcwidth + 1], 0);
				if (y < vcwidth - 1 && (ovolcube2[ind + vcwidth + 1].neighbour_same & NB_SAME_UP))	//second up
					accel += acceleration(ovolcube2[ind], ovolcube2[ind + 2 * (vcwidth + 1)], 2);
			}
			if (same & NB_SAME_FRONT){				// nearer neighbour
				accel += acceleration(ovolcube2[ind], ovolcube2[ind - (vcwidth + 1)*(vcwidth + 1)], 0);
				if (z > 1 && (ovolcube2[ind - (vcwidth + 1)*(vcwidth + 1)].neighbour_same & NB_SAME_FRONT))				//second front
					accel += acceleration(ovolcube2[ind], ovolcube2[ind - 2 * (vcwidth + 1) * (vcwidth + 1)], 2);
			}
			if (same & NB_SAME_BACK){				// farther neighbour
				accel += acceleration(ovolcube2[ind], ovolcube2[ind + (vcwidth + 1)*(vcwidth + 1)], 0);
				if (z < vcwidth - 1 && (ovolcube2[ind + (vcwidth + 1)*(vcwidth + 1)].neighbour_same & NB_SAME_BACK))	//second back
					accel += acceleration(ovolcube2[ind], ovolcube2[ind + 2 * (vcwidth + 1) * (vcwidth + 1)], 2);
			}

			// neighbours in second volcube
			if (other & NB_OTHER_NEAR_BOT_LEFT)	// front, bottomleft
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - vcwidth*vcwidth - vcwidth - 1], 1);
			if (other & NB_OTHER_NEAR_BOT_RIGHT)	// front, bottomright
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - vcwidth*vcwidth - vcwidth], 1);
			if (other & NB_OTHER_NEAR_TOP_LEFT)	// front, topleft
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - vcwidth*vcwidth - 1], 1);
			if (other & NB_OTHER_NEAR_TOP_RIGHT)	// front, topright
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - vcwidth*vcwidth], 1);
			if (other & NB_OTHER_FAR_BOT_LEFT)		// back, bottomleft
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - vcwidth - 1], 1);
			if (other & NB_OTHER_FAR_BOT_RIGHT)	// back, bottomright
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - vcwidth], 1);
			if (other & NB_OTHER_FAR_TOP_LEFT)		// back, topleft
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1 - 1], 1);
			if (other & NB_OTHER_FAR_TOP_RIGHT)	// back, topright
				accel += acceleration(ovolcube2[ind], ovolcube1[ind1], 1);

			//Verlet + Acceleration
			if (ovolcube2[ind].newpos.y < tablepos && notstaticmass){  // table
				accel += float3(0, min(exp_max, 1000 * exp2(-ovolcube2[ind].newpos.y)*exp_mul), 0);
			}
			volcube2[ind].acc.xyz = accel;
			volcube2[ind].oldpos = ovolcube2[ind].newpos;
			volcube2[ind].newpos.xyz = ovolcube2[ind].newpos.xyz * 2 - ovolcube2[ind].oldpos.xyz + accel*dt*dt;
		}
}