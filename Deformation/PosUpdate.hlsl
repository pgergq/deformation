//--------------------------------------------------------------------------------------
// File: PosUpdate.hlsl
//
// Update main object's positions based on the changes in the volumetric cube
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------


// include structure definitions
#include "Structures.hlsl"

RWStructuredBuffer<Particle> particles	: register(u0);
StructuredBuffer<Indexer> indexer   	: register(t0);
RWStructuredBuffer<MassPoint> volcube1	: register(u1);
RWStructuredBuffer<MassPoint> volcube2	: register(u2);

[numthreads(particle_tgsize, 1, 1)]
void PosUpdate(uint3 DTid : SV_DispatchThreadID)
{
	// Calculate new particle position
    Indexer old = indexer[DTid.x];
    int ind1 = old.vc1index.z*cube_width*cube_width + old.vc1index.y*cube_width + old.vc1index.x;
	int ind2 = old.vc2index.z*(cube_width + 1)*(cube_width + 1) + old.vc2index.y*(cube_width + 1) + old.vc2index.x;
    float3 pos1 = old.w1[0] * volcube1[ind1].newpos.xyz +
		old.w1[1] * volcube1[ind1 + 1].newpos.xyz +
		old.w1[2] * volcube1[ind1 + cube_width].newpos.xyz +
		old.w1[3] * volcube1[ind1 + cube_width + 1].newpos.xyz +
		old.w1[4] * volcube1[ind1 + cube_width*cube_width].newpos.xyz +
		old.w1[5] * volcube1[ind1 + cube_width*cube_width + 1].newpos.xyz +
		old.w1[6] * volcube1[ind1 + cube_width*cube_width + cube_width].newpos.xyz +
		old.w1[7] * volcube1[ind1 + cube_width*cube_width + cube_width + 1].newpos.xyz;
	float3 pos2 = old.w2[0] * volcube2[ind2].newpos.xyz +
		old.w2[1] * volcube2[ind2 + 1].newpos.xyz +
		old.w2[2] * volcube2[ind2 + (cube_width + 1)].newpos.xyz +
		old.w2[3] * volcube2[ind2 + (cube_width + 1) + 1].newpos.xyz +
		old.w2[4] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1)].newpos.xyz +
		old.w2[5] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + 1].newpos.xyz +
		old.w2[6] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1)].newpos.xyz +
		old.w2[7] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1) + 1].newpos.xyz;
	// Set final position
	particles[DTid.x].pos.xyz = pos1 * 0.5f + pos2 * 0.5f;

	// Calculate new normals
	float3 npos1 = old.nw1[0] * volcube1[ind1].newpos.xyz +
		old.nw1[1] * volcube1[ind1 + 1].newpos.xyz +
		old.nw1[2] * volcube1[ind1 + cube_width].newpos.xyz +
		old.nw1[3] * volcube1[ind1 + cube_width + 1].newpos.xyz +
		old.nw1[4] * volcube1[ind1 + cube_width*cube_width].newpos.xyz +
		old.nw1[5] * volcube1[ind1 + cube_width*cube_width + 1].newpos.xyz +
		old.nw1[6] * volcube1[ind1 + cube_width*cube_width + cube_width].newpos.xyz +
		old.nw1[7] * volcube1[ind1 + cube_width*cube_width + cube_width + 1].newpos.xyz;
	float3 npos2 = old.nw2[0] * volcube2[ind2].newpos.xyz +
		old.nw2[1] * volcube2[ind2 + 1].newpos.xyz +
		old.nw2[2] * volcube2[ind2 + (cube_width + 1)].newpos.xyz +
		old.nw2[3] * volcube2[ind2 + (cube_width + 1) + 1].newpos.xyz +
		old.nw2[4] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1)].newpos.xyz +
		old.nw2[5] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + 1].newpos.xyz +
		old.nw2[6] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1)].newpos.xyz +
		old.nw2[7] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1) + 1].newpos.xyz;
	particles[DTid.x].npos.xyz = npos1 * 0.5f + npos2 * 0.5f;
}