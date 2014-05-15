//--------------------------------------------------------------------------------------
// File: PosUpdate.hlsl
//
// Update main object's positions based on the changes in the volumetric cube
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#define maxv1	cube_width*cube_width*cube_width
#define maxv2	(cube_width+1)*(cube_width+1)*(cube_width+1)


struct Particle			// vertex in the input object
{
	float4 pos;
	float4 npos;
	float4 mpid1;
	float4 mpid2;
};

struct Indexer
{
	float3 vc1index;	// index in first volumetric cube
	float3 vc2index;	// index in second volumetric cube
	float w1[8];		// neighbour weights in first volumetric cube
	float w2[8];		// neighbour weights in second volumetric cube
	float nw1[8];		// neighbour weights for normal
	float nw2[8];		//			--||--
};

struct MassPoint		// vertex in the input volumetric cubes
{
	float4 oldpos;
	float4 newpos;
	float4 acc;
	uint neighbour_same;
	uint neighbour_other;
};

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

    float4 pick_dir;
    float4 eye_pos;
};

RWStructuredBuffer<Particle>  particles	: register(u0);
StructuredBuffer<Indexer>     indexer	: register(t0);
RWStructuredBuffer<MassPoint> volcube1	: register(u1);
RWStructuredBuffer<MassPoint> volcube2	: register(u2);

[numthreads(1, 1, 1)]
void PosUpdate(uint3 DTid : SV_DispatchThreadID)
{
	// Calculate new particle position
	int ind1 = indexer[DTid.x].vc1index.z*cube_width*cube_width + indexer[DTid.x].vc1index.y*cube_width + indexer[DTid.x].vc1index.x;
	int ind2 = indexer[DTid.x].vc2index.z*(cube_width + 1)*(cube_width + 1) + indexer[DTid.x].vc2index.y*(cube_width + 1) + indexer[DTid.x].vc2index.x;
	float3 pos1 = indexer[DTid.x].w1[0] * volcube1[ind1].newpos.xyz +
		indexer[DTid.x].w1[1] * volcube1[ind1 + 1].newpos.xyz +
		indexer[DTid.x].w1[2] * volcube1[ind1 + cube_width].newpos.xyz +
		indexer[DTid.x].w1[3] * volcube1[ind1 + cube_width + 1].newpos.xyz +
		indexer[DTid.x].w1[4] * volcube1[ind1 + cube_width*cube_width].newpos.xyz +
		indexer[DTid.x].w1[5] * volcube1[ind1 + cube_width*cube_width + 1].newpos.xyz +
		indexer[DTid.x].w1[6] * volcube1[ind1 + cube_width*cube_width + cube_width].newpos.xyz +
		indexer[DTid.x].w1[7] * volcube1[ind1 + cube_width*cube_width + cube_width + 1].newpos.xyz;
	float3 pos2 = indexer[DTid.x].w2[0] * volcube2[ind2].newpos.xyz +
		indexer[DTid.x].w2[1] * volcube2[ind2 + 1].newpos.xyz +
		indexer[DTid.x].w2[2] * volcube2[ind2 + (cube_width + 1)].newpos.xyz +
		indexer[DTid.x].w2[3] * volcube2[ind2 + (cube_width + 1) + 1].newpos.xyz +
		indexer[DTid.x].w2[4] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1)].newpos.xyz +
		indexer[DTid.x].w2[5] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + 1].newpos.xyz +
		indexer[DTid.x].w2[6] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1)].newpos.xyz +
		indexer[DTid.x].w2[7] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1) + 1].newpos.xyz;
	// Set final position
	particles[DTid.x].pos.xyz = pos1 * 0.5f + pos2 * 0.5f;

	// Calculate new normals
	float3 npos1 = indexer[DTid.x].nw1[0] * volcube1[ind1].newpos.xyz +
		indexer[DTid.x].nw1[1] * volcube1[ind1 + 1].newpos.xyz +
		indexer[DTid.x].nw1[2] * volcube1[ind1 + cube_width].newpos.xyz +
		indexer[DTid.x].nw1[3] * volcube1[ind1 + cube_width + 1].newpos.xyz +
		indexer[DTid.x].nw1[4] * volcube1[ind1 + cube_width*cube_width].newpos.xyz +
		indexer[DTid.x].nw1[5] * volcube1[ind1 + cube_width*cube_width + 1].newpos.xyz +
		indexer[DTid.x].nw1[6] * volcube1[ind1 + cube_width*cube_width + cube_width].newpos.xyz +
		indexer[DTid.x].nw1[7] * volcube1[ind1 + cube_width*cube_width + cube_width + 1].newpos.xyz;
	float3 npos2 = indexer[DTid.x].nw2[0] * volcube2[ind2].newpos.xyz +
		indexer[DTid.x].nw2[1] * volcube2[ind2 + 1].newpos.xyz +
		indexer[DTid.x].nw2[2] * volcube2[ind2 + (cube_width + 1)].newpos.xyz +
		indexer[DTid.x].nw2[3] * volcube2[ind2 + (cube_width + 1) + 1].newpos.xyz +
		indexer[DTid.x].nw2[4] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1)].newpos.xyz +
		indexer[DTid.x].nw2[5] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + 1].newpos.xyz +
		indexer[DTid.x].nw2[6] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1)].newpos.xyz +
		indexer[DTid.x].nw2[7] * volcube2[ind2 + (cube_width + 1)*(cube_width + 1) + (cube_width + 1) + 1].newpos.xyz;
	particles[DTid.x].npos.xyz = npos1 * 0.5f + npos2 * 0.5f;
}