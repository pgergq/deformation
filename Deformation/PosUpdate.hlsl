//--------------------------------------------------------------------------------------
// File: PosUpdate.hlsl
//
// Update main object's positions based on the changes in the volumetric cube
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#define maxv1	vcwidth*vcwidth*vcwidth
#define maxv2	(vcwidth+1)*(vcwidth+1)*(vcwidth+1)


struct Particle			// vertex in the input object
{
	float4 pos;
	float4 normal;
	float4 mpid1;
	float4 mpid2;
};

struct Indexer
{
	float3 vc1index;	// index in first volumetric cube
	float w1[8];		// neighbour weights in first volumetric cube
	float3 vc2index;	// index in second volumetric cube
	float w2[8];		// neighbour weights in second volumetric cube
};

struct MassPoint		// vertex in the input volumetric cubes
{
	float4 oldpos;
	float4 newpos;
    float4 acc;
	uint neighbour_same;
	uint neighbour_other;
};

cbuffer cbCS : register( b0 )
{
	uint vcwidth;			// number of masspoint in the smaller volcube in one row
	uint vccell;			// size of one volcube cell
	uint numparticles;		// total count of model vertices
	
	uint is_picking;		// bool for mouse picking
	uint pickOriginX;		// windows height
	uint pickOriginY;		// windows width
	uint dummy1;			// dummy
	uint dummy2;
	float4 pickDir;			// picking vector direction
	float4 eyePos;			// eye position

	float stiffness;		// stiffness
	float damping;			// damping, negative!
	float dt;				// delta time
	float im;				// inverse mass of masspoints
};

RWStructuredBuffer<Particle>  particles	: register( u0 );
StructuredBuffer<Indexer>     indexer	: register( t0 );
RWStructuredBuffer<MassPoint> volcube1	: register( u1 );
RWStructuredBuffer<MassPoint> volcube2	: register( u2 );

[numthreads(1, 1, 1)]
void PosUpdate( uint3 DTid : SV_DispatchThreadID )
{
	// vertex# in DTid.x : 
	//						0->numparticles = model vertices,
	//						numparticles->numparticles+maxv1 = volcube1,
	//						numparticles+maxv1->numparticles+maxv1+maxv2 = volcube2
	
	if(DTid.x < numparticles){
		// Calculate new particle position
		float3 oldppos = particles[DTid.x].pos.xyz;
		int ind1 = indexer[DTid.x].vc1index.z*vcwidth*vcwidth + indexer[DTid.x].vc1index.y*vcwidth + indexer[DTid.x].vc1index.x;
		int ind2 = indexer[DTid.x].vc2index.z*(vcwidth+1)*(vcwidth+1) + indexer[DTid.x].vc2index.y*(vcwidth+1) + indexer[DTid.x].vc2index.x;
		float3 pos1 = indexer[DTid.x].w1[0] * volcube1[ind1].newpos.xyz + 
					  indexer[DTid.x].w1[1] * volcube1[ind1+1].newpos.xyz + 
					  indexer[DTid.x].w1[2] * volcube1[ind1+vcwidth].newpos.xyz + 
					  indexer[DTid.x].w1[3] * volcube1[ind1+vcwidth+1].newpos.xyz + 
					  indexer[DTid.x].w1[4] * volcube1[ind1+vcwidth*vcwidth].newpos.xyz + 
					  indexer[DTid.x].w1[5] * volcube1[ind1+vcwidth*vcwidth+1].newpos.xyz + 
					  indexer[DTid.x].w1[6] * volcube1[ind1+vcwidth*vcwidth+vcwidth].newpos.xyz + 
					  indexer[DTid.x].w1[7] * volcube1[ind1+vcwidth*vcwidth+vcwidth+1].newpos.xyz;
		float3 pos2 = indexer[DTid.x].w2[0] * volcube2[ind2].newpos.xyz + 
					  indexer[DTid.x].w2[1] * volcube2[ind2+1].newpos.xyz + 
					  indexer[DTid.x].w2[2] * volcube2[ind2+(vcwidth+1)].newpos.xyz + 
					  indexer[DTid.x].w2[3] * volcube2[ind2+(vcwidth+1)+1].newpos.xyz + 
					  indexer[DTid.x].w2[4] * volcube2[ind2+(vcwidth+1)*(vcwidth+1)].newpos.xyz + 
					  indexer[DTid.x].w2[5] * volcube2[ind2+(vcwidth+1)*(vcwidth+1)+1].newpos.xyz + 
					  indexer[DTid.x].w2[6] * volcube2[ind2+(vcwidth+1)*(vcwidth+1)+(vcwidth+1)].newpos.xyz + 
					  indexer[DTid.x].w2[7] * volcube2[ind2+(vcwidth+1)*(vcwidth+1)+(vcwidth+1)+1].newpos.xyz;
		// Set final position
		particles[DTid.x].pos.xyz = pos1 * 0.5f + pos2 * 0.5f;

		// Calculate new normals
		float3 
		
	} else if(DTid.x >= numparticles && DTid.x < numparticles+maxv1){
		particles[DTid.x].pos.xyz = volcube1[DTid.x-numparticles].newpos.xyz;
	} else {
		particles[DTid.x].pos.xyz = volcube2[DTid.x-numparticles-maxv1].newpos.xyz;
	}
}