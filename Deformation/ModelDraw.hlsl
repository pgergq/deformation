//--------------------------------------------------------------------------------------
// File: ModelDraw.hlsl
//-------------------------------------------------------------------------------------

struct Particle
{
	float4 pos			: POSITION;
	float4 acc			: COLOR;
};

struct GParticle
{
	float4 pos			: SV_POSITION;
	float4 acc			: COLOR;
};

cbuffer cb0
{
    row_major float4x4 g_mWorldViewProj;
    row_major float4x4 g_mInvView;    
};

cbuffer cb1
{
    static float g_fParticleRad = 10.0f;   
};

cbuffer cbImmutable
{
    static float3 g_positions[4] =
    {
        float3( -1, 1, 0 ),
        float3( 1, 1, 0 ),
        float3( -1, -1, 0 ),
        float3( 1, -1, 0 ),
    };
    
    static float2 g_texcoords[4] = 
    { 
        float2(0,0), 
        float2(1,0),
        float2(0,1),
        float2(1,1),
    };
};

//
// Render model for dragging purposes
//
Particle VSModel(Particle input)
{
	Particle output = input;

	return output;
}

//
// Render model for dragging purposes
//
[maxvertexcount(3)]
void GSModel(point Particle input[1], inout TriangleStream<GParticle> SpriteStream)
{
    GParticle output;
    //for(int i=0; i<3; i++)
    //{

    //    float3 position = mul( input[i].pos.xyz, (float3x3)g_mInvView );
    //    output.pos = mul( float4(position,1.0), g_mWorldViewProj ); 
    //    output.acc = input[i].acc;        
    //    SpriteStream.Append(output);
    //}
	    //
    // Emit two new triangles
    //
    for(int i=0; i<4; i++)
    {
		float3 position = g_positions[i] * g_fParticleRad * 2.5f;
        position = mul( position, (float3x3)g_mInvView ) + input[0].pos.xyz;
        output.pos = mul( float4(position,1.0), g_mWorldViewProj ); 

        output.color = input[0].acc;        
        output.tex = g_texcoords[i];
        SpriteStream.Append(output);
    }
    SpriteStream.RestartStrip();
}


//
// Render model for dragging purposes
//
float4 PSModel(GParticle input) : SV_Target
{
	float4 output;

	return float4(1.0f,0.1f,0.1f,1.0f);
}