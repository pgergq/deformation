//--------------------------------------------------------------------------------------
// File: ParticleDraw.hlsl
//
// Shaders for rendering the particle as point sprite (particle -> 2 triangles)
// (green particles highlighted)
//
// @(Copyright (c) Microsoft Corporation. All rights reserved. - NBodyGravityCS11 project)
// @Modified by pgq
//--------------------------------------------------------------------------------------

struct VSParticleIn
{
    float4  color   : COLOR;
    uint    id      : SV_VertexID;
};

struct VSParticleDrawOut
{
    float3 pos			: POSITION;
    float3 normal		: TEXCOORD3;
    float3 mpid1		: TEXCOORD1;
    float3 mpid2		: TEXCOORD2;
    float4 color		: COLOR;
};

struct GSParticleDrawOut
{
    float2 tex			: TEXCOORD0;
    float3 normal		: TEXCOORD3;
    float3 mpid1		: TEXCOORD1;
    float3 mpid2		: TEXCOORD2;
    float4 color		: COLOR;
    float4 pos			: SV_Position;
};

struct PSParticleDrawIn
{
    float2 tex			: TEXCOORD0;
    float3 normal		: TEXCOORD3;
    float3 mpid1		: TEXCOORD1;
    float3 mpid2		: TEXCOORD2;
    float4 color		: COLOR;
};

struct Particle
{
    float4 pos			: SV_Position;
    float4 normal		: TEXCOORD3;
    float4 mpid1		: TEXCOORD1;
    float4 mpid2		: TEXCOORD2;
};

Texture2D		            g_txDiffuse;
StructuredBuffer<Particle>   g_bufParticle;


SamplerState g_samLinear
{
    Filter = MIN_MAG_MIP_LINEAR;
    AddressU = Clamp;
    AddressV = Clamp;
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
// Vertex shader for drawing the point-sprite particles
//
VSParticleDrawOut VSParticleDraw(VSParticleIn input)
{
    VSParticleDrawOut output;
    
    output.pos = g_bufParticle[input.id].pos.xyz;
    output.normal = g_bufParticle[input.id].normal.xyz;
    output.mpid1 = g_bufParticle[input.id].mpid1.xyz;
    output.mpid2 = g_bufParticle[input.id].mpid2.xyz;

    // load colour from Particle color
    output.color = input.color;
    
    return output;
}

//
// GS for rendering point sprite particles.  Takes a point and turns it into 2 tris.
//
[maxvertexcount(4)]
void GSParticleDraw(point VSParticleDrawOut input[1], inout TriangleStream<GSParticleDrawOut> SpriteStream)
{

    GSParticleDrawOut output;
    
    for(int i=0; i<4; i++)
    {
        //highlight green particles
        float green = input[0].color.g;
        if((green-0.9f) > 0) green = 2.5f;
        else green = 1.0f;

        float3 position = g_positions[i] * g_fParticleRad * green;
        position = mul( position, (float3x3)g_mInvView ) + input[0].pos;
        output.pos = mul( float4(position,1.0), g_mWorldViewProj ); 
        output.color = input[0].color;        
        output.tex = g_texcoords[i];
        output.normal = normalize(mul(input[0].normal, (float3x3)g_mInvView));	//supposedly correct world normal
        output.mpid1 = input[0].mpid1;
        output.mpid2 = input[0].mpid2;
        SpriteStream.Append(output);
    }
    SpriteStream.RestartStrip();
}

//
// PS for drawing particles
//
float4 PSParticleDraw(PSParticleDrawIn input) : SV_Target
{   
    return g_txDiffuse.Sample( g_samLinear, input.tex ) * input.color;
}

//
// PS for drawing model (picking)
//
float4 PSModelDraw1(PSParticleDrawIn input) : SV_Target
{   
    return float4(input.mpid1,1.0f);
}

//
// PS for drawing model (picking)
//
float4 PSModelDraw2(PSParticleDrawIn input) : SV_Target
{   
    return float4(input.mpid2,1.0f);
}