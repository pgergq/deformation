//--------------------------------------------------------------------------------------
// File: GX_RenderObjects.hlsl
//
// Shaders for rendering the particle as point sprite (particle -> 2 triangles)
// (green particles highlighted)
//
// @pgq
//--------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------
// DEFINITIONS
//--------------------------------------------------------------------------------------
#include "HH_DataStructures.hlsl"


#define depth_bias        0.00001f


StructuredBuffer<BufferVertex> g_bufParticle : register(t0);
StructuredBuffer<Face> g_bufFace : register(t1);
Texture2D<float> g_txShadowMap : register(t2);
StructuredBuffer<MassPoint> g_bufMasspoint : register(t3);
Texture2D g_txDiffuse : register(t0);


SamplerState g_samLinear : register(s0);

SamplerState g_samBilinear
{
    Filter = MIN_MAG_MIP_LINEAR;
    AddressU = MIRROR;
    AddressV = MIRROR;
};

SamplerState g_samPoint
{
    Filter = MIN_MAG_MIP_LINEAR;
    AddressU = MIRROR;
    AddressV = MIRROR;
};

cbuffer cb0
{
    row_major float4x4 g_mWorldViewProj;
    row_major float4x4 g_mInvView;
    row_major float4x4 g_mLightViewProj;
    float4 eyepos;
    float4 lightpos;
    float4 lightcol;
};

cbuffer cb1
{
    static float g_fParticleRad = 10.0f;
};

cbuffer cbImmutable
{
    static float3 g_positions[4] =
    {
        float3(-1, 1, 0),
        float3(1, -1, 0),
        float3(-1, -1, 0),
        float3(1, 1, 0),
    };

    static float2 g_texcoords[4] =
    {
        float2(0, 1),
        float2(1, 0),
        float2(0, 0),
        float2(1, 1),
    };

    static float4 g_table[4] = 
    {
        float4(-10000, -1000, -10000, 1),
        float4(10000, -1000, -10000, 1),
        float4(10000, -1000, 10000, 1),
        float4(-10000, -1000, 10000, 1)
    };
};


//--------------------------------------------------------------------------------------
// OBJECT RENDERING
//--------------------------------------------------------------------------------------

//
// Blinn-Phong shading, árány
//
float4 BlinnPhong(float3 P, float3 N)
{
    // árány
    //float4 ka = float4(0.24725, 0.1995, 0.0745, 1);
    //float4 kd = float4(0.75164, 0.60648, 0.22648, 1);
    //float4 ks = float4(0.628281, 0.555802, 0.366065, 1);
    //float shininess = 80;

    // türkiz
    float4 ka = float4(0.1, 0.18725, 0.1745, 1);
    float4 kd = float4(0.096, 0.94151, 0.09102, 1);
    float4 ks = float4(0.297254, 0.30829, 0.306678, 1);
    float shininess = 20;
    
    float3 L = normalize(lightpos.xyz - P);
    float3 V = normalize(eyepos.xyz - P);
    float3 H = normalize(L + V);
    float3 diffuse = max(0, saturate(dot(L, N)));
    float3 spec = pow(saturate(dot(N, H)), shininess);
    return ka + kd * float4(diffuse, 1) + ks * float4(spec, 1);
}

//
// Vertex shader for drawing the point-sprite particles (object drawing)
//
Face VSObjectDraw(uint id : SV_VertexID)
{
    return g_bufFace[id];
}

//
// GS for rendering point sprite particles.  Takes a point and turns it into 2 tris. (object drawing)
//
[maxvertexcount(6)]
void GSObjectDraw(uint id : SV_PrimitiveID, point Face input[1], inout TriangleStream<ObjectVertex> SpriteStream)
{
    ObjectVertex output;
    uint points[3] = { input[0].vertices.x, input[0].vertices.y, input[0].vertices.z };

    //table draw
    if (id == 0){
        output.color = float4(0.7f, 0.7f, 0.7f, 1);
        // first half
        output.pos = mul(g_table[0], g_mWorldViewProj);
        output.lpos = mul(g_table[0], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[1], g_mWorldViewProj);
        output.lpos = mul(g_table[1], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[2], g_mWorldViewProj);
        output.lpos = mul(g_table[2], g_mLightViewProj);
        SpriteStream.Append(output);
        SpriteStream.RestartStrip();
        // second half
        output.pos = mul(g_table[0], g_mWorldViewProj);
        output.lpos = mul(g_table[0], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[2], g_mWorldViewProj);
        output.lpos = mul(g_table[2], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[3], g_mWorldViewProj);
        output.lpos = mul(g_table[3], g_mLightViewProj);
        SpriteStream.Append(output);
    }

    // draw object face
    for (int i = 0; i<3; i++)
    {
        BufferVertex vertex = g_bufParticle[points[2 - i]];
        output.pos = mul(vertex.pos, g_mWorldViewProj);
        output.lpos = mul(vertex.pos, g_mLightViewProj);

        //calculate lighting data
        // vertex normal
        float3 n = normalize(vertex.npos.xyz - vertex.pos.xyz);
        // opposite of light direction (light: lightpos -> origo)
        float3 l = normalize(lightpos.xyz);

        //shading
        float3 v = normalize(eyepos.xyz - vertex.pos.xyz);
        float3 h = normalize(v + l);
        // not null vector normal = model vertices
        if (any(vertex.npos.xyz)){
            output.color = BlinnPhong(vertex.pos.xyz, n);
        }
        // null normal -> other vertices
        else{
            output.color = float4(1, 0, 0, 0);
        }

        SpriteStream.Append(output);
    }
    SpriteStream.RestartStrip();
}

//
// PS for drawing particles
//
float4 PSObjectDraw(ObjectVertex input) : SV_Target
{

    // shadow mapping:
    float4 ka = float4(0.1, 0.18725, 0.1745, 1);

    // to homogenous
    input.lpos.xyz /= input.lpos.w;

    // out of bounds: not visible
    if (input.lpos.x < -1.0f || input.lpos.x > 1.0f || input.lpos.y < -1.0f || input.lpos.y > 1.0f || input.lpos.z <  0.0f || input.lpos.z > 1.0f)
        return ka;

    // convert to texture coordinates
    input.lpos.x = input.lpos.x * 0.5f + 0.5;
    input.lpos.y = input.lpos.y * (-0.5f) + 0.5;

    // sample texture
    float depth = g_txShadowMap.Sample(g_samLinear, input.lpos.xy).r;

    // decide if in shadow: shadow map depth is less than fragment depth
    //if (depth < input.lpos.z){
    if (abs(depth - input.lpos.z) > depth_bias){
        return ka;
    }
    else
    {
        return input.color;
    }

}


//--------------------------------------------------------------------------------------
// PICKING
//--------------------------------------------------------------------------------------

//
// Vertex shader for drawing the point-sprite particles (masspoint drawing)
//
PickingVertex VSPickingDraw(uint id : SV_VertexID)
{
    PickingVertex output;

    output.pos = g_bufParticle[id].pos;
    output.mpid1 = g_bufParticle[id].mpid1.xyz;
    output.mpid2 = g_bufParticle[id].mpid2.xyz;

    return output;
}

//
// GS for rendering point sprite particles.  Takes a point and turns it into 2 tris. (masspoint drawing)
//
[maxvertexcount(4)]
void GSPickingDraw(point PickingVertex input[1], inout TriangleStream<PickingVertex> SpriteStream)
{

    PickingVertex output;

    for (int i = 0; i<4; i++){

        // position
        float3 position = g_positions[i] * g_fParticleRad * 2.5f;
        // world position
        position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;
        output.pos = mul(float4(position, 1.0), g_mWorldViewProj);

        // vertex IDs
        output.mpid1 = input[0].mpid1.xyz;
        output.mpid2 = input[0].mpid2.xyz;

        SpriteStream.Append(output);
    }
    SpriteStream.RestartStrip();
}

//
// PS for drawing model (picking)
//
float4 PSPickingDraw1(PickingVertex input) : SV_Target
{
    return float4(input.mpid1, 1.0f);
}

//
// PS for drawing model (picking)
//
float4 PSPickingDraw2(PickingVertex input) : SV_Target
{
    return float4(input.mpid2, 1.0f);
}


//--------------------------------------------------------------------------------------
// SHADOW MAPPING
//--------------------------------------------------------------------------------------

//
// GS for shadow mapping
//
[maxvertexcount(6)]
void GSShadowDraw(uint id : SV_PrimitiveID, point Face input[1], inout TriangleStream<ShadowVertex> SpriteStream)
{
    ShadowVertex output;
    uint points[3] = { input[0].vertices.x, input[0].vertices.y, input[0].vertices.z };

    //table draw
    if (id == 0){
        // first half
        output.pos = mul(g_table[0], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[1], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[2], g_mLightViewProj);
        SpriteStream.Append(output);
        SpriteStream.RestartStrip();
        // second half
        output.pos = mul(g_table[0], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[2], g_mLightViewProj);
        SpriteStream.Append(output);
        output.pos = mul(g_table[3], g_mLightViewProj);
        SpriteStream.Append(output);
    }

    // draw object face
    for (int i = 0; i<3; i++)
    {
        BufferVertex vertex = g_bufParticle[points[2 - i]];
        output.pos = mul(vertex.pos, g_mLightViewProj);
        SpriteStream.Append(output);
    }
    SpriteStream.RestartStrip();
}

//
// PS for shadow mapping
//
void PSShadowDraw(ShadowVertex input){}


//--------------------------------------------------------------------------------------
// MASSPOINT DRAWING
//--------------------------------------------------------------------------------------

//
// VS for drawing masspoints
//
ObjectVertex VSMasspointDraw(uint id : SV_VertexID)
{
    ObjectVertex output;
    output.pos = g_bufMasspoint[id].newpos;
    output.lpos = float4(0, 0, 0, 0);
    output.color = g_bufMasspoint[id].color;
    return output;
}

//
// GS for drawing masspoints
//
[maxvertexcount(6)]
void GSMasspointDraw(point ObjectVertex input[1], inout TriangleStream<ObjectVertex> SpriteStream)
{
    ObjectVertex output;
    output.color = input[0].color;

    // render 2 triangles
    uint i = 0;
    float3 position = g_positions[i] * g_fParticleRad * 2.5f;
    position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;
    output.pos = mul(float4(position, 1.0), g_mWorldViewProj);
    output.lpos = float4(g_texcoords[i].x, g_texcoords[i].y, 0, 0);
    SpriteStream.Append(output);

    i = 2;
    position = g_positions[i] * g_fParticleRad * 2.5f;
    position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;
    output.pos = mul(float4(position, 1.0), g_mWorldViewProj);
    output.lpos = float4(g_texcoords[i].x, g_texcoords[i].y, 0, 0);
    SpriteStream.Append(output);

    i = 1;
    position = g_positions[i] * g_fParticleRad * 2.5f;
    position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;
    output.pos = mul(float4(position, 1.0), g_mWorldViewProj);
    output.lpos = float4(g_texcoords[i].x, g_texcoords[i].y, 0, 0);
    SpriteStream.Append(output);
    SpriteStream.RestartStrip();

    i = 0;
    position = g_positions[i] * g_fParticleRad * 2.5f;
    position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;
    output.pos = mul(float4(position, 1.0), g_mWorldViewProj);
    output.lpos = float4(g_texcoords[i].x, g_texcoords[i].y, 0, 0);
    SpriteStream.Append(output);

    i = 1;
    position = g_positions[i] * g_fParticleRad * 2.5f;
    position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;
    output.pos = mul(float4(position, 1.0), g_mWorldViewProj);
    output.lpos = float4(g_texcoords[i].x, g_texcoords[i].y, 0, 0);
    SpriteStream.Append(output);

    i = 3;
    position = g_positions[i] * g_fParticleRad * 2.5f;
    position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;
    output.pos = mul(float4(position, 1.0), g_mWorldViewProj);
    output.lpos = float4(g_texcoords[i].x, g_texcoords[i].y, 0, 0);
    SpriteStream.Append(output);

    SpriteStream.RestartStrip();
}

//
// PS for drawing masspoints
//
float4 PSMasspointDraw(ObjectVertex input) : SV_Target
{
    float4 res = g_txDiffuse.Sample(g_samLinear, input.lpos.xy);
    return res * input.color;
}
