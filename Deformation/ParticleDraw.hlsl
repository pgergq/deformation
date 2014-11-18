//--------------------------------------------------------------------------------------
// File: ParticleDraw.hlsl
//
// Shaders for rendering the particle as point sprite (particle -> 2 triangles)
// (green particles highlighted)
//
// @(Copyright (c) Microsoft Corporation. All rights reserved. - NBodyGravityCS11 project)
// @Modified by pgq
//--------------------------------------------------------------------------------------


struct VSParticle
{
	float4 pos			: WPOS;
	float4 npos			: NPOS;
	float4 mpid1		: MPID1;
	float4 mpid2		: MPID2;
};

struct ShadowPos
{
    float4 pos          : SV_Position;
};

struct GSParticleDrawOut
{
	float2 tex			: TEXCOORD0;
	float3 mpid1		: MPID1;
	float3 mpid2		: MPID2;
	float4 color		: COLOR;
    float4 shpos        : SHPOS;
	float4 pos			: SV_Position;
};

struct PSParticleDrawIn
{
	float2 tex			: TEXCOORD0;
	float3 mpid1		: MPID1;
	float3 mpid2		: MPID2;
	float4 color		: COLOR;
    float4 shpos        : SHPOS;
};

Texture2D g_txDiffuse;
StructuredBuffer<VSParticle> g_bufParticle;


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
		float3(1, 1, 0),
		float3(-1, -1, 0),
		float3(1, -1, 0),
	};

	static float2 g_texcoords[4] =
	{
		float2(0, 0),
		float2(1, 0),
		float2(0, 1),
		float2(1, 1),
	};
};

//
// Phong-Blinn, ruby
//
float4 PhongBlinn(float3 n, float3 h, float3 l){
	float4 ka = float4(0.1745, 0.01175, 0.01175, 0.55);
	float4 kd = float4(0.61424, 0.04136, 0.04136, 0.55);
	float4 ks = float4(0.727811, 0.626959, 0.626959, 0.55);
	float shininess = 76.8;
	return (ka + kd * saturate(dot(n, l)) + ks * pow(saturate(dot(n, h)), shininess)) * lightcol;
}

//
// Vertex shader for drawing the point-sprite particles
//
VSParticle VSParticleDraw(uint id : SV_VertexID)
{
	VSParticle output;

	output.pos = g_bufParticle[id].pos;
	output.npos = g_bufParticle[id].npos;
	output.mpid1 = g_bufParticle[id].mpid1;
	output.mpid2 = g_bufParticle[id].mpid2;

	return output;
}

//
// GS for rendering point sprite particles.  Takes a point and turns it into 2 tris.
//
[maxvertexcount(4)]
void GSParticleDraw(point VSParticle input[1], inout TriangleStream<GSParticleDrawOut> SpriteStream)
{

	GSParticleDrawOut output;

	//calculate lighting data
	float3 n = normalize(input[0].npos.xyz - input[0].pos.xyz);		// vertex normal
	float3 l = normalize(lightpos.xyz);								// opposite of light direction (light: lightpos -> origo)

	for (int i = 0; i<4; i++)
	{

		//vertex -> two triangles
		float3 position = g_positions[i] * g_fParticleRad * 2.5f;
		position = mul(position, (float3x3)g_mInvView) + input[0].pos.xyz;	// world position
		output.pos = mul(float4(position, 1.0), g_mWorldViewProj);
		
		//shading
		float3 v = normalize(eyepos.xyz - input[0].pos.xyz);
		float3 h = normalize(v + l);
		if (any(input[0].npos.xyz)){								// not null vector normal = model vertices
			output.color = PhongBlinn(n, h, l);
		}
		else{														// null normal -> other vertices
			output.color = float4(1,0,0,0);
		}

		//texture and vertex IDs
		output.tex = g_texcoords[i];
		output.mpid1 = input[0].mpid1.xyz;
		output.mpid2 = input[0].mpid2.xyz;
        //shadow data
        output.shpos = mul(float4(position, 1.0), g_mLightViewProj);
		SpriteStream.Append(output);
	}
	SpriteStream.RestartStrip();
}

//
// PS for drawing particles
//
float4 PSParticleDraw(PSParticleDrawIn input) : SV_Target
{
	return g_txDiffuse.Sample(g_samLinear, input.tex) * input.color;
}

//
// PS for drawing model (picking)
//
float4 PSModelDraw1(PSParticleDrawIn input) : SV_Target
{
	return float4(input.mpid1, 1.0f);
}

//
// PS for drawing model (picking)
//
float4 PSModelDraw2(PSParticleDrawIn input) : SV_Target
{
	return float4(input.mpid2, 1.0f);
}

////////// Shadow mapping

//
// Shadow VS
//
ShadowPos VSShadow(uint id : SV_VertexID)
{
    ShadowPos output;
    output.pos = mul(g_bufParticle[id].pos, g_mLightViewProj);
    return output;
}