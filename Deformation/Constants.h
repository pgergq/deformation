//--------------------------------------------------------------------------------------
// File: Constants.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Deformation constant definitions
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <cfloat>
#include <vector>
#include <tuple>
#include <DirectXMath.h>


using namespace DirectX;

// constant defines for Deformation.cpp and Deformable.cpp

#define GET_X_LPARAM(lp)		((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp)		((int)(short)HIWORD(lp))


/// DEFORMATION defines
#define VCUBEWIDTH		13											// n*n*n inner cube, (n+1)*(n+1)*(n+1) outer cube

// volcube neighbouring data
#define NB_SAME_LEFT			0x20								// 0010 0000, has left neighbour
#define NB_SAME_RIGHT			0x10								// 0001 0000, has right neighbour
#define NB_SAME_DOWN			0x08								// 0000 1000, ...
#define NB_SAME_UP				0x04								// 0000 0100
#define NB_SAME_FRONT			0x02								// 0000 0010
#define NB_SAME_BACK			0x01								// 0000 0001
// NEAR = lower end of Z axis, nearer to the viewer
// FAR = higher Z values, farther into the screen
#define NB_OTHER_NEAR_BOT_LEFT	0x80								// 1000 0000
#define NB_OTHER_NEAR_BOT_RIGHT	0x40								// 0100 0000
#define NB_OTHER_NEAR_TOP_LEFT	0x20								// 0010 0000
#define NB_OTHER_NEAR_TOP_RIGHT	0x10								// 0001 0000
#define NB_OTHER_FAR_BOT_LEFT	0x08								// 0000 1000
#define NB_OTHER_FAR_BOT_RIGHT	0x04								// 0000 0100
#define NB_OTHER_FAR_TOP_LEFT	0x02								// 0000 0010
#define NB_OTHER_FAR_TOP_RIGHT	0x01								// 0000 0001


// some constants
extern float g_fSpread;
extern float g_fStiffness;
extern float g_fDamping;
extern float g_fInvMass;
extern float g_fCollisionRange;



// Helper structures
struct MASSPOINT
{
    XMFLOAT4 oldpos;		// previous position of masspoint
    XMFLOAT4 newpos;		// current position of masspoint
    XMFLOAT4 acc;			// masspoint acceleration
    unsigned int neighbour_same;	// neighbour_data mask in the same volcube
    unsigned int neighbour_other;	// neighbour_data mask in the other volcube
};

struct PARTICLE
{
    XMFLOAT4 pos;			// model vertex position in world space
    XMFLOAT4 npos;			// model vertex normal's end point (normal = (npos-pos))
    XMFLOAT4 mpid1;			// model masscube ID (#1)
    XMFLOAT4 mpid2;			// model masscube ID (#2)
};

struct INDEXER
{
    XMFLOAT3 vc1index;
    XMFLOAT3 vc2index;
    float w1[8];
    float w2[8];
    float nw1[8];
    float nw2[8];
};

/// Structure representing BVBox data
/// isLeaf -> the box contains 2 masspoints with given bounding box (<-- valid ID(s))
struct BVBOX {

    int leftID;            // ID of left child masspoint
    int rightID;           // ID of right child masspoint

    float minX;             // coordinates
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;

    BVBOX() : leftID(-1), rightID(-1), minX(FLT_MAX), maxX(FLT_MIN), minY(FLT_MAX), maxY(FLT_MIN), minZ(FLT_MAX), maxZ(FLT_MIN) {}
};

// typedefs 
typedef unsigned int uint;
typedef std::vector<float> vec1float;
typedef std::vector<int> vec1int;
typedef std::vector<std::vector<float>> vec2float;
typedef std::vector<std::vector<int>> vec2int;
typedef std::vector<MASSPOINT> MassVector;
typedef std::vector<BVBOX> BVBoxVector;
typedef std::tuple<int, MASSPOINT> MassID;
typedef std::vector<MassID> MassIDVector;


#endif