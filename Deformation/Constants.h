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

#define GET_X_LPARAM(lp)        ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp)        ((int)(short)HIWORD(lp))


/// DEFORMATION defines
#define VCUBEWIDTH              13          // n*n*n inner cube, (n+1)*(n+1)*(n+1) outer cube

// volcube neighbouring data
#define NB_SAME_LEFT            0x20        // 0010 0000, has left neighbour
#define NB_SAME_RIGHT           0x10        // 0001 0000, has right neighbour
#define NB_SAME_DOWN            0x08        // 0000 1000, ...
#define NB_SAME_UP              0x04        // 0000 0100
#define NB_SAME_FRONT           0x02        // 0000 0010
#define NB_SAME_BACK            0x01        // 0000 0001
// NEAR = lower end of Z axis, nearer to the viewer
// FAR = higher Z values, farther into the screen
#define NB_OTHER_NEAR_BOT_LEFT  0x80        // 1000 0000
#define NB_OTHER_NEAR_BOT_RIGHT 0x40        // 0100 0000
#define NB_OTHER_NEAR_TOP_LEFT  0x20        // 0010 0000
#define NB_OTHER_NEAR_TOP_RIGHT 0x10        // 0001 0000
#define NB_OTHER_FAR_BOT_LEFT   0x08        // 0000 1000
#define NB_OTHER_FAR_BOT_RIGHT  0x04        // 0000 0100
#define NB_OTHER_FAR_TOP_LEFT   0x02        // 0000 0010
#define NB_OTHER_FAR_TOP_RIGHT  0x01        // 0000 0001


// some constants
extern float spreadConstant;
extern float stiffnessConstant;
extern float dampingConstant;
extern float invMassConstant;
extern float collisionRangeConstant;



// Helper structures
struct CB_GS
{
    XMFLOAT4X4 worldViewProjection; // world-view-projection matrix
    XMFLOAT4X4 inverseView;         // inverse view matrix
    XMFLOAT4 eyePos;                // current eye position
    XMFLOAT4 lightPos;              // current light position
};

struct CB_CS
{
    unsigned int cubeWidth;         // number of masspoint in the smaller volcube in one row
    unsigned int cubeCellSize;      // size of one volcube cell
    unsigned int particleCount;     // total count of model vertices

    unsigned int isPicking;         // bool for mouse picking
    unsigned int pickOriginX;       // pick origin
    unsigned int pickOriginY;       // pick origin
    unsigned int dummy1;            // dummy
    unsigned int dummy2;            // dummy

    XMFLOAT4 pickDir;               // picking vector direction
    XMFLOAT4 eyePos;                // eye position

    float stiffness;                // stiffness
    float damping;                  // damping, negative!
    float dt;                       // delta time
    float im;                       // inverse mass of masspoints
};

struct MASSPOINT
{
    XMFLOAT4 oldpos;                // previous position of masspoint
    XMFLOAT4 newpos;                // current position of masspoint
    XMFLOAT4 acc;                   // masspoint acceleration
    unsigned int neighbour_same;    // neighbour_data mask in the same volcube
    unsigned int neighbour_other;   // neighbour_data mask in the other volcube
};

struct PARTICLE
{
    XMFLOAT4 pos;                   // model vertex position in world space
    XMFLOAT4 npos;                  // model vertex normal's end point (normal = (npos-pos))
    XMFLOAT4 mpid1;                 // model masscube ID (#1)
    XMFLOAT4 mpid2;                 // model masscube ID (#2)
};

struct INDEXER
{
    XMFLOAT3 vc1index;              // neighbouring masspoint index in the first volcube
    XMFLOAT3 vc2index;              // neighbouring masspoint index in the second volcube
    float w1[8];                    // masspoint weights of the first volcube neighbours
    float w2[8];                    // masspoint weights of the second volcube neighbours
    float nw1[8];                   // normal weights of the first volcube neighbours
    float nw2[8];                   // normal weights of the second volcube neighbours
};

/// Structure representing BVBox data
/// isLeaf -> the box contains 2 masspoints with given bounding box (<-- valid ID(s))
struct BVBOX {

    int leftID;                     // ID of left child masspoint
    int rightID;                    // ID of right child masspoint

    float minX;                     // minX coordinate of the bounding box
    float maxX;                     // maxX coordinate of the bounding box
    float minY;                     // minY coordinate of the bounding box
    float maxY;                     // maxY coordinate of the bounding box
    float minZ;                     // minZ coordinate of the bounding box
    float maxZ;                     // maxZ coordinate of the bounding box

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