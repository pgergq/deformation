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
#include <atomic>
#include <tuple>
#include <DirectXMath.h>

using namespace DirectX;

/// constant defines for Deformation.cpp and DeformableBase.cpp
#define GET_X_LPARAM(lp)        ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp)        ((int)(short)HIWORD(lp))


/// enable DeformationConsole communication
#define IPCENABLED              0


/// DEFORMATION defines
// n*n*n inner cube, (n+1)*(n+1)*(n+1) outer cube
#define VCUBEWIDTH              13
// particle update CS threadgroup size
#define PARTICLE_TGSIZE         256
// masspoint update CS threadgroup size
#define MASSPOINT_TGSIZE        256

/// volcube neighbouring data
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


/// Some constants
extern std::atomic<float> spreadConstant;
extern std::atomic<float> stiffnessConstant;
extern std::atomic<float> dampingConstant;
extern std::atomic<float> invMassConstant;
extern std::atomic<float> collisionRangeConstant;
extern std::atomic<float> gravityConstant;
extern std::atomic<float> tablePositionConstant;


/// Helper structures
struct VECTOR3
{
    float x;
    float y;
    float z;
    operator float() = delete;
    VECTOR3(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f) : x(_x), y(_y), z(_z) {}
    VECTOR3(const VECTOR3& v) : x(v.x), y(v.y), z(v.z) {}
    VECTOR3 operator+(const VECTOR3& v) { return VECTOR3(x + v.x, y + v.y, z + v.z); }
    VECTOR3 operator-(const VECTOR3& v) { return VECTOR3(x - v.x, y - v.y, z - v.z); }
    VECTOR3 operator*(const VECTOR3& v) { return VECTOR3(x * v.x, y * v.y, z * v.z); }
    VECTOR3 operator*(float s) const { return VECTOR3(s*x, s*y, s*z); }
    VECTOR3 normalized() const { float l = 1.0f / sqrt(x*x + y*y + z*z); return (*this) * l; }
    float dot(const VECTOR3& v) const { return x * v.x + y * v.y + z * v.z; }
    VECTOR3 cross(const VECTOR3& v) const { return VECTOR3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
};

struct VECTOR4
{
    float x;
    float y;
    float z;
    float w;
    VECTOR4(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f, float _w = 0.0f) : x(_x), y(_y), z(_z), w(_w) {}
    VECTOR4(const VECTOR4& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}
    VECTOR4 operator+(const VECTOR4& v) { return VECTOR4(x + v.x, y + v.y, z + v.z, w + v.w); }
    VECTOR4 operator-(const VECTOR4& v) { return VECTOR4(x - v.x, y - v.y, z - v.z, w - v.w); }
    VECTOR4 operator*(const VECTOR4& v) { return VECTOR4(x * v.x, y * v.y, z * v.z, w * v.w); }
};

struct CB_GS
{
    // world-view-projection matrix
    XMFLOAT4X4 worldViewProjection;
    // inverse view matrix
    XMFLOAT4X4 inverseView;
    // light-view-projection matrix
    XMFLOAT4X4 lightViewProjection;
    // current eye position
    XMFLOAT4 eyePos;
    // current light position
    VECTOR4 lightPos;
    // current light colour
    VECTOR4 lightCol;
};

struct CB_CS
{
    // number of masspoint in the smaller volcube in one row
    unsigned int cubeWidth;
    // size of one volcube cell
    unsigned int cubeCellSize;
    // total count of deformable bodies
    unsigned int objectCount;

    // bool for mouse picking
    unsigned int isPicking;
    // pick origin
    unsigned int pickOriginX;
    // pick origin
    unsigned int pickOriginY;

    // stiffness
    float stiffness;
    // damping, negative!
    float damping;
    // delta time
    float dt;
    // inverse mass of masspoints
    float im;
    // F(gravity) = (0, gravity, 0)
    float gravity;
    // table position = (0, tablePos, 0)
    float tablePos;
    // collision effect range constant
    float collisionRange;

    XMFLOAT3 dummy;
    // picking vector direction
    XMFLOAT4 pickDir;
    // eye position
    XMFLOAT4 eyePos;

};

struct MASSPOINT
{
    // previous position of masspoint
    XMFLOAT4 oldpos;
    // current position of masspoint
    XMFLOAT4 newpos;
    // masspoint acceleration
    XMFLOAT4 acc;
    // masspoint colour, debug purposes
    XMFLOAT4 color;
    // neighbour_data mask in the same volcube
    unsigned int neighbour_same;
    // neighbour_data mask in the other volcube
    unsigned int neighbour_other;
    // index in cube
    unsigned int localID;

};

struct PARTICLE
{
    // model vertex position in world space
    XMFLOAT4 pos;
    // model vertex normal's end point (normal = (npos-pos))
    XMFLOAT4 npos;
    // model masscube ID (#1)
    XMFLOAT4 mpid1;
    // model masscube ID (#2)
    XMFLOAT4 mpid2;
};

struct INDEXER
{
    // neighbouring masspoint index in the first volcube
    XMFLOAT3 vc1index;
    // neighbouring masspoint index in the second volcube
    XMFLOAT3 vc2index;
    // masspoint weights of the first volcube neighbours
    float w1[8];
    // masspoint weights of the second volcube neighbours
    float w2[8];
    // normal weights of the first volcube neighbours
    float nw1[8];
    // normal weights of the second volcube neighbours
    float nw2[8];
};

struct FACE
{
    // 3 vertex for object faces
    XMUINT4 vertices;
};

struct BONE
{
    // bone indices
    XMUINT4 boneIndices;
    // bone weights
    XMFLOAT4 boneWeights;

    BONE() : boneIndices(0, 0, 0, 0), boneWeights(0.0f, 0.0f, 0.0f, 0.0f) {}
};

/// Structure representing BVBox data
/// isLeaf -> the box contains 2 masspoints with given bounding box (<-- valid ID(s))
struct BVBOX {

    // ID of left child masspoint
    int leftID;
    // masspoint type of left child (1st or 2nd masscube)
    int leftType;
    // ID of right child masspoint
    int rightID;
    // masspoint type of right child
    int rightType;

    // minX coordinate of the bounding box
    float minX;
    // maxX coordinate of the bounding box
    float maxX;
    // minY coordinate of the bounding box
    float minY;
    // maxY coordinate of the bounding box
    float maxY;
    // minZ coordinate of the bounding box
    float minZ;
    // maxZ coordinate of the bounding box
    float maxZ;

    BVBOX() : leftID(-1), leftType(0), rightID(-1), rightType(0), minX(FLT_MAX), maxX(-FLT_MAX), minY(FLT_MAX), maxY(-FLT_MAX), minZ(FLT_MAX), maxZ(-FLT_MAX) {}
};

/// Structure representing a BVBOX hierarchy on the GPU
struct BVHDESC {
    // BVBOX-tree starting index in the unified BVBOX-buffer
    unsigned int arrayOffset;
    // number of masspoints in this tree
    unsigned int masspointCount;
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
};

/// Typedefs 
typedef unsigned int uint;
typedef std::vector<float> vec1float;
typedef std::vector<int> vec1int;
typedef std::vector<std::vector<float>> vec2float;
typedef std::vector<std::vector<int>> vec2int;
typedef std::vector<MASSPOINT> MassVector;
typedef std::vector<BVBOX> BVBoxVector;
// Masspoint ID (index in masscube), Type (1st or 2nd masscube), Masspoint (data)
typedef std::tuple<int, int, MASSPOINT> MassIDType;
typedef std::vector<MassIDType> MassIDTypeVector;
typedef std::tuple<std::wstring, std::wstring> wstuple;

/// More variables
extern std::atomic<VECTOR4> lightPos;
extern std::atomic<VECTOR4> lightCol;

#endif