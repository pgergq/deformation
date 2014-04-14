//--------------------------------------------------------------------------------------
// File: Deformable.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Deformable class declaration and helper typedefs
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _DEFORMABLE_H_
#define _DEFORMABLE_H_

// includes
#include <string>
#include <vector>
#include <DirectXMath.h>


// namespace
using namespace DirectX;

// typedefs for easier handling, also used in Deformation.cpp
typedef std::vector<float> vec1float;
typedef std::vector<int> vec1int;
typedef std::vector<std::vector<float>> vec2float;
typedef std::vector<std::vector<int>> vec2int;


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


class Deformable final
{
private:
	// variables

	// functions
	void importFile();			// initialize data from file, only available to ctor
	void initVars();			// initialize variables (cube cell size, cube and model position)
	void initParticles();		// initialize particle container
	void initMasscubes();		// initialize masscube data
	void initIndexer();			// init indexer structure
	void initNeighbouring();	// set neighbouring data

public:
	// variables
	std::string file;			// model .obj file

	float vertexCount;			// 
	vec2float vertices;			// file import data > model vertices
	float normalCount;			// 
	vec2float normals;			// file import data > model vertex normals
	float faceCount;			// 
	vec2int faces;				// file import data > model faces (index from 1)

	XMFLOAT3 cubePos;			// offset vector added to every volumetric masspoint
	XMFLOAT3 modelPos;			// offset vector added to every model vertex
	int cubeCellSize;			// cell size of volcube, initial distance between two neighbouring masspoints

	std::vector<PARTICLE> particles;	// particle (vertex+normal+ID) data
	std::vector<MASSPOINT> masscube1;	// masscube1 of model
	std::vector<MASSPOINT> masscube2;	// masscube2 of model
	std::vector<INDEXER> indexcube;		// indexer structure for the model
	
	// functions
	Deformable() = delete;		// no default constructor
	~Deformable();				// default destructor
	Deformable(std::string);	// construct with file name
	//Deformable(const Deformable&) = delete;	// no copy constructor ***
	void build();				// execute initializations
	
};

#endif