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



class Deformable
{
private:
	// variables


	// functions
	void init();				// initialize data from file, only available to ctor


public:
	// variables
	std::string file;			// model .obj file

	int index;					// model index (created during file loading)

	float vertexCount;			// 
	vec2float vertices;			// file import data > model vertices
	float normalCount;			// 
	vec2float normals;			// file import data > model vertex normals
	float faceCount;			// 
	vec2int faces;				// file import data > model faces (index from 1)

	XMFLOAT3 cubePos;			// offset vector added to every volumetric masspoint
	XMFLOAT3 modelPos;			// offset vector added to every model vertex

	int cubeCellSize;			// cell size of volcube, initial distance between two neighbouring masspoints
	
	// functions
	Deformable() = delete;		// no default constructor
	~Deformable();				// default destructor
	Deformable(int, std::string);	// construct with file name
	Deformable(const Deformable&) = delete;	// no copy constructor



};










#endif