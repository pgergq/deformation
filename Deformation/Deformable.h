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
#include <array>
#include <DirectXMath.h>
#include "Constants.h"
#include "Collision.h"


// namespace
using namespace DirectX;


/// Class representing a deformable object model (vertices, masscubes, helper structures)
class Deformable final
{
private:
    // variables
    int id;
    // functions
    void importFile();			// initialize data from file, only available to ctor
    void initVars();			// initialize variables (cube cell size, cube and model position)
    void initParticles();		// initialize particle container
    void initMasscubes();		// initialize masscube data
    void initIndexer();			// init indexer structure
    void initNeighbouring();	// set neighbouring data
    void addOffset();           // add offset to picking IDs

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
    int cubeCellSize;			// cell size of volcube, initial distance between two neighbouring masspoints

    std::vector<PARTICLE> particles;	// particle (vertex+normal+ID) data
    std::vector<MASSPOINT> masscube1;	// masscube1 of model
    std::vector<MASSPOINT> masscube2;	// masscube2 of model
    std::vector<INDEXER> indexcube;		// indexer structure for the model

    std::array<std::array<std::array<uint, VCUBEWIDTH>, VCUBEWIDTH>, VCUBEWIDTH> nvc1;   // neighbouring data in the first volcube
    std::array<std::array<std::array<uint, VCUBEWIDTH + 1>, VCUBEWIDTH + 1>, VCUBEWIDTH + 1> nvc2;   // ...second volcube
    BVBoxVector ctree1;         // collision detection helper structure for 1st masscube
    BVBoxVector ctree2;         // ... 2nd masscube

    // functions
    Deformable() = delete;		// no default constructor
    ~Deformable();				// default destructor
    Deformable(std::string, int);// construct with file name
    void build();				// execute initializations
    void translate(int, int, int);// translate model and masscubes in space
    void initCollisionDetection();  // initialize collision detection helper structures
    
};

#endif