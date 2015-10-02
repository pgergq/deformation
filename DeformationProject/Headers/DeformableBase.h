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

#include <string>
#include <vector>
#include <array>
#include <DirectXMath.h>
#include "Constants.h"
#include "Collision.h"

using namespace DirectX;


/// Class representing a deformable object model (vertices, masscubes, helper structures)
/// Generic, use inherited classes to import concrete file types
class DeformableBase
{
protected:
    // object ID
    int id;

    // initialize data from file, only available to ctor, redefine in subclasses
    virtual void importFile() = 0;
    // check to see if import was successful
    void checkImport();
    // initialize variables (cube cell size, cube and model position)
    void initVars();
    // initialize particle container
    void initParticles();
    // initialize masscube data
    void initMasscubes();
    // init indexer structure
    void initIndexer();
    // set neighbouring data
    void initNeighbouring();
    // add offset to picking IDs
    void addOffset();
    // initialize collision detection helper structures
    void initCollisionDetection();

public:
    // model .obj file
    std::string file;

    // file import data > model vertices
    vec2float vertices;
    uint vertexCount;
    // file import data > model vertex normals
    vec2float normals;
    uint normalCount;
    // file import data > model faces (index from 1(?...))
    vec2int faces;
    uint faceCount;

    // offset vector added to every volumetric masspoint
    XMFLOAT3 cubePos;
    // cell size of volcube, initial distance between two neighbouring masspoints
    int cubeCellSize;

    // particle (vertex+normal+ID) data
    std::vector<PARTICLE> particles;
    // masscube1 of model
    std::vector<MASSPOINT> masscube1;
    // masscube2 of model
    std::vector<MASSPOINT> masscube2;
    // indexer structure for the model
    std::vector<INDEXER> indexcube;

    // neighbouring data in the first volcube
    std::array<std::array<std::array<uint, VCUBEWIDTH>, VCUBEWIDTH>, VCUBEWIDTH> nvc1;
    // ...second volcube
    std::array<std::array<std::array<uint, VCUBEWIDTH + 1>, VCUBEWIDTH + 1>, VCUBEWIDTH + 1> nvc2;
    // collision detection helper structure for masscube
    BVBoxVector ctree;

    // no default constructor
    DeformableBase() = delete;
    // default destructor
    ~DeformableBase();
    // construct with file name
    DeformableBase(std::string, int);
    // execute initializations
    void build();
    // translate model and masscubes in space
    void translate(int, int, int);

};

#endif