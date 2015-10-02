//--------------------------------------------------------------------------------------
// File: DeformableFBX.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Deformable class definition
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "DXUT.h"
#include "../Headers/DeformableFBX.h"
#include "../Headers/Constants.h"
#include "../Headers/Collision.h"



//--------------------------------------------------------------------------------------
// Init (1) Deformable model data: import vertices|normals|faces from file
//--------------------------------------------------------------------------------------
void DeformableFBX::importFile(){
    
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(this->file, aiProcess_GenNormals |
            aiProcess_LimitBoneWeights |
            aiProcess_ConvertToLeftHanded |
            aiProcess_JoinIdenticalVertices |
            aiProcess_Triangulate);

    // error handling
    if (!scene)
    {
        throw importer.GetErrorString();
    }
    if ((!scene->mMeshes[0]->HasNormals()) || (!scene->mMeshes[0]->HasFaces()))
    {
        throw "The imported scene does not have normals and/or faces.";
    }

    // successful import, process data
    else
    {
        // fill vertex array
        for (uint i = 0; i < scene->mMeshes[0]->mNumVertices; i++)
        {
            vec1float v;
            v.push_back(-1.0f);
            v.push_back(scene->mMeshes[0]->mVertices[i].x);
            v.push_back(scene->mMeshes[0]->mVertices[i].y);
            v.push_back(scene->mMeshes[0]->mVertices[i].z);
            this->vertices.push_back(v);
        }
        // fill normals array
        for (uint i = 0; i < scene->mMeshes[0]->mNumVertices; i++)
        {
            vec1float n;
            n.push_back(-1.0f);
            n.push_back(scene->mMeshes[0]->mNormals[i].x);
            n.push_back(scene->mMeshes[0]->mNormals[i].y);
            n.push_back(scene->mMeshes[0]->mNormals[i].z);
            this->normals.push_back(n);
        }
        // fill faces array
        for (uint i = 0; i < scene->mMeshes[0]->mNumFaces; i++)
        {
            vec1int f;
            f.push_back(-1);
            f.push_back(scene->mMeshes[0]->mFaces[i].mIndices[0]);
            f.push_back(scene->mMeshes[0]->mFaces[i].mIndices[1]);
            f.push_back(scene->mMeshes[0]->mFaces[i].mIndices[2]);
            this->faces.push_back(f);
        }

    }
}
