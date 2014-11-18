//--------------------------------------------------------------------------------------
// File: Deformable.cpp
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
#include "Deformable.h"
#include "Constants.h"
#include "Collision.h"


//--------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------
Deformable::Deformable(std::string x, int id){

    this->file = x;		// file must be .obj formatted, with equal number of vertices and vertex normals (plus optional face data)
    this->id = id;      // object ID in the applications object-container (determines offset in buffers)
}

//--------------------------------------------------------------------------------------
// Init (A) Initialize all components
//--------------------------------------------------------------------------------------
void Deformable::build(){

    this->importFile();
    this->initVars();
    this->initParticles();
    this->initMasscubes();
    this->initIndexer();
    this->initNeighbouring();
    this->addOffset();

}

//--------------------------------------------------------------------------------------
// Init (1) Deformable model data: import vertices|normals|faces from file
//--------------------------------------------------------------------------------------
void Deformable::importFile(){

    // Open file
    std::ifstream input;
    input.open(this->file);

    // Process file
    while (!input.eof()){
        // get line
        std::string line;
        getline(input, line);

        // split at spaces
        vec1float v;
        vec1float n;
        vec1int f;
        std::istringstream buf(line);
        std::istream_iterator<std::string> start(buf), end;
        std::vector<std::string> parts(start, end);

        // store vertex
        if (line[0] == 'v' && line[1] != 'n'){
            // parse
            for (auto& s : parts){
                v.push_back((float)atof(s.c_str()));
            }
            //store
            this->vertices.push_back(v);
        }
        // store normal
        else if (line[0] == 'v' && line[1] == 'n'){
            // parse
            for (auto& s : parts){
                n.push_back((float)atof(s.c_str()));
            }
            //store
            this->normals.push_back(n);
        }
        // store face
        else if (line[0] == 'f'){
            // parse
            for (auto& s : parts){
                f.push_back(atoi(s.substr(0,1).c_str()));
            }
            //store
            this->faces.push_back(f);
        }
    }

    // Close file
    input.close();
}

//--------------------------------------------------------------------------------------
// Init (2) Deformable model data: initialize cube cell size, cube pos and model pos
//--------------------------------------------------------------------------------------
void Deformable::initVars(){

    // get/set space division parameters
    float minx, miny, minz, maxx, maxy, maxz;
    maxx = minx = this->vertices[0][1];
    maxy = miny = this->vertices[0][2];
    maxz = minz = this->vertices[0][3];
    for (unsigned int i = 0; i < this->vertices.size(); i++){
        if (this->vertices[i][1] < minx)
            minx = this->vertices[i][1];
        if (this->vertices[i][1] > maxx)
            maxx = this->vertices[i][1];
        if (this->vertices[i][2] < miny)
            miny = this->vertices[i][2];
        if (this->vertices[i][2] > maxy)
            maxy = this->vertices[i][2];
        if (this->vertices[i][3] < minz)
            minz = this->vertices[i][3];
        if (this->vertices[i][3] > maxz)
            maxz = this->vertices[i][3];
    }

    // tmp = greatest length in any direction (x|y|z)
    float tmp = std::max(abs(ceil(maxx) - floor(minx)), std::max(abs(ceil(maxy) - floor(miny)), abs(ceil(maxz) - floor(minz)))) + 1;
    // ceil((float)tmp/VCUBEWIDTH) = "tight" value of VOLCUBECELL
    // ceil((float)(tight+50)/100)*100 = upper 100 neighbour of tight
    tmp = ceil((float)tmp / VCUBEWIDTH);
    tmp = ceil((float)(tmp + 50) / 100) * 100;
    this->cubeCellSize = (int)tmp;

    // Set global volumetric cube offsets to align the model
    this->cubePos = XMFLOAT3(floor(minx), floor(miny), floor(minz));

    // Set data
    this->vertexCount = this->vertices.size();
    this->normalCount = this->normals.size();
    this->faceCount = this->faces.size();

    // Assert
    if (this->vertexCount != this->normalCount){
        throw "[ERROR]: vertexCount and normalCount are not equal!" ;
    }
}

//--------------------------------------------------------------------------------------
// Init (3) Deformable model data: initialize particle container (position+normal)
//--------------------------------------------------------------------------------------
void Deformable::initParticles(){

    // Load model vertices + normals
    for (int i = 0; i < this->vertexCount; i++)
    {
        PARTICLE push { XMFLOAT4(0, 0, 0, 1), XMFLOAT4(0, 0, 0, 1), XMFLOAT4(0, 0, 0, 0), XMFLOAT4(0, 0, 0, 0) };
        
        // position
        XMVECTOR tmp = XMVectorSet(this->vertices[i][1], this->vertices[i][2], this->vertices[i][3], 1);
        XMStoreFloat4(&push.pos, tmp);

        // normalized normals -> store the endpoint of the normals (=npos)
        float len = this->normals[i][1] * this->normals[i][1] + this->normals[i][2] * this->normals[i][2] + this->normals[i][3] * this->normals[i][3];
        len = (len == 0 ? -1 : sqrtf(len));
        XMVECTOR tmp2 = XMVectorSet((float)this->normals[i][1] / len, (float)this->normals[i][2] / len, (float)this->normals[i][3] / len, 0);
        XMStoreFloat4(&push.npos, XMVectorAdd(tmp, XMVector3Normalize(tmp2)));

        // store temporary vector in container
        particles.push_back(push);
    }
}

//--------------------------------------------------------------------------------------
// Init (4) Deformable model data: initialize masscube containers (pos, acc)
//--------------------------------------------------------------------------------------
void Deformable::initMasscubes(){

    int ind;
    MASSPOINT push;

    // Load first volumetric cube
    for (int i = 0; i < VCUBEWIDTH; i++)
    {
        for (int j = 0; j < VCUBEWIDTH; j++)
        {
            for (int k = 0; k < VCUBEWIDTH; k++)
            {
                ind = i*VCUBEWIDTH*VCUBEWIDTH + j*VCUBEWIDTH + k;
                XMVECTOR tmp = XMVectorAdd(XMVectorSet(k * this->cubeCellSize, j * this->cubeCellSize, i * this->cubeCellSize, 1), 
                    XMVectorSet(this->cubePos.x, this->cubePos.y, this->cubePos.z, 0));
                XMStoreFloat4(&push.newpos, tmp);
                XMStoreFloat4(&push.oldpos, tmp);
                push.acc = XMFLOAT4(0, 0, 0, 0);
                this->masscube1.push_back(push);
            }
        }
    }

    // Load second volumetric cube
    for (int i = 0; i < VCUBEWIDTH + 1; i++)
    {
        for (int j = 0; j < VCUBEWIDTH + 1; j++)
        {
            for (int k = 0; k < VCUBEWIDTH + 1; k++)
            {
                ind = i*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + j*(VCUBEWIDTH + 1) + k;
                XMVECTOR tmp = XMVectorAdd(XMVectorSet(k * this->cubeCellSize - 0.5f * this->cubeCellSize, j * this->cubeCellSize - 0.5f * this->cubeCellSize, i * this->cubeCellSize - 0.5f * this->cubeCellSize, 1), 
                    XMVectorSet(this->cubePos.x, this->cubePos.y, this->cubePos.z, 0));
                XMStoreFloat4(&push.oldpos, tmp);
                XMStoreFloat4(&push.newpos, tmp);
                push.acc = XMFLOAT4(0, 0, 0, 0);
                this->masscube2.push_back(push);
            }
        }
    }
}

//--------------------------------------------------------------------------------------
// Init (5) Deformable model data: initialize indexer container (IDs, weights)
//--------------------------------------------------------------------------------------
void Deformable::initIndexer(){

    // Load indexer cube
    XMFLOAT3 vc_pos1(this->masscube1[0].oldpos.x, this->masscube1[0].oldpos.y, this->masscube1[0].oldpos.z);
    XMFLOAT3 vc_pos2(this->masscube2[0].oldpos.x, this->masscube2[0].oldpos.y, this->masscube2[0].oldpos.z);
    XMFLOAT3 vertex;
    int vind;

    for (int i = 0; i < this->vertexCount; i++)
    {
        INDEXER push;
        
        // Get indices and weights in first volumetric cube
        vertex = XMFLOAT3(this->particles[i].pos.x, this->particles[i].pos.y, this->particles[i].pos.z);
        int x = (std::max(vertex.x, vc_pos1.x) - std::min(vertex.x, vc_pos1.x)) / this->cubeCellSize;
        int y = (std::max(vertex.y, vc_pos1.y) - std::min(vertex.y, vc_pos1.y)) / this->cubeCellSize;
        int z = (std::max(vertex.z, vc_pos1.z) - std::min(vertex.z, vc_pos1.z)) / this->cubeCellSize;
        XMStoreFloat3(&push.vc1index, XMVectorSet(x, y, z, 0));
        XMStoreFloat4(&this->particles[i].mpid1, XMVectorSet(x, y, z, 1));

        // trilinear interpolation
        vind = z*VCUBEWIDTH*VCUBEWIDTH + y*VCUBEWIDTH + x;
        float wx = (vertex.x - this->masscube1[vind].newpos.x) / this->cubeCellSize;
        float dwx = 1.0f - wx;
        float wy = (vertex.y - this->masscube1[vind].newpos.y) / this->cubeCellSize;
        float dwy = 1.0f - wy;
        float wz = (vertex.z - this->masscube1[vind].newpos.z) / this->cubeCellSize;
        float dwz = 1.0f - wz;
        push.w1[0] = dwx*dwy*dwz; push.w1[1] = wx*dwy*dwz; push.w1[2] = dwx*wy*dwz; push.w1[3] = wx*wy*dwz;
        push.w1[4] = dwx*dwy*wz; push.w1[5] = wx*dwy*wz; push.w1[6] = dwx*wy*wz; push.w1[7] = wx*wy*wz;

        // trilinear for npos
        vertex = XMFLOAT3(this->particles[i].npos.x, this->particles[i].npos.y, this->particles[i].npos.z);
        wx = (vertex.x - this->masscube1[vind].newpos.x) / this->cubeCellSize;
        dwx = 1.0f - wx;
        wy = (vertex.y - this->masscube1[vind].newpos.y) / this->cubeCellSize;
        dwy = 1.0f - wy;
        wz = (vertex.z - this->masscube1[vind].newpos.z) / this->cubeCellSize;
        dwz = 1.0f - wz;
        push.nw1[0] = dwx*dwy*dwz; push.nw1[1] = wx*dwy*dwz; push.nw1[2] = dwx*wy*dwz; push.nw1[3] = wx*wy*dwz;
        push.nw1[4] = dwx*dwy*wz; push.nw1[5] = wx*dwy*wz; push.nw1[6] = dwx*wy*wz; push.nw1[7] = wx*wy*wz;


        // Fill second indexer
        vertex = XMFLOAT3(this->particles[i].pos.x, this->particles[i].pos.y, this->particles[i].pos.z);
        x = (std::max(vertex.x, vc_pos2.x) - std::min(vertex.x, vc_pos2.x)) / this->cubeCellSize;
        y = (std::max(vertex.y, vc_pos2.y) - std::min(vertex.y, vc_pos2.y)) / this->cubeCellSize;
        z = (std::max(vertex.z, vc_pos2.z) - std::min(vertex.z, vc_pos2.z)) / this->cubeCellSize;
        XMStoreFloat3(&push.vc2index, XMVectorSet(x, y, z, 0));
        XMStoreFloat4(&this->particles[i].mpid2, XMVectorSet(x, y, z, 1));

        vind = z*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + y*(VCUBEWIDTH + 1) + x;
        wx = (vertex.x - this->masscube2[vind].newpos.x) / this->cubeCellSize;
        dwx = 1.0f - wx;
        wy = (vertex.y - this->masscube2[vind].newpos.y) / this->cubeCellSize;
        dwy = 1.0f - wy;
        wz = (vertex.z - this->masscube2[vind].newpos.z) / this->cubeCellSize;
        dwz = 1.0f - wz;
        push.w2[0] = dwx*dwy*dwz; push.w2[1] = wx*dwy*dwz; push.w2[2] = dwx*wy*dwz; push.w2[3] = wx*wy*dwz;
        push.w2[4] = dwx*dwy*wz; push.w2[5] = wx*dwy*wz; push.w2[6] = dwx*wy*wz; push.w2[7] = wx*wy*wz;

        vertex = XMFLOAT3(this->particles[i].npos.x, this->particles[i].npos.y, this->particles[i].npos.z);
        wx = (vertex.x - this->masscube2[vind].newpos.x) / this->cubeCellSize;
        dwx = 1.0f - wx;
        wy = (vertex.y - this->masscube2[vind].newpos.y) / this->cubeCellSize;
        dwy = 1.0f - wy;
        wz = (vertex.z - this->masscube2[vind].newpos.z) / this->cubeCellSize;
        dwz = 1.0f - wz;
        push.nw2[0] = dwx*dwy*dwz; push.nw2[1] = wx*dwy*dwz; push.nw2[2] = dwx*wy*dwz; push.nw2[3] = wx*wy*dwz;
        push.nw2[4] = dwx*dwy*wz; push.nw2[5] = wx*dwy*wz; push.nw2[6] = dwx*wy*wz; push.nw2[7] = wx*wy*wz;

        indexcube.push_back(push);
    }
}

//--------------------------------------------------------------------------------------
// Init (6) Deformable model data: initialize neighbouring on masspoints
//--------------------------------------------------------------------------------------
void Deformable::initNeighbouring(){

    /// Set proper neighbouring data (disable masspoints with no model points)
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int y = 0; y < VCUBEWIDTH; y++){
            for (int x = 0; x < VCUBEWIDTH; x++){
                nvc1[z][y][x] = 0;
            }
        }
    }
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int y = 0; y < VCUBEWIDTH + 1; y++){
            for (int x = 0; x < VCUBEWIDTH + 1; x++){
                nvc2[z][y][x] = 0;
            }
        }
    }
    XMFLOAT3 vx;
    // Set edge masspoints to 1
    for (int i = 0; i < this->vertexCount; i++)
    {
        vx = this->indexcube[i].vc1index;
        nvc1[(int)vx.z][(int)vx.y][(int)vx.x] = 1;
        nvc1[(int)vx.z][(int)vx.y][(int)vx.x + 1] = 1;
        nvc1[(int)vx.z][(int)vx.y + 1][(int)vx.x] = 1;
        nvc1[(int)vx.z][(int)vx.y + 1][(int)vx.x + 1] = 1;
        nvc1[(int)vx.z + 1][(int)vx.y][(int)vx.x] = 1;
        nvc1[(int)vx.z + 1][(int)vx.y][(int)vx.x + 1] = 1;
        nvc1[(int)vx.z + 1][(int)vx.y + 1][(int)vx.x] = 1;
        nvc1[(int)vx.z + 1][(int)vx.y + 1][(int)vx.x + 1] = 1;
        vx = this->indexcube[i].vc2index;
        nvc2[(int)vx.z][(int)vx.y][(int)vx.x] = 1;
        nvc2[(int)vx.z][(int)vx.y][(int)vx.x + 1] = 1;
        nvc2[(int)vx.z][(int)vx.y + 1][(int)vx.x] = 1;
        nvc2[(int)vx.z][(int)vx.y + 1][(int)vx.x + 1] = 1;
        nvc2[(int)vx.z + 1][(int)vx.y][(int)vx.x] = 1;
        nvc2[(int)vx.z + 1][(int)vx.y][(int)vx.x + 1] = 1;
        nvc2[(int)vx.z + 1][(int)vx.y + 1][(int)vx.x] = 1;
        nvc2[(int)vx.z + 1][(int)vx.y + 1][(int)vx.x + 1] = 1;
    }

    // Set outer masspoints to 2 - 1st volcube
    //left+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int y = 0; y < VCUBEWIDTH; y++){
            for (int x = 0; x < VCUBEWIDTH; x++){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //right+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int y = 0; y < VCUBEWIDTH; y++){
            for (int x = VCUBEWIDTH - 1; x >= 0; x--){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //bottom+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int y = 0; y < VCUBEWIDTH; y++){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //up+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int y = VCUBEWIDTH - 1; y >= 0; y--){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //front+
    for (int y = 0; y < VCUBEWIDTH; y++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int z = 0; z < VCUBEWIDTH; z++){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //back+
    for (int y = 0; y < VCUBEWIDTH; y++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int z = VCUBEWIDTH - 1; z >= 0; z--){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //Set outer masspoints to 2 - 2nd volcube
    //left+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int y = 0; y < VCUBEWIDTH + 1; y++){
            for (int x = 0; x < VCUBEWIDTH + 1; x++){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //right+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int y = 0; y < VCUBEWIDTH + 1; y++){
            for (int x = VCUBEWIDTH; x >= 0; x--){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //bottom+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int y = 0; y < VCUBEWIDTH + 1; y++){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //up+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int y = VCUBEWIDTH; y >= 0; y--){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //front+
    for (int y = 0; y < VCUBEWIDTH + 1; y++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int z = 0; z < VCUBEWIDTH + 1; z++){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //back+
    for (int y = 0; y < VCUBEWIDTH + 1; y++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int z = VCUBEWIDTH; z >= 0; z--){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }

    // Correct neighbouring in 1st volcube
    unsigned int ind, mask;
    for (int i = 0; i < VCUBEWIDTH; i++)
    {
        for (int j = 0; j < VCUBEWIDTH; j++)
        {
            for (int k = 0; k < VCUBEWIDTH; k++)
            {
                ind = i*VCUBEWIDTH*VCUBEWIDTH + j*VCUBEWIDTH + k;

                // same volcube bitmasks
                mask = 0x00;
                mask = k != 0 && nvc1[i][j][k - 1] != 2 ? mask | NB_SAME_LEFT : mask;
                mask = k != (VCUBEWIDTH - 1) && nvc1[i][j][k + 1] != 2 ? mask | NB_SAME_RIGHT : mask;
                mask = j != 0 && nvc1[i][j - 1][k] != 2 ? mask | NB_SAME_DOWN : mask;
                mask = j != (VCUBEWIDTH - 1) && nvc1[i][j + 1][k] != 2 ? mask | NB_SAME_UP : mask;
                mask = i != 0 && nvc1[i - 1][j][k] != 2 ? mask | NB_SAME_FRONT : mask;
                mask = i != (VCUBEWIDTH - 1) && nvc1[i + 1][j][k] != 2 ? mask | NB_SAME_BACK : mask;
                this->masscube1[ind].neighbour_same = mask;

                // other volcube bitmasks
                mask = 0x00;
                mask = nvc2[i][j][k] != 2 ? mask | NB_OTHER_NEAR_BOT_LEFT : mask;
                mask = nvc2[i][j][k + 1] != 2 ? mask | NB_OTHER_NEAR_BOT_RIGHT : mask;
                mask = nvc2[i][j + 1][k] != 2 ? mask | NB_OTHER_NEAR_TOP_LEFT : mask;
                mask = nvc2[i][j + 1][k + 1] != 2 ? mask | NB_OTHER_NEAR_TOP_RIGHT : mask;
                mask = nvc2[i + 1][j][k] != 2 ? mask | NB_OTHER_FAR_BOT_LEFT : mask;
                mask = nvc2[i + 1][j][k + 1] != 2 ? mask | NB_OTHER_FAR_BOT_RIGHT : mask;
                mask = nvc2[i + 1][j + 1][k] != 2 ? mask | NB_OTHER_FAR_TOP_LEFT : mask;
                mask = nvc2[i + 1][j + 1][k + 1] != 2 ? mask | NB_OTHER_FAR_TOP_RIGHT : mask;
                this->masscube1[ind].neighbour_other = mask;
            }
        }
    }

    // Correct neighbouring in 2nd volcube
    for (int i = 0; i < VCUBEWIDTH + 1; i++)
    {
        for (int j = 0; j < VCUBEWIDTH + 1; j++)
        {
            for (int k = 0; k < VCUBEWIDTH + 1; k++)
            {
                ind = i*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + j*(VCUBEWIDTH + 1) + k;

                // same volcube bitmasks
                mask = 0x00;
                mask = k != 0 && nvc2[i][j][k - 1] != 2 ? mask | NB_SAME_LEFT : mask;
                mask = k != (VCUBEWIDTH) && nvc2[i][j][k + 1] != 2 ? mask | NB_SAME_RIGHT : mask;
                mask = j != 0 && nvc2[i][j - 1][k] != 2 ? mask | NB_SAME_DOWN : mask;
                mask = j != (VCUBEWIDTH) && nvc2[i][j + 1][k] != 2 ? mask | NB_SAME_UP : mask;
                mask = i != 0 && nvc2[i - 1][j][k] != 2 ? mask | NB_SAME_FRONT : mask;
                mask = i != (VCUBEWIDTH) && nvc2[i + 1][j][k] != 2 ? mask | NB_SAME_BACK : mask;
                this->masscube2[ind].neighbour_same = mask;

                //// other volcube bitmasks
                mask = 0x00;
                mask = (i != 0 && j != 0 && k != 0) && nvc1[i - 1][j - 1][k - 1] != 2 ? mask | NB_OTHER_NEAR_BOT_LEFT : mask;
                mask = (i != 0 && j != 0 && k != VCUBEWIDTH) && nvc1[i - 1][j - 1][k] != 2 ? mask | NB_OTHER_NEAR_BOT_RIGHT : mask;
                mask = (i != 0 && j != VCUBEWIDTH && k != 0) && nvc1[i - 1][j][k - 1] != 2 ? mask | NB_OTHER_NEAR_TOP_LEFT : mask;
                mask = (i != 0 && j != VCUBEWIDTH && k != VCUBEWIDTH) && nvc1[i - 1][j][k] != 2 ? mask | NB_OTHER_NEAR_TOP_RIGHT : mask;
                mask = (i != VCUBEWIDTH && j != 0 && k != 0) && nvc1[i][j - 1][k - 1] != 2 ? mask | NB_OTHER_FAR_BOT_LEFT : mask;
                mask = (i != VCUBEWIDTH && j != 0 && k != VCUBEWIDTH) && nvc1[i][j - 1][k] != 2 ? mask | NB_OTHER_FAR_BOT_RIGHT : mask;
                mask = (i != VCUBEWIDTH && j != VCUBEWIDTH && k != 0) && nvc1[i][j][k - 1] != 2 ? mask | NB_OTHER_FAR_TOP_LEFT : mask;
                mask = (i != VCUBEWIDTH && j != VCUBEWIDTH && k != VCUBEWIDTH) && nvc1[i][j][k] != 2 ? mask | NB_OTHER_FAR_TOP_RIGHT : mask;
                this->masscube2[ind].neighbour_other = mask;
            }
        }
    }
}

//--------------------------------------------------------------------------------------
// Add offset to picking helper IDs
//--------------------------------------------------------------------------------------
void Deformable::addOffset(){

    int offset1 = id * VCUBEWIDTH;           // object masspoint1 offset in central containers, added to Z mpid
    int offset2 = id * (VCUBEWIDTH + 1);     // object masspoint2 offset in central containers, added to Z mpid

    // add offset to particle data
    for (uint i = 0; i < particles.size(); i++){
        particles[i].mpid1.z += offset1;
        particles[i].mpid2.z += offset2;
    }

    // add offset to indexer data
    for (uint i = 0; i < indexcube.size(); i++){
        indexcube[i].vc1index.z += offset1;
        indexcube[i].vc2index.z += offset2;
    }
}

//--------------------------------------------------------------------------------------
// Translate model and masscubes in space
//--------------------------------------------------------------------------------------
void Deformable::translate(int x, int y, int z){

    // add vector to model points
    for (uint i = 0; i < particles.size(); i++){
        particles[i].pos.x += x;
        particles[i].pos.y += y;
        particles[i].pos.z += z;
        particles[i].npos.x += x;
        particles[i].npos.y += y;
        particles[i].npos.z += z;
    }

    // add offset to indexer data
    for (uint i = 0; i < masscube1.size(); i++){
        masscube1[i].newpos.x += x;
        masscube1[i].oldpos.x += x;
        masscube1[i].newpos.y += y;
        masscube1[i].oldpos.y += y;
        masscube1[i].newpos.z += z;
        masscube1[i].oldpos.z += z;
    }

    // add offset to indexer data
    for (uint i = 0; i < masscube2.size(); i++){
        masscube2[i].newpos.x += x;
        masscube2[i].oldpos.x += x;
        masscube2[i].newpos.y += y;
        masscube2[i].oldpos.y += y;
        masscube2[i].newpos.z += z;
        masscube2[i].oldpos.z += z;
    }
}

//--------------------------------------------------------------------------------------
// Initialize collision detection helper structures
// CALL MANUALLY AFTER TRANSLATING THE MODEL!
//--------------------------------------------------------------------------------------
void Deformable::initCollisionDetection(){

    MassIDTypeVector tmp;

    // create MassIDs from surface masspoints in 1st vc
    for (uint z = 0; z < VCUBEWIDTH; z++){
        for (uint y = 0; y < VCUBEWIDTH; y++){
            for (uint x = 0; x < VCUBEWIDTH; x++){
                if (nvc1[z][y][x] == 1){               // type 1 masspoint -> model surface masspoints
                    tmp.push_back(MassIDType(z*VCUBEWIDTH*VCUBEWIDTH + y*VCUBEWIDTH + x, 1, masscube1[z*VCUBEWIDTH*VCUBEWIDTH + y*VCUBEWIDTH + x]));
                }
            }
        }
    }

    // create MassIDs from surface masspoints in 2nd vc
    for (uint z = 0; z < VCUBEWIDTH + 1; z++){
        for (uint y = 0; y < VCUBEWIDTH + 1; y++){
            for (uint x = 0; x < VCUBEWIDTH + 1; x++){
                if (nvc2[z][y][x] == 1){               // type 1 masspoint -> model surface masspoints
                    tmp.push_back(MassIDType(z*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + y*(VCUBEWIDTH + 1) + x, 2, masscube2[z*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + y*(VCUBEWIDTH + 1) + x]));
                }
            }
        }
    }
    ctree = BVHierarchy(tmp).bvh;         // store BVHierarchy
}

//--------------------------------------------------------------------------------------
// Destructor
//--------------------------------------------------------------------------------------
Deformable::~Deformable(){ /* nothing dynamic to dispose of*/ }
