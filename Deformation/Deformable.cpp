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

#include "Deformable.h"
#include "Constants.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

//--------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------
Deformable::Deformable(std::string x){

	this->file = x;		// file must be .obj formatted, with equal number of vertices and vertex normals (plus optional face data)
	this->importFile();
	this->initVars();
	this->initMasscubes();
	this->initIndexer();
	this->initNeighbouring();
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
				f.push_back(atoi(s.c_str()));
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
	this->modelPos = XMFLOAT3(MODEL_OFFSET, MODEL_OFFSET, MODEL_OFFSET);

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
		XMVECTOR tmp = XMVectorAdd(XMVectorSet(this->vertices[i][1], this->vertices[i][2], this->vertices[i][3], 1),
						XMVectorSet(this->modelPos.x, this->modelPos.y, this->modelPos.z, 0));
		XMStoreFloat4(&push.pos, tmp);

		// normalized normals -> store the endpoint of the normals (=npos)
		float len = this->normals[i][1] * this->normals[i][1] + this->normals[i][2] * this->normals[i][2] + this->normals[i][3] * this->normals[i][3];
		len = (len == 0 ? -1 : sqrtf(len));
		XMVECTOR tmp2 = XMVectorSet((float)this->normals[i][1] / len, (float)this->normals[i][2] / len, (float)this->normals[i][3] / len, 1);
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
		// Get indices and weights in first volumetric cube
		vertex = XMFLOAT3(this->particles[i].pos.x, this->particles[i].pos.y, this->particles[i].pos.z);
		int x = (std::max(vertex.x, vc_pos1.x) - std::min(vertex.x, vc_pos1.x)) / this->cubeCellSize;
		int y = (std::max(vertex.y, vc_pos1.y) - std::min(vertex.y, vc_pos1.y)) / this->cubeCellSize;
		int z = (std::max(vertex.z, vc_pos1.z) - std::min(vertex.z, vc_pos1.z)) / this->cubeCellSize;
		XMStoreFloat3(&this->indexcube[i].vc1index, XMVectorSet(x, y, z, 0));
		XMStoreFloat4(&this->particles[i].mpid1, XMVectorSet(x, y, z, 1));

		// trilinear interpolation
		vind = z*VCUBEWIDTH*VCUBEWIDTH + y*VCUBEWIDTH + x;
		float wx = (vertex.x - this->masscube1[vind].newpos.x) / this->cubeCellSize;
		float dwx = 1.0f - wx;
		float wy = (vertex.y - this->masscube1[vind].newpos.y) / this->cubeCellSize;
		float dwy = 1.0f - wy;
		float wz = (vertex.z - this->masscube1[vind].newpos.z) / this->cubeCellSize;
		float dwz = 1.0f - wz;
		this->indexcube[i].w1[0] = dwx*dwy*dwz; this->indexcube[i].w1[1] = wx*dwy*dwz; this->indexcube[i].w1[2] = dwx*wy*dwz; this->indexcube[i].w1[3] = wx*wy*dwz;
		this->indexcube[i].w1[4] = dwx*dwy*wz; this->indexcube[i].w1[5] = wx*dwy*wz; this->indexcube[i].w1[6] = dwx*wy*wz; this->indexcube[i].w1[7] = wx*wy*wz;

		// trilinear for npos
		vertex = XMFLOAT3(this->particles[i].npos.x, this->particles[i].npos.y, this->particles[i].npos.z);
		wx = (vertex.x - this->masscube1[vind].newpos.x) / this->cubeCellSize;
		dwx = 1.0f - wx;
		wy = (vertex.y - this->masscube1[vind].newpos.y) / this->cubeCellSize;
		dwy = 1.0f - wy;
		wz = (vertex.z - this->masscube1[vind].newpos.z) / this->cubeCellSize;
		dwz = 1.0f - wz;
		this->indexcube[i].nw1[0] = dwx*dwy*dwz; this->indexcube[i].nw1[1] = wx*dwy*dwz; this->indexcube[i].nw1[2] = dwx*wy*dwz; this->indexcube[i].nw1[3] = wx*wy*dwz;
		this->indexcube[i].nw1[4] = dwx*dwy*wz; this->indexcube[i].nw1[5] = wx*dwy*wz; this->indexcube[i].nw1[6] = dwx*wy*wz; this->indexcube[i].nw1[7] = wx*wy*wz;


		// Fill second indexer
		vertex = XMFLOAT3(this->particles[i].pos.x, this->particles[i].pos.y, this->particles[i].pos.z);
		x = (std::max(vertex.x, vc_pos2.x) - std::min(vertex.x, vc_pos2.x)) / this->cubeCellSize;
		y = (std::max(vertex.y, vc_pos2.y) - std::min(vertex.y, vc_pos2.y)) / this->cubeCellSize;
		z = (std::max(vertex.z, vc_pos2.z) - std::min(vertex.z, vc_pos2.z)) / this->cubeCellSize;
		XMStoreFloat3(&this->indexcube[i].vc2index, XMVectorSet(x, y, z, 0));
		XMStoreFloat4(&this->particles[i].mpid2, XMVectorSet(x, y, z, 1));

		vind = z*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + y*(VCUBEWIDTH + 1) + x;
		wx = (vertex.x - this->masscube2[vind].newpos.x) / this->cubeCellSize;
		dwx = 1.0f - wx;
		wy = (vertex.y - this->masscube2[vind].newpos.y) / this->cubeCellSize;
		dwy = 1.0f - wy;
		wz = (vertex.z - this->masscube2[vind].newpos.z) / this->cubeCellSize;
		dwz = 1.0f - wz;
		this->indexcube[i].w2[0] = dwx*dwy*dwz; this->indexcube[i].w2[1] = wx*dwy*dwz; this->indexcube[i].w2[2] = dwx*wy*dwz; this->indexcube[i].w2[3] = wx*wy*dwz;
		this->indexcube[i].w2[4] = dwx*dwy*wz; this->indexcube[i].w2[5] = wx*dwy*wz; this->indexcube[i].w2[6] = dwx*wy*wz; this->indexcube[i].w2[7] = wx*wy*wz;

		vertex = XMFLOAT3(this->particles[i].npos.x, this->particles[i].npos.y, this->particles[i].npos.z);
		wx = (vertex.x - this->masscube2[vind].newpos.x) / this->cubeCellSize;
		dwx = 1.0f - wx;
		wy = (vertex.y - this->masscube2[vind].newpos.y) / this->cubeCellSize;
		dwy = 1.0f - wy;
		wz = (vertex.z - this->masscube2[vind].newpos.z) / this->cubeCellSize;
		dwz = 1.0f - wz;
		this->indexcube[i].nw2[0] = dwx*dwy*dwz; this->indexcube[i].nw2[1] = wx*dwy*dwz; this->indexcube[i].nw2[2] = dwx*wy*dwz; this->indexcube[i].nw2[3] = wx*wy*dwz;
		this->indexcube[i].nw2[4] = dwx*dwy*wz; this->indexcube[i].nw2[5] = wx*dwy*wz; this->indexcube[i].nw2[6] = dwx*wy*wz; this->indexcube[i].nw2[7] = wx*wy*wz;

	}
}

//--------------------------------------------------------------------------------------
// Init (6) Deformable model data: initialize neighbouring on masspoints
//--------------------------------------------------------------------------------------
void Deformable::initNeighbouring(){

}













Deformable::~Deformable(){ /* nothing dynamic to dispose of*/ }
