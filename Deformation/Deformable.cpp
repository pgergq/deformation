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
	this->import();
	this->initVars();
}

//--------------------------------------------------------------------------------------
// Init (1) Deformable model data: import vertices|normals|faces from file
//--------------------------------------------------------------------------------------
void Deformable::import(){

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
		std::cout << "[ERROR]: vertexCount and normalCount are not equal!" << std::endl;
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



















Deformable::~Deformable(){ /* nothing dynamic to dispose of*/ }
