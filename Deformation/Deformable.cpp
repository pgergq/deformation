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
#include <fstream>
#include <sstream>
#include <algorithm>

//--------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------
Deformable::Deformable(int i, std::string x){

	this->index = i;
	this->file = x;

	this->init();
}

//--------------------------------------------------------------------------------------
// Init Deformable model data: import vertices|normals|faces from file
//--------------------------------------------------------------------------------------
void Deformable::init(){

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
	g_nVolCubeCell = (int)tmp;

	// Set global volumetric cube offsets to align the model
	g_vVolCubeO = XMFLOAT3(floor(minx), floor(miny), floor(minz));

	// Set model vertex count
	g_nNumParticles = g_vVertices.size();

}






















Deformable::~Deformable(){}
