//--------------------------------------------------------------------------------------
// File: DeformableOBJ.cpp
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
#include <boost/algorithm/string.hpp>
#include "DXUT.h"
#include "../Headers/DeformableOBJ.h"
#include "../Headers/Constants.h"
#include "../Headers/Collision.h"



//--------------------------------------------------------------------------------------
// Init (1) Deformable model data: import vertices|normals|faces from file
//--------------------------------------------------------------------------------------
void DeformableOBJ::importFile(){

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
                auto z = strtok(const_cast<char*>(s.c_str()), "//");
                f.push_back(atoi(z));
            }
            //store
            this->faces.push_back(f);
        }
    }

    // Close file
    input.close();
}