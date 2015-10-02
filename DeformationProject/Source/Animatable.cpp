//--------------------------------------------------------------------------------------
// File: Animatable.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Animatable class definition
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include "DXUT.h"
#include "../Headers/Animatable.h"
#include "../Headers/Constants.h"
#include "../Headers/Collision.h"


//--------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------
Animatable::Animatable(std::string x, int id) : dObject(x, id){

    // build deformable object structures
    dObject.build();

}

//--------------------------------------------------------------------------------------
// Translate inner deformable object
//--------------------------------------------------------------------------------------
void Animatable::translateDeformable(int x, int y, int z)
{
    dObject.translate(x, y, z);
}
