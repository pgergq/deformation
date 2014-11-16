//--------------------------------------------------------------------------------------
// File: Constants.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Constant definitions
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#include "Constants.h"

std::atomic<float> spreadConstant = 400.0f;
std::atomic<float> stiffnessConstant = 400.0f;
std::atomic<float> dampingConstant = -3.0f;
std::atomic<float> invMassConstant = 1.0f;
std::atomic<float> collisionRangeConstant = 5.0f;
std::atomic<float> gravityConstant = -1000.0f;
std::atomic<float> tablePositionConstant = -1000.0f;
std::atomic<VECTOR4> lightPos(VECTOR4{ 0, 0, -10000, 1 });
std::atomic<VECTOR4> lightCol(VECTOR4{ 0, 1, 1, 1 });