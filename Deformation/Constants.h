//--------------------------------------------------------------------------------------
// File: Constants.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Deformation constant definitions
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

// constant defines for Deformation.cpp and Deformable.cpp

#define GET_X_LPARAM(lp)                        ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp)                        ((int)(short)HIWORD(lp))


/// DEFORMATION defines
#define VCUBEWIDTH		19											// n*n*n inner cube, (n+1)*(n+1)*(n+1) outer cube
#define VOLCUBE1_COUNT	VCUBEWIDTH*VCUBEWIDTH*VCUBEWIDTH			// number of masspoints in the smaller volumetric cube
#define VOLCUBE2_COUNT	(VCUBEWIDTH+1)*(VCUBEWIDTH+1)*(VCUBEWIDTH+1) // number of masspoints in the bigger volumetric cube
#define MODEL_OFFSET	100

// volcube neighbouring data
#define NB_SAME_LEFT			0x20								// 0010 0000, has left neighbour
#define NB_SAME_RIGHT			0x10								// 0001 0000, has right neighbour
#define NB_SAME_DOWN			0x08								// 0000 1000, ...
#define NB_SAME_UP				0x04								// 0000 0100
#define NB_SAME_FRONT			0x02								// 0000 0010
#define NB_SAME_BACK			0x01								// 0000 0001
// NEAR = lower end of Z axis, nearer to the viewer
// FAR = higher Z values, farther into the screen
#define NB_OTHER_NEAR_BOT_LEFT	0x80								// 1000 0000
#define NB_OTHER_NEAR_BOT_RIGHT	0x40								// 0100 0000
#define NB_OTHER_NEAR_TOP_LEFT	0x20								// 0010 0000
#define NB_OTHER_NEAR_TOP_RIGHT	0x10								// 0001 0000
#define NB_OTHER_FAR_BOT_LEFT	0x08								// 0000 1000
#define NB_OTHER_FAR_BOT_RIGHT	0x04								// 0000 0100
#define NB_OTHER_FAR_TOP_LEFT	0x02								// 0000 0010
#define NB_OTHER_FAR_TOP_RIGHT	0x01								// 0000 0001



// typedefs for easier handling, also used in Deformation.cpp
typedef std::vector<float> vec1float;
typedef std::vector<int> vec1int;
typedef std::vector<std::vector<float>> vec2float;
typedef std::vector<std::vector<int>> vec2int;


#endif