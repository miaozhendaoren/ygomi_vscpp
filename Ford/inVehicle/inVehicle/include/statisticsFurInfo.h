/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Visualization.h
* @brief A sample 3D engine, provide some function to add road, sign, and perspective information.
*        road and sign information can load in once or each seconds.
*        perspective information is based one road and sign coordinates, each frame has its own information.
*
* Change Log:
*      Date                Who             What
*      2015/01/28         Qin Shi         Create
*******************************************************************************
*/
#pragma once
#include "database.h"
#include "databaseInVehicle.h"

using ns_database::furAttributesInVehicle_t;
using ns_database::point3D_t;

namespace ns_statistics
{
struct furWithPosition_t
{
    public:
        ns_database::furAttributesInVehicle_t  furAttri;
        int offsetNumPerFur;
        std::vector<float> offset;
        std::vector<point3D_t> position;
};

struct statisticsFurInfo_t
{
	int16 number;
	std::vector<point3D_t> firstGps;
	furAttributesInVehicle_t  furAttri;
    std::vector<int> offsetNumPerFur;
    std::vector<std::vector<float>> offset;
    std::vector<std::vector<point3D_t>> position;
};
struct deleteFurInfo_t
{
	int16 delNumber;
	int16 detNumber;
	int16 frameNumber;
	furAttributesInVehicle_t furAttri;
};

 const double MIN_DIST = 9e+100;
 const int VAR_ACC_TIME = 1;
 const bool VAR_ALG = 0;
 const double DIST_METER = 50;
 const double PARALLEL_LINE_DEGREE = 1.0;
 const double DIST_THREHOLD = 20.0;
}