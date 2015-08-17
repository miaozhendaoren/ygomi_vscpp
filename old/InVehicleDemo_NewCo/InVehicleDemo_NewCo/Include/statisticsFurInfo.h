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
using namespace ns_database;
struct statisticsFurInfo_t
{
	int16 number;
	furAttributes_t furAttri;
};
struct deleteFurInfo_t
{
	int16 delNumber;
	int16 detNumber;
	int16 frameNumber;
	furAttributes_t furAttri;
};
