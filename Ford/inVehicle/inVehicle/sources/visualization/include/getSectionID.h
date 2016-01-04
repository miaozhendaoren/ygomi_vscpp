/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  detection.h
* @brief Road furniture detection header file
*
* Change Log:
*      Date                Who             What
*      2015/09/09          Qian Xu         Create
*******************************************************************************
*/
#ifndef GETSECTIONID_H
#include <list>
#include <vector>
#include "string"
#include "windows.h"
#include "database.h"
#include "RoadSeg.h"
#include "AppInitCommon.h"
#include "configure.h"

using namespace ns_roadsegment;

bool readSectionConfig(string configPath,list<ns_database::segAttributes_t> &segCfgList);

#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE) 
bool getSectionId(IN point3D_t inPoint,
				  IN int currentloopIdx,
	              OUT uint32 &segId);

bool getSegIdAndDirection(IN point3D_t inPoint1,
	                      IN point3D_t inPoint2,
						  IN int currentloopIdx,
	                      OUT uint32 &segId,
	                      OUT bool &reverseFlag);
#else
bool getSectionId(IN point3D_t inPoint,
	              IN list<segAttributes_t> segConfigList,
	              OUT uint32 &foundSectionID);

bool getSegIdAndDirection(IN point3D_t inPoint1,
	                      IN point3D_t inPoint2,
	                      IN list<segAttributes_t> segConfigList,
	                      OUT uint32 &foundSectionID,
	                      OUT bool &reverseFlag);
#endif

bool fixVehicleLocationInLane(point3D_t reletiveGps,list<vector<point3D_t>> &allLines,bool direction, point3D_t *locationInLane);

bool checkLineSection(list<list<lineAttributes_t>> &lineAttr, int sectionId, int &LineNum);

#endif
