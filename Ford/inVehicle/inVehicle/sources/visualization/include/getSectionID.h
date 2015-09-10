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

bool readSectionConfig(string configPath,list<ns_database::segAttributes_t> &segCfgList);
//double getLength1(IN double point1_x,IN double point1_y, IN double point2_x, IN double point2_y);
//called by getSectionId

uint32 getSectionId(IN point3D_t inPoint,
                    IN list<segAttributes_t> &segConfigList,
                    OUT uint32 &foundSectionID);

bool fixVehicleLocationInLane(point3D_t reletiveGps,list<vector<point3D_t>> &allLines, point3D_t *locationInLane);

bool checkLineSection(list<list<lineAttributes_t>> &lineAttr, int sectionId, int &LineNum);

#endif
