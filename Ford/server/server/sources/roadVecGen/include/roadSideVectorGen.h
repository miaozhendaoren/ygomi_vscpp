/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  roadSideVectorGen.cpp
* @brief Source file for  road side vector generation
*
* Change Log:
*      Date                Who             What
*      2015/5/15          yuanzhang      Create
*******************************************************************************
*/

#ifndef ROAD_SIDE_VECTOR_GEN_H
#define ROAD_SIDE_VECTOR_GEN_H

#include <vector>
#include <list>
#include <queue>
#include "database.h" // tlvCommon_t
#include "icpPointToPoint.h"// Matrix

#define STITCH_WEIGHT (3) // DB = (new + (STITCH_WEIGHT - 1) * old_DB) / STITCH_WEIGHT;

using std::vector;
using std::list;
using std::queue;
using namespace ns_database;

void roadSideVectorGen(IN  vector<point3D_t>& line, 
                       IN  vector<double>& distanceInMeterR, 
                       IN  vector<double>& distanceInMeterL, 
                       OUT vector<point3D_t>& lineR, 
                       OUT vector<point3D_t>& lineL);

void roadSideDistGen(IN  vector<laneType_t>& queueP, 
                     OUT vector<point3D_t>& line,
                     OUT vector<double>& distR, 
                     OUT vector<double>& distL);

void simpleMatchIdx(IN std::vector<int32> &activeInNewData, 
                    IN int newDataLen, 
                    IN std::vector<int32> &matchIdxInDB, 
                    IN std::vector<int32> &closeIdxInDB, 
                    IN int dbDataLen, 
                    OUT std::vector<int32> &outputNewDataIdx,
                    OUT std::vector<int32> &outputDBIdx);

void simpleStitch(IN  vector<int32>    &newDataIdxVec,
                  IN  vector<point3D_t> &newDataVec,
                  IN  vector<int32>    &dbDataIdxVec, 
                  IN  vector<point3D_t> &dbDataVec, 
                  OUT vector<point3D_t> &mergedVec);

void stitchLineWithIdx(IN    vector<int32>    &newDataIdxVec, 
                       INOUT vector<point3D_t>  &newDbDataVec,
                       INOUT vector<double>     &distRVec,
                       INOUT vector<double>     &distLVec,
                       IN    vector<int32>    &dbDataIdxVec, 
                       IN    vector<point3D_t>  &linesInSegVec,
                       IN    vector<point3D_t>  &linesRInSegVec,
                       IN    vector<point3D_t>  &linesLInSegVec);

void getLocationArray(IN  vector<point3D_t>& dataVec, 
                      IN  point3D_t& standPoint,
                      OUT double **T);
#endif
