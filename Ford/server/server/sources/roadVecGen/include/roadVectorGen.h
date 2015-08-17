/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  roadVectorGen.h
* @brief Header file for calculating principal curves and road vector gen
*
* Change Log:
*      Date                Who             What
*      2015/5/12           Linkun Xu       Create
*******************************************************************************
*/
#ifndef ROAD_VECTOR_GEN_H
#define ROAD_VECTOR_GEN_H

#include <vector>
#include <list>
#include <queue>
#include "database.h" // tlvCommon_t
#include "databaseServer.h" // databaseServer
#include "laneQueueClass.h" // laneQueueClass

enum roadVecGenEnum : uint32
{
    pushAllVec_e,
    simpleAlign_e,
    dashAlign_e,
};

#define ROAD_VEC_GEN_DEBUG 0

#define SEGMENT_ID_DEMO 1

bool roadVectorGen(laneSpace::laneQueueClass &newDataQueues, std::vector<int> &storeLane, roadVecGenEnum alg_e);

#endif
