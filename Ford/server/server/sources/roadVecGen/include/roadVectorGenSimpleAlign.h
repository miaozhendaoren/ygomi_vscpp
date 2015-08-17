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
#ifndef ROAD_VECTOR_GEN_SIMPLE_ALIGN_H
#define ROAD_VECTOR_GEN_SIMPLE_ALIGN_H

#include <vector>
#include <list>
#include <queue>
#include "database.h" // tlvCommon_t
#include "databaseServer.h" // databaseServer
#include "laneQueueClass.h" // laneQueueClass

bool roadVectorGenSimpleAlign(laneSpace::laneQueueClass &newDataQueues, std::vector<int> &storeLane);

#endif
