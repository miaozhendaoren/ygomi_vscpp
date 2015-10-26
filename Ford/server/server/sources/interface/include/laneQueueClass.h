/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  laneQueueClass.h
* @brief Header file for queuing GPS points
*
* Change Log:
*      Date                Who             What
*      2015/5/10           Qin Shi       Create
*******************************************************************************
*/

#pragma once
#include <queue>
#include <list>
#include <vector>
#include "database.h"

namespace laneSpace
{
	using namespace ns_database;
	#define MAX_LANE_NUM	12
	#define MAX_GPS_NUM_PER_LANE	300
	class laneQueueClass
	{
	private: 
		int laneNum;
		std::queue<laneType_t> * laneQueuePtr[MAX_LANE_NUM];
		//HANDLE _hMutex;
	public:
		laneQueueClass(void);
		bool addLaneQueue(int laneId);
		void addLanePoint(int laneId,laneType_t &linePoint);
		bool specifiedLaneQueueSize(int laneId,int *queueSize);
		bool deleteSpecifiedLaneQueue(int laneId);
        void getAllVectors(OUT std::list<std::list<std::vector<point3D_t>>> &newDataLaneList,
            OUT std::list<std::vector<point3D_t>> &newDataGpsList);
		std::queue<laneType_t>*  getSpecifiedLane(int laneId);
		~laneQueueClass(void);
	};
}

