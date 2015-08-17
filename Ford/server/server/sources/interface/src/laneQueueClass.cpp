#include <queue>
#include "database.h"
#include "laneQueueClass.h"

namespace laneSpace
{
	laneQueueClass::laneQueueClass(void)
	{
		laneNum = 0;
		//_hMutex = CreateMutex(NULL,FALSE,NULL);
		//ReleaseMutex(_hMutex); 
	}
	bool laneQueueClass::addLaneQueue(int laneId)
	{
	//	WaitForSingleObject(_hMutex,INFINITE);
		if((laneQueuePtr[laneId] == NULL) && (laneNum < MAX_LANE_NUM))
		{
			laneQueuePtr[laneId] = new std::queue<laneType_t>;
			//ReleaseMutex(_hMutex);
			laneNum++;
			return true;
		}
		else
		{
			//ReleaseMutex(_hMutex);
			return false;
		}
	}
	void laneQueueClass::addLanePoint(int laneId,laneType_t &linePoint)
	{
		//WaitForSingleObject(_hMutex,INFINITE);
		if(laneQueuePtr[laneId] != NULL) 
		{
		
			std::queue<laneType_t> * lanePtr = laneQueuePtr[laneId];
			lanePtr->push(linePoint);
			//ReleaseMutex(_hMutex);
		}
		else if(laneNum < MAX_LANE_NUM)
		{
			laneQueuePtr[laneId] = new std::queue<laneType_t>;
			std::queue<laneType_t> * lanePtr = laneQueuePtr[laneId];
			lanePtr->push(linePoint);
			laneNum++;
			//ReleaseMutex(_hMutex);
		}
	}
	bool laneQueueClass::specifiedLaneQueueSize(int laneId,int *queueSize)
	{
		//WaitForSingleObject(_hMutex,INFINITE);
		if(laneQueuePtr[laneId] != NULL)
		{
			*queueSize = laneQueuePtr[laneId]->size();
		//	ReleaseMutex(_hMutex);
			return true;
		}
		else
		{
			//ReleaseMutex(_hMutex);
			return false;
		}
	}
	bool laneQueueClass::deleteSpecifiedLaneQueue(int laneId)
	{
		//WaitForSingleObject(_hMutex,INFINITE);
		if(laneQueuePtr[laneId] != NULL)
		{
			int gpsNum = laneQueuePtr[laneId]->size();
			for(int idx = 0; idx < gpsNum;++idx)
			{
				laneQueuePtr[laneId]->pop();
			}
			delete laneQueuePtr[laneId];
			laneQueuePtr[laneId] = NULL ;
			//ReleaseMutex(_hMutex);
			laneNum--;
			return true;
		}
		else
		{
			//ReleaseMutex(_hMutex);
			return false;
		}
	}
	std::queue<laneType_t>*  laneQueueClass::getSpecifiedLane(int laneId)
	{
			return laneQueuePtr[laneId];
	}

	laneQueueClass::~laneQueueClass(void)
	{
		for(int idx = 0; idx < MAX_LANE_NUM;++idx)
		{
			deleteSpecifiedLaneQueue(idx);
		}
		laneNum = 0;
		//CloseHandle(_hMutex);  
	}
}
