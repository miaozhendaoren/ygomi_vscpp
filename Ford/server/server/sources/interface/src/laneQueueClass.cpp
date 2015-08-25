#include <queue>
#include "database.h"
#include "laneQueueClass.h"

using std::queue;
using std::list;
using std::vector;

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

    void laneQueueClass::getAllVectors(OUT list<list<vector<point3D_t>>> &newDataList)
    {
        newDataList.clear();
        int laneId = 0;

        for(int laneIdx = 0; laneIdx < MAX_LANE_NUM; ++laneIdx)
        {
            if(laneQueuePtr[laneId] != NULL)
            {
                list<vector<point3D_t>> newDataSection;
                vector<point3D_t> newDataVecL;
                vector<point3D_t> newDataVecR;

                queue<laneType_t> * lanePtr = laneQueuePtr[laneId];

                while(lanePtr->size() > 0)
                {
                    laneType_t newDataTmp = lanePtr->front();;
                    lanePtr->pop();

                    point3D_t newDataL, newDataR;

                    newDataL = newDataTmp.gpsL;
                    newDataR = newDataTmp.gpsR;

                    newDataL.paintFlag = newDataTmp.linePaintFlagL;
                    newDataR.paintFlag = newDataTmp.linePaintFlagR;

                    newDataVecL.push_back(newDataL);
                    newDataVecR.push_back(newDataR);
                }

                newDataSection.push_back(newDataVecL);
                newDataSection.push_back(newDataVecR);

                newDataList.push_back(newDataSection);

                laneId++;
            }
        }
    }
}
