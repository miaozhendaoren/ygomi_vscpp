#include "saveLinePointInSafe.h"

namespace ns_historyLine
{
	saveLinePointInSafe::saveLinePointInSafe(int numberLine,int numberGps)
	{
		lineNum = numberLine;
		gpsNum = numberGps;
		_lineMutex = CreateMutex(NULL,FALSE,NULL);
		ReleaseMutex(_lineMutex);
		_gpsMutex = CreateMutex(NULL,FALSE,NULL);
		ReleaseMutex(_gpsMutex);
	}

	void saveLinePointInSafe::getHistoryBuffer(list<lineInfoPerVector_t> &historyList)
	{
		WaitForSingleObject(_lineMutex,INFINITE);
		historyList.clear();
		list<lineInfoPerVector_t>::iterator lineIter = historyLine.begin();
		while(lineIter != historyLine.end())
		{
			historyList.push_back(*lineIter);
			lineIter++;
		}
		ReleaseMutex(_lineMutex);

	}
	void saveLinePointInSafe::getGpsBuffer(list<point3D_t> &gpsList)
	{
		WaitForSingleObject(_gpsMutex,INFINITE);
		gpsList.clear();
		list<point3D_t>::iterator gpsIter = gpsBuffer.begin();

		while(gpsIter != gpsBuffer.end())
		{
			gpsList.push_back(*gpsIter);
			gpsIter++;
		}

		ReleaseMutex(_gpsMutex);
	}

	void saveLinePointInSafe::saveHistoryLine(databaseInVehicle* database_gp)
	{
		lineInfoPerVector_t lineInfoPerVector;
		list<list<vector<point3D_t>>> allLines; // segment list / vector list / point list / point
		list<list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
		database_gp->getAllVectors(allLines, lineAttr);

		//point3D_t standPoint = (*(*allLines.begin()).begin())[0];

		list<list<vector<point3D_t>>>::iterator lineInSegIter = allLines.begin();
		list<list<lineAttributes_t>>::iterator lineAttrInSegIter = lineAttr.begin();

		// For each segment
		while(lineInSegIter != allLines.end())
		{
			list<vector<point3D_t>>::iterator lineIter = (*lineInSegIter).begin();
			list<lineAttributes_t>::iterator lineAttrIter = (*lineAttrInSegIter).begin();

			// For each vector
			while(lineIter != (*lineInSegIter).end())
			{
				if((*lineAttrIter).lineStyle == 0)
				{
					lineInfoPerVector.lineStyle = (*lineAttrIter).lineStyle;

					lineInfoPerVector.pointNum = (*lineAttrIter).numPoints;
				
					for(int pointIdx = 0; pointIdx < (*lineAttrIter).numPoints; pointIdx++)
					{
						point3D_t *gpsPoint = &(*lineIter)[pointIdx];
						lineInfoPerVector.allGps.push_back(*gpsPoint);
						//coordinateChange(&standPoint,&(*lineIter)[pointIdx], &(pointVecBuf[pointIdx]));
					}
				}
					lineIter++;
					lineAttrIter++;
				
			}

			lineInSegIter++;
			lineAttrInSegIter++;
		}

		WaitForSingleObject(_lineMutex,INFINITE);
		if(allLines.size() > 0)
		{
			historyLine.push_back(lineInfoPerVector);
			// delete the line
			if(historyLine.size() > lineNum)
			{
				lineInfoPerVector = historyLine.front();
				lineInfoPerVector.allGps.clear();
				historyLine.pop_front();
			}
		}
		ReleaseMutex(_lineMutex);

		database_gp->getAllVectors_clear(allLines, lineAttr);

	}

	void saveLinePointInSafe::saveCurrentGps(point3D_t gpsPoint)
	{
		WaitForSingleObject(_gpsMutex,INFINITE);
		gpsBuffer.push_back(gpsPoint);
		if(gpsBuffer.size() > gpsNum)
		{
			gpsBuffer.pop_front();
		}
		ReleaseMutex(_gpsMutex);
	}

	saveLinePointInSafe::~saveLinePointInSafe()
	{
		historyLine.clear();
		gpsBuffer.clear();
		CloseHandle(_lineMutex); 
		CloseHandle(_gpsMutex); 
	}
}
