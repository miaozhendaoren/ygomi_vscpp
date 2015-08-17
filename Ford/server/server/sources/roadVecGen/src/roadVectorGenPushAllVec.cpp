/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  roadVectorGen.cpp
* @brief Source file for calculating principal curve and road vector generation
*
* Change Log:
*      Date                Who             What
*      2015/5/12           Linkun Xu       Create
*******************************************************************************
*/

#include "roadVectorGen.h" // SEGMENT_ID_DEMO
#include "roadVectorGenPushAllVec.h"
#include "roadSideVectorGen.h"
#include "LogInfo.h"

using std::vector;
using std::queue;
using std::list;
using namespace laneSpace;
using namespace ns_database;

extern databaseServer* database_gp;

bool roadVectorGenPushAllVec(laneQueueClass &newDataQueues, vector<int> &storeLane)
{
    for(int laneIdx = 0; laneIdx < storeLane.size(); laneIdx++)
    {
        int dbDataLen;

        list<list<vector<point3D_t>>> allLines;
        list<list<lineAttributes_t>> lineAttr;

        database_gp->getAllVectors(allLines, lineAttr);

        int laneId = storeLane[laneIdx];
        queue<laneType_t>* queueP = newDataQueues.getSpecifiedLane(laneId);

        int newDataLen = queueP->size();

        vector<laneType_t> newDataVec;
        laneType_t newDataTmp;
        newDataVec.assign(newDataLen, newDataTmp);

        for(int idx = 0; idx < newDataLen; idx++)
        {
            newDataTmp = queueP->front();
            queueP->pop();
            newDataVec[idx] = newDataTmp;
        }

        vector<point3D_t> line;
        vector<double> distR;
        vector<double> distL;

        roadSideDistGen(newDataVec, line, distR, distL);

        lineAttributes_t lineAttrTmp;
        {
            lineAttrTmp.segmentId = SEGMENT_ID_DEMO; // FIXME: only one segment exist for demo
            lineAttrTmp.lineId = 1;
            lineAttrTmp.width  = 0.1;
            lineAttrTmp.lineStyle  = 0; // reference line
            lineAttrTmp.segVersion = 0;
            lineAttrTmp.numPoints  = line.size();
        }

        // Generate road vectors with road sides
        vector<point3D_t> lineR, lineL;

        roadSideVectorGen(line, distR, distL, lineR, lineL);

        
        if (allLines.size() != 0)
        {
            allLines.begin()->push_back(line);	
            lineAttr.begin()->push_back(lineAttrTmp);

            allLines.begin()->push_back(lineR);
            lineAttrTmp.lineId = 2;
            lineAttrTmp.lineStyle  = 5;
            lineAttr.begin()->push_back(lineAttrTmp);

            allLines.begin()->push_back(lineL);
            lineAttrTmp.lineId = 3;
            lineAttrTmp.lineStyle  = 5;
            lineAttr.begin()->push_back(lineAttrTmp);
        }else
        {
            list<vector<point3D_t>> allLinesInSeg;
            list<lineAttributes_t> lineAttrInSeg;

            allLinesInSeg.push_back(line);	
            lineAttrInSeg.push_back(lineAttrTmp);

            allLinesInSeg.push_back(lineR);
            lineAttrTmp.lineId = 2;
            lineAttrTmp.lineStyle  = 5;
            lineAttrInSeg.push_back(lineAttrTmp);

            allLinesInSeg.push_back(lineL);
            lineAttrTmp.lineId = 3;
            lineAttrTmp.lineStyle  = 5;
            lineAttrInSeg.push_back(lineAttrTmp);

            allLines.push_back(allLinesInSeg);
            lineAttr.push_back(lineAttrInSeg);
        }


        // Check if the vector is closed
/*
        if (checkGpsInRange(&line[0], &line[line.size() - 1], 170.0))  // 170m
        {
            list<vector<point3D_t>>::iterator linesInSegIter = linesInSeg.begin();
            list<lineAttributes_t>::iterator  lineAttrInSegIter = lineAttrInSeg.begin();

            while(linesInSegIter != linesInSeg.end())
            {
                (*linesInSegIter).push_back((*linesInSegIter)[0]); // make the vector closed
                lineAttrInSegIter->numPoints = lineAttrInSegIter->numPoints + 1;
                linesInSegIter++;
                lineAttrInSegIter++;
            }
        }
*/

        // Update database
        database_gp->resetAllVectors(allLines, lineAttr);

        newDataQueues.deleteSpecifiedLaneQueue(laneId);
        database_gp->getAllVectors_clear(allLines, lineAttr);
    }

    return true;
}
