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
*      2015/6/30           Linkun Xu       Create
*******************************************************************************
*/

#include "roadVectorGen.h" // SEGMENT_ID_DEMO
#include "roadVectorGenDashAlign.h"
#include "roadSideVectorGen.h"
#include "LogInfo.h"
#include "icpPointToPoint.h"
#include "database.h" // checkGpsInRange
#include "databaseServer.h" // setNewDataVec

#include <sstream> // for debugging

using std::vector;
using std::queue;
using std::list;
using namespace laneSpace;
using namespace ns_database;

extern databaseServer* database_gp;

void getPaintLandMarkDashAlign(IN list<vector<point3D_t>>& newDbDataVec, 
                               OUT vector<point3D_t>& paintedData, 
                               OUT vector<point3D_t>& landMarkDataStart,
                               OUT vector<point3D_t>& landMarkDataEnd)
{
    const int lmInterval = 15;

    list<vector<point3D_t>>::iterator newDbDataIter = newDbDataVec.begin();

    paintedData.clear();
    landMarkDataStart.clear();
    landMarkDataEnd.clear();

    while(newDbDataIter != newDbDataVec.end())
    {
        int count = lmInterval;
        int dataLen = newDbDataIter->size();

        for(int dataIdx = 0; dataIdx < dataLen; ++dataIdx)
        {
            if( (*newDbDataIter)[dataIdx].paintFlag == 1 )
            {
                paintedData.push_back((*newDbDataIter)[dataIdx]);
            }
            
            if(dataIdx > 0)
            {
                ++count;
                char curFlag = (*newDbDataIter)[dataIdx].paintFlag;
                char preFlag = (*newDbDataIter)[dataIdx-1].paintFlag;
                if( (curFlag == 1) && (preFlag == 0))
                {
                    if(count > lmInterval)
                    {
                        landMarkDataStart.push_back((*newDbDataIter)[dataIdx]);
                    }
                    count = 0;
                }else if( (curFlag == 0) && (preFlag == 1))
                {
                    if(count > lmInterval)
                    {
                        landMarkDataEnd.push_back((*newDbDataIter)[dataIdx-1]);
                    }
                    count = 0;
                }
            }
        }

        ++newDbDataIter;
    }
}

void mergeLineVecDashAlign(IN     vector<point3D_t>       &srcLine, 
                           INOUT  list<vector<point3D_t>> &destLineVec)
{
    double *M;
    point3D_t standPointDummy;
    getLocationArray(srcLine, standPointDummy, &M);
    list<vector<point3D_t>> srcLineVec;
    srcLineVec.push_back(srcLine);

    bool mergedFlag = false;

    list<vector<point3D_t>>::iterator mergedDataIter = destLineVec.begin();
    list<vector<point3D_t>>::iterator sourceDataIter = srcLineVec.begin();

    while(mergedDataIter != destLineVec.end())
    {
        IcpPointToPoint icp(M, sourceDataIter->size(), 2);
        icp.setMaxIterations(0); // set iter number to 0 for only comparing close point

        double *T;
        getLocationArray(*mergedDataIter, standPointDummy, &T);

        Matrix R = Matrix::eye(2);
        Matrix t(2, 1);
        double indist = 50;
        vector<int32_t> activeInNewData;
        vector<int32_t> matchIdxInDB;
        vector<int32_t> closeIdxInDB;
        double minDist;
        double meaDist;

        // compare lines
        icp.fit(T, mergedDataIter->size(), R, t, indist,
                activeInNewData, matchIdxInDB, closeIdxInDB, &minDist, &meaDist);

        if(meaDist < 1)
        {
            vector<int32> ndMatchIdx;
            vector<int32> dbMatchIdx;

            simpleMatchIdx(activeInNewData, 
                    mergedDataIter->size(), 
                    matchIdxInDB, 
                    closeIdxInDB, 
                    sourceDataIter->size(), 
                    ndMatchIdx,
                    dbMatchIdx);

            vector<point3D_t> mergedVec;

            simpleStitch(ndMatchIdx,
                         (*mergedDataIter),
                         dbMatchIdx, 
                         (*sourceDataIter), 
                         mergedVec);

            if(mergedFlag == false)
            {
                sourceDataIter = mergedDataIter;
                ++mergedDataIter;
            }else
            // if mergedFlag is true, do not update sourceDataIter
            {
                mergedDataIter = destLineVec.erase(mergedDataIter);
            }

            (*sourceDataIter) = mergedVec;
            free(M);
            getLocationArray(*sourceDataIter, standPointDummy, &M);

            mergedFlag = true;
        }else
        {
            ++mergedDataIter;
        }

        free(T);
    }

    if(mergedFlag == false)
    {
        destLineVec.push_back(srcLine);
    }

    free(M);
}

bool mergeDashAlign(IN  list<vector<point3D_t>>& newDataVec,
                    IN  list<vector<point3D_t>>& dbDataVec,
                    OUT list<vector<point3D_t>>& mergedDataVec)
{
    bool mergeSuccess = true;

#if 0
    // test code to push all vectors
    mergedDataVec.clear();
    for(list<vector<point3D_t>>::iterator newIter = newDataVec.begin(); newIter != newDataVec.end(); ++newIter)
    {
        mergedDataVec.push_back(*newIter);
    }
    for(list<vector<point3D_t>>::iterator newIter = dbDataVec.begin(); newIter != dbDataVec.end(); ++newIter)
    {
        mergedDataVec.push_back(*newIter);
    }
#else
    point3D_t standPoint = *(dbDataVec.begin()->begin());

    vector<vector<int32_t>> activeInNewDataVec;
    vector<vector<int32_t>> matchIdxInDBVec;
    vector<vector<int32_t>> closeIdxInDBVec;
    vector<double> minDistVec;
    vector<double> meanDistVec;

    int numNdLine = newDataVec.size();
    int numDbLine = dbDataVec.size();

    int numBaseDbLine = min(numDbLine, 3);
    int numBaseNdLine = min(numNdLine, 3);

    // for each db data line
    list<vector<point3D_t>>::iterator dbDataIter = dbDataVec.begin();
    for(int dbDataIdx = 0; dbDataIdx < numBaseDbLine; ++dbDataIdx)
    {
        double *M;
        getLocationArray(*dbDataIter, standPoint, &M);

        IcpPointToPoint icp(M, dbDataIter->size(), 2);
        icp.setMaxIterations(0); // set iter number to 0 for only comparing close point

        // for each new data line
        list<vector<point3D_t>>::iterator newDataIter = newDataVec.begin();
        for(int newDataIdx = 0; newDataIdx < numBaseNdLine; ++newDataIdx)
        {
            double *T;
            getLocationArray(*newDataIter, standPoint, &T);

            Matrix R = Matrix::eye(2);
            Matrix t(2, 1);
            double indist = 100;
            vector<int32_t> activeInNewData;
            vector<int32_t> matchIdxInDB;
            vector<int32_t> closeIdxInDB;
            double minDist;
            double meaDist;

            // compare lines
            icp.fit(T, newDataIter->size(), R, t, indist,
                    activeInNewData, matchIdxInDB, closeIdxInDB, &minDist, &meaDist);
            
            activeInNewDataVec.push_back(activeInNewData);
            matchIdxInDBVec.push_back(matchIdxInDB);
            closeIdxInDBVec.push_back(closeIdxInDB);
            minDistVec.push_back(minDist);
            meanDistVec.push_back(meaDist);

            free(T);
            ++newDataIter;
        }

        free(M);
        ++dbDataIter;
    }

    // find how to match line groups
    vector<double> meanDistArr, meanDistAddCount;

    meanDistArr.assign(numBaseDbLine+numBaseNdLine-1, 100000000.0);
    meanDistAddCount.assign(numBaseDbLine+numBaseNdLine-1, 0.0);
    for(int dbLineIdx = 0; dbLineIdx < numBaseDbLine; ++dbLineIdx)
    {
        for(int ndLineIdx = 0; ndLineIdx < numBaseNdLine; ++ndLineIdx)
        {
            int meanDistIdx = ndLineIdx - dbLineIdx + numBaseDbLine - 1;
            meanDistArr[meanDistIdx] = min(meanDistArr[meanDistIdx], meanDistVec[dbLineIdx*numBaseNdLine+ndLineIdx]);
            meanDistAddCount[meanDistIdx] = meanDistAddCount[meanDistIdx] + 1;
        }
    }
    
    double minMeanVal = 100000000;
    int minMeanIdx = 0;
    for(int idx = 0; idx < meanDistArr.size(); ++idx)
    {
        if(minMeanVal > meanDistArr[idx])
        {
            minMeanVal = meanDistArr[idx];
            minMeanIdx = idx;
        }
    }

    // make sure only two lanes!
    if(numDbLine >= 3 && numNdLine >= 3)
    {
        minMeanIdx = 2;
    }else if(numDbLine >= 3 && numNdLine == 2)
    {
        if(meanDistArr[1] < meanDistArr[2])
            minMeanIdx = 1;
        else
            minMeanIdx = 2;
    }else if(numDbLine == 2 && numNdLine >= 3)
    {
        if(meanDistArr[1] < meanDistArr[2])
            minMeanIdx = 1;
        else
            minMeanIdx = 2;
    }

    // output
    list<vector<point3D_t>> mergedData;
    int dbLineIdx = 0;
    int ndLineIdx = 0;
    dbDataIter = dbDataVec.begin();
    list<vector<point3D_t>>::iterator newDataIter = newDataVec.begin();

    if((minMeanIdx+1) > numBaseDbLine)
    // new data line on the left
    {
        for(ndLineIdx = 0; ndLineIdx < (minMeanIdx-numBaseDbLine+1); ++ndLineIdx)
        {
            mergedData.push_back(*newDataIter++);
        }
    }else if((minMeanIdx+1) < numBaseDbLine)
    // db data line on the left
    {
        for(dbLineIdx = 0; dbLineIdx < (numBaseDbLine-minMeanIdx-1); ++dbLineIdx)
        {
            mergedData.push_back(*dbDataIter++);
        }
    }

    list<vector<point3D_t>> ndUnmergedVec;

    for(int idx = 0; idx < meanDistAddCount[minMeanIdx]; ++idx)
    {
        int index = dbLineIdx*numBaseNdLine+ndLineIdx;
        if(closeIdxInDBVec[index].size() == 0)
        {
            ndUnmergedVec.push_back(*newDataIter);
            mergedData.push_back(*dbDataIter);
        }else
        {
            vector<int32> ndMatchIdx;
            vector<int32> dbMatchIdx;

            simpleMatchIdx(activeInNewDataVec[index], 
                    (*newDataIter).size(), 
                    matchIdxInDBVec[index], 
                    closeIdxInDBVec[index], 
                    (*dbDataIter).size(), 
                    ndMatchIdx,
                    dbMatchIdx);

            vector<point3D_t> mergedVec;

            simpleStitch(ndMatchIdx,
                         (*newDataIter),
                         dbMatchIdx, 
                         (*dbDataIter), 
                         mergedVec);

            mergedData.push_back(mergedVec);

        }

        ++newDataIter;
        ++dbDataIter;
        ++ndLineIdx;
        ++dbLineIdx;
    }

    if(dbLineIdx < numBaseDbLine)
    {
        while(dbLineIdx < numBaseDbLine)
        {
            mergedData.push_back(*dbDataIter++);
            ++dbLineIdx;
        }
    }else if(ndLineIdx < numBaseNdLine)
    {
        while(ndLineIdx < numBaseNdLine)
        {
            mergedData.push_back(*newDataIter++);
            ++ndLineIdx;
        }
    }

    // tail
    while(dbLineIdx < numDbLine)
    {
        mergedData.push_back(*dbDataIter++);
        ++dbLineIdx;
    }

    while(ndLineIdx < numNdLine)
    {
        mergeLineVecDashAlign(*newDataIter, mergedData);
        ++newDataIter;
        ++ndLineIdx;
    }

    // unmerged
    list<vector<point3D_t>>::iterator nuMergedIter = ndUnmergedVec.begin();
    while(nuMergedIter != ndUnmergedVec.end())
    {
        mergedData.push_back(*nuMergedIter++);
    }

    mergedDataVec.clear();
    mergedDataVec = mergedData;
#endif

    return mergeSuccess;
}

bool roadVectorGenDashAlign(laneQueueClass &newDataQueues, vector<int> &storeLane)
{
    const int minPointNum = 5; // should always >= 5

    int laneIdx = 0;
    //for(int laneIdx = 0; laneIdx < storeLane.size(); laneIdx++)
    {
        double* T_landMarkStart;
        double* T_landMarkEnd;
        double* T_painted;

        int newDataLen, newDataLandMarkStartLen, newDataLandMarkEndLen, newDataPaintedLen;
        int dbDataLen, dbDataLandMarkStartLen, dbDataLandMarkEndLen, dbDataPaintedLen;

        list<vector<point3D_t>> newDbDataVec;
        list<vector<point3D_t>> newDataVecForSave;

        point3D_t standPoint;

        // Get new data from input queue
        {
            int laneId = storeLane[laneIdx];
            queue<laneType_t>* queueP = newDataQueues.getSpecifiedLane(laneId);
            newDataLen = queueP->size();

            if(newDataLen < 200)
            // ignore the new data if too short
            {
                newDataQueues.deleteSpecifiedLaneQueue(laneId);
                return false;
            }

            laneType_t newDataTmp;
            point3D_t gpsRef;
            gpsRef.lon = 0; gpsRef.lat = 0;

            vector<point3D_t> newDataL;
            vector<point3D_t> newDataR;
            newDataL.assign(newDataLen, gpsRef);
            newDataR.assign(newDataLen, gpsRef);

            for(int idx = 0; idx < newDataLen; idx++)
            {
                newDataTmp = queueP->front();
                queueP->pop();

                {
                    // change input relative location to GPS location
                    //point3D_t standPointAlgo;
                    //standPointAlgo.lon = -83.213250649943689;
                    //standPointAlgo.lat = 42.296855933108084;

                    newDataL[idx] = newDataTmp.gpsL;
                    newDataR[idx] = newDataTmp.gpsR;

                    newDataL[idx].paintFlag = newDataTmp.linePaintFlagL;
                    newDataR[idx].paintFlag = newDataTmp.linePaintFlagR;
                }
            }

            // Use first new data GPS as standard point
            standPoint = newDataL[0];

            newDbDataVec.push_back(newDataL);
            newDbDataVec.push_back(newDataR);

            newDataVecForSave = newDbDataVec;

            newDataQueues.deleteSpecifiedLaneQueue(laneId);

            // Get painted data and landmark data
            vector<point3D_t> paintedData;
            vector<point3D_t> landMarkDataStart, landMarkDataEnd;
            getPaintLandMarkDashAlign(newDbDataVec, paintedData, landMarkDataStart, landMarkDataEnd);

            if((landMarkDataStart.size() + landMarkDataEnd.size())< 15)
            // ignore the new data if too few landmarks
            {
                newDataQueues.deleteSpecifiedLaneQueue(laneId);
                return false;
            }

            newDataLandMarkStartLen = landMarkDataStart.size();
            newDataLandMarkEndLen = landMarkDataEnd.size();
            newDataPaintedLen = paintedData.size();

            getLocationArray(landMarkDataStart, standPoint, &T_landMarkStart);
            getLocationArray(landMarkDataEnd, standPoint, &T_landMarkEnd);
            getLocationArray(paintedData,  standPoint, &T_painted);

#if (ROAD_VEC_GEN_DEBUG == 1)
            {
                std::stringstream fileName;
                fileName << "log/newData.txt";
                FILE *fpOut = fopen(fileName.str().c_str(), "a");

                for (int idx = 0; idx < newDataLen; idx++)
                {
                    fprintf(fpOut, "%.14f ", newDataL[idx].lat);
                }
                fprintf(fpOut, "\n");
                for (int idx = 0; idx < newDataLen; idx++)
                {
                    fprintf(fpOut, "%.14f ", newDataR[idx].lat);
                }
                fprintf(fpOut, "\n");
                for (int idx = 0; idx < newDataLen; idx++)
                {
                    fprintf(fpOut, "%.14f ", newDataL[idx].lon);
                }
                fprintf(fpOut, "\n");
                for (int idx = 0; idx < newDataLen; idx++)
                {
                    fprintf(fpOut, "%.14f ", newDataR[idx].lon);
                }
                fprintf(fpOut, "\n");
                for (int idx = 0; idx < newDataLen; idx++)
                {
                    fprintf(fpOut, "%d ", newDataL[idx].paintFlag);
                }
                fprintf(fpOut, "\n");
                for (int idx = 0; idx < newDataLen; idx++)
                {
                    fprintf(fpOut, "%d ", newDataR[idx].paintFlag);
                }
                fprintf(fpOut, "\n\n");
                fclose(fpOut);
            }

#endif
        }

        // Save new data to DB
        database_gp->setNewDataVec(newDataVecForSave);

        // Get data points from DB
        list<list<vector<point3D_t>>> allLines;
        list<list<lineAttributes_t>> lineAttr;

        database_gp->getAllVectors(allLines, lineAttr);

        if(allLines.size() != 0)
        {
            list<list<vector<point3D_t>>>::iterator linesInSeg = allLines.begin();
            list<list<lineAttributes_t>>::iterator  attrsInSeg = lineAttr.begin();

            bool firstTimeMerge = true;

            // For each piece of road in DB
            while(linesInSeg != allLines.end())
            {
                bool mergeSuccess = false;

                vector<point3D_t> paintedData;
                vector<point3D_t> landMarkDataStart, landMarkDataEnd;
                getPaintLandMarkDashAlign((*linesInSeg), paintedData, landMarkDataStart, landMarkDataEnd);

                dbDataLandMarkStartLen = landMarkDataStart.size();
                dbDataLandMarkEndLen = landMarkDataEnd.size();
                dbDataPaintedLen = paintedData.size();

                double* M_landMarkStart;
                double* M_landMarkEnd;
                double* M_painted;
                getLocationArray(landMarkDataStart,  standPoint, &M_landMarkStart);
                getLocationArray(landMarkDataEnd,  standPoint, &M_landMarkEnd);
                getLocationArray(paintedData,  standPoint, &M_painted);

                // trying to match land mark points
                Matrix RS = Matrix::eye(2);
                Matrix tS(2, 1);
                double indist = 16;
                std::vector<int32_t> activeInNewDataS;
                std::vector<int32_t> matchIdxInDBS;
                std::vector<int32_t> closeIdxInDBS;
                double minDistS;
                double meaDistS;

                IcpPointToPoint icpS(M_landMarkStart, dbDataLandMarkStartLen, 2);
                icpS.setMaxIterations(ICP_MAX_ITER);
                icpS.fit(T_landMarkStart, newDataLandMarkStartLen, RS, tS, indist,
                        activeInNewDataS, matchIdxInDBS, closeIdxInDBS, &minDistS, &meaDistS);

                Matrix RE = Matrix::eye(2);
                Matrix tE(2, 1);
                std::vector<int32_t> activeInNewDataE;
                std::vector<int32_t> matchIdxInDBE;
                std::vector<int32_t> closeIdxInDBE;
                double minDistE;
                double meaDistE;

                IcpPointToPoint icpE(M_landMarkEnd, dbDataLandMarkEndLen, 2);
                icpE.setMaxIterations(ICP_MAX_ITER);
                icpE.fit(T_landMarkEnd, newDataLandMarkEndLen, RE, tE, indist,
                        activeInNewDataE, matchIdxInDBE, closeIdxInDBE, &minDistE, &meaDistE);

                if((activeInNewDataS.size() >= 2) && (activeInNewDataE.size() >= 2))
                {
                    double tS_x = tS.val[0][0];
                    double tS_y = tS.val[1][0];
                    double tE_x = tE.val[0][0];
                    double tE_y = tE.val[1][0];

                    // Check start and end shift matrix
                    if((abs(tE.val[0][0] - tS.val[0][0]) > 2) || (abs(tE.val[1][0] - tS.val[1][0]) > 2))
                    {
                        free(M_landMarkStart);
                        free(M_landMarkEnd);
                        free(M_painted);
                        free(T_landMarkStart);
                        free(T_landMarkEnd);
                        free(T_painted);
                        return false;
                    }

                    // trying to match painted points
                    indist = 2;
                    std::vector<int32_t> activeInNewData;
                    std::vector<int32_t> matchIdxInDB;
                    std::vector<int32_t> closeIdxInDB;
                    double minDist;
                    double meaDist;

                    IcpPointToPoint icp(M_painted, dbDataPaintedLen, 2);
                    icp.setMaxIterations(ICP_MAX_ITER);
                    icp.fit(T_painted, newDataPaintedLen, RE, tE, indist,
                            activeInNewData, matchIdxInDB, closeIdxInDB, &minDist, &meaDist);

                    if(activeInNewData.size() >= 20)
                    {
                        // Pan both DB data and new data
                        double newDataMoveCoeff;
                        if(firstTimeMerge == true)
                        // new data with DB data
                        {
                            newDataMoveCoeff = ((double)newDataPaintedLen) / ((double)(dbDataPaintedLen+newDataPaintedLen)) / 5.0;
                        }else
                        // DB data with DB data
                        {
                            newDataMoveCoeff = ((double)newDataPaintedLen) / ((double)(dbDataPaintedLen+newDataPaintedLen));
                        }

                        list<vector<point3D_t>> newDataPanVec;
                        for(list<vector<point3D_t>>::iterator newIter = newDbDataVec.begin(); newIter != newDbDataVec.end(); ++newIter)
                        {
                            vector<point3D_t> lineTmp = (*newIter);
                            for(int pointIdx = 0; pointIdx < newIter->size(); ++pointIdx)
                            {
                                lineTmp[pointIdx].lon = (*newIter)[pointIdx].lon + (1-newDataMoveCoeff) * tE.val[0][0];
                                lineTmp[pointIdx].lat = (*newIter)[pointIdx].lat + (1-newDataMoveCoeff) * tE.val[1][0];
                            }
                            newDataPanVec.push_back(lineTmp);
                        }
                        list<vector<point3D_t>> dbDataPanVec;
                        for(list<vector<point3D_t>>::iterator newIter = linesInSeg->begin(); newIter != linesInSeg->end(); ++newIter)
                        {
                            vector<point3D_t> lineTmp = (*newIter);
                            for(int pointIdx = 0; pointIdx < newIter->size(); ++pointIdx)
                            {
                                lineTmp[pointIdx].lon = (*newIter)[pointIdx].lon - newDataMoveCoeff * tE.val[0][0];
                                lineTmp[pointIdx].lat = (*newIter)[pointIdx].lat - newDataMoveCoeff * tE.val[1][0];
                            }
                            dbDataPanVec.push_back(lineTmp);
                        }

                        // Merge data
                        mergeSuccess = mergeDashAlign(newDataPanVec, dbDataPanVec, newDbDataVec);

                        firstTimeMerge = firstTimeMerge || mergeSuccess;

                        // Update T
                        {
                            free(T_landMarkStart);
                            free(T_landMarkEnd);
                            free(T_painted);

                            vector<point3D_t> paintedData;
                            vector<point3D_t> landMarkDataStart;
                            vector<point3D_t> landMarkDataEnd;
                            getPaintLandMarkDashAlign(newDbDataVec, paintedData, landMarkDataStart, landMarkDataEnd);

                            newDataLandMarkStartLen = landMarkDataStart.size();
                            newDataLandMarkEndLen = landMarkDataEnd.size();
                            newDataPaintedLen = paintedData.size();

                            getLocationArray(landMarkDataStart, standPoint, &T_landMarkStart);
                            getLocationArray(landMarkDataEnd, standPoint, &T_landMarkEnd);
                            getLocationArray(paintedData,  standPoint, &T_painted);
                        }
                    }
                }

                // Free DB
                if(mergeSuccess)
                {
                    linesInSeg = allLines.erase(linesInSeg);
                    attrsInSeg = lineAttr.erase(attrsInSeg);
                }else
                {
                    ++linesInSeg;
                    ++attrsInSeg;
                }

                free(M_landMarkStart);
                free(M_landMarkEnd);
                free(M_painted);
            }
        }

        // Add new data to database
#if (ROAD_VEC_GEN_DEBUG == 1)
        {
            std::ostringstream ostr;
            ostr << "+++ Add Number Lines: " << newDbDataVec.size();
            logPrintf(logLevelNotice_e, "ROAD_VEC_GEN", ostr.str(), FOREGROUND_RED);
        }
#endif

        {
            lineAttributes_t lineAttrTmp;
            {
                lineAttrTmp.segmentId = SEGMENT_ID_DEMO; // FIXME: only one segment exist for demo
                lineAttrTmp.lineId = 1;
                lineAttrTmp.width  = 0.1;
                lineAttrTmp.lineStyle  = 5; // road side
                lineAttrTmp.segVersion = 0;
                lineAttrTmp.numPoints  = 0;
            }

            {
                list<lineAttributes_t> lineAttrInSeg;

                list<vector<point3D_t>>::iterator newDbDataIter = newDbDataVec.begin();
                for(int idx = 0; idx < newDbDataVec.size(); ++idx)
                {
                    lineAttrTmp.numPoints  = newDbDataIter->size();
                    ++newDbDataIter;

                    lineAttrInSeg.push_back(lineAttrTmp);
                }

                allLines.push_back(newDbDataVec);    
                lineAttr.push_back(lineAttrInSeg);
            }

            // Update database
            database_gp->resetAllVectors(allLines, lineAttr);
            database_gp->getAllVectors_clear(allLines, lineAttr);
        }

        free(T_landMarkStart);
        free(T_landMarkEnd);
        free(T_painted);
    }

    return true;
}
