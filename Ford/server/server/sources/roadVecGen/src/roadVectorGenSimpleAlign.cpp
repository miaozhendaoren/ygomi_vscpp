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
#include "roadVectorGenSimpleAlign.h"
#include "roadSideVectorGen.h"
#include "LogInfo.h"
#include "icpPointToPoint.h"
#include "database.h" // checkGpsInRange

#include <sstream> // for debugging

using std::vector;
using std::queue;
using std::list;
using namespace laneSpace;
using namespace ns_database;

extern databaseServer* database_gp;

bool roadVectorGenSimpleAlign(laneQueueClass &newDataQueues, vector<int> &storeLane)
{
    const int minPointNum = 5; // should always >= 5

    int laneIdx = 0;
    //for(int laneIdx = 0; laneIdx < storeLane.size(); laneIdx++)
    {
        double* T;

        int newDataLen;
        int dbDataLen;

        vector<point3D_t> newDbDataVec;
        vector<double> distRVec;
        vector<double> distLVec;

        point3D_t standPoint;

        // Get new data from input queue
        {
            vector<laneType_t> newDataVec;

            int laneId = storeLane[laneIdx];
            queue<laneType_t>* queueP = newDataQueues.getSpecifiedLane(laneId);
            newDataLen = queueP->size();

            T = (double*)calloc(2*newDataLen,sizeof(double)); // 2 for lat/lon

            laneType_t newDataTmp;
            newDataVec.assign(newDataLen, newDataTmp);
            int newDataIdx = 0;
            point3D_t gpsRef, preGpsRef;
            gpsRef.lon = 0; gpsRef.lat = 0;
            for(int idx = 0; idx < newDataLen; idx++)
            {
                newDataTmp = queueP->front();
                queueP->pop();

                // Get GPS on reference line according to lane ID
                preGpsRef = gpsRef;

                if(newDataTmp.laneId == 0)
                {
                    gpsRef = newDataTmp.gpsR;
                }else
                {
                    gpsRef = newDataTmp.gpsL;
                }

                if(idx == 0)
                // Use first new data GPS as standard point
                {
                    if(newDataTmp.laneId == 0)
                    {
                        standPoint = gpsRef;
                    }else
                    {
                        standPoint = gpsRef;
                    }
                }

                if(newDataIdx > 0 && checkGpsInRange(&gpsRef, &preGpsRef, 1))
                // ignore too close points
                {
                    continue;
                }else
                {
                    newDataVec[newDataIdx] = newDataTmp;

                    pointRelative3D_t pointRel;
                    calcRelativeLocation(&standPoint, &gpsRef, &pointRel);

                    T[2*newDataIdx]   = (double)pointRel.x;
                    T[2*newDataIdx+1] = (double)pointRel.y;
                }

                ++newDataIdx;
            }
            newDataLen = newDataIdx;
            newDataVec.resize(newDataLen);

            roadSideDistGen(newDataVec, newDbDataVec, distRVec, distLVec);

            newDataQueues.deleteSpecifiedLaneQueue(laneId);

            if(newDataLen < minPointNum)
            // ignore the new data if too short
            {
                free(T);
                return false;
            }

#if (ROAD_VEC_GEN_DEBUG == 1)
            {
                std::stringstream fileName;
                fileName << "log/newData.txt";
                FILE *fpOut = fopen(fileName.str().c_str(), "a");

                fprintf(fpOut, "%s", "latL = [");
                for (int idx = 0; idx < newDataVec.size(); idx++)
                {
                    fprintf(fpOut, "%.14f,", newDataVec[idx].gpsL.lat);
                }
                fprintf(fpOut, "];\n%s", "latR = [");
                for (int idx = 0; idx < newDataVec.size(); idx++)
                {
                    fprintf(fpOut, "%.14f,", newDataVec[idx].gpsR.lat);
                }
                fprintf(fpOut, "];\n%s", "lonL = [");
                for (int idx = 0; idx < newDataVec.size(); idx++)
                {
                    fprintf(fpOut, "%.14f,", newDataVec[idx].gpsL.lon);
                }
                fprintf(fpOut, "];\n%s", "lonR = [");
                for (int idx = 0; idx < newDataVec.size(); idx++)
                {
                    fprintf(fpOut, "%.14f,", newDataVec[idx].gpsR.lon);
                }
                fprintf(fpOut, "];\n%s", "linePaintFlagL = [");
                for (int idx = 0; idx < newDataVec.size(); idx++)
                {
                    fprintf(fpOut, "%d,", newDataVec[idx].linePaintFlagL);
                }
                fprintf(fpOut, "];\n%s", "linePaintFlagR = [");
                for (int idx = 0; idx < newDataVec.size(); idx++)
                {
                    fprintf(fpOut, "%d,", newDataVec[idx].linePaintFlagR);
                }
                fprintf(fpOut, "];\n\n");
                fclose(fpOut);
            }

#endif
        }

        // Get data points from DB
        list<list<vector<point3D_t>>> allLines;
        list<list<lineAttributes_t>> lineAttr;

        database_gp->getAllVectors(allLines, lineAttr);

        bool isMatchedFlag = false;

        if(allLines.size() != 0)
        {
            list<vector<point3D_t>>::iterator linesInSeg = allLines.begin()->begin(); // FIXME: only one segment
            list<lineAttributes_t>::iterator  attrsInSeg = lineAttr.begin()->begin();

            // For each piece of road in DB
            while(linesInSeg != allLines.begin()->end())
            {
                if(attrsInSeg->lineStyle == 0) // reference line
                {
                    list<vector<point3D_t>>::iterator refLineInSeg = linesInSeg++;
                    list<vector<point3D_t>>::iterator RLineInSeg = linesInSeg++;
                    list<vector<point3D_t>>::iterator LLineInSeg = linesInSeg++;
                    list<lineAttributes_t>::iterator  refAttrsInSeg = attrsInSeg++;
                    list<lineAttributes_t>::iterator  RAttrsInSeg = attrsInSeg++;
                    list<lineAttributes_t>::iterator  LAttrsInSeg = attrsInSeg++;

                    dbDataLen = (*refLineInSeg).size();

                    if(dbDataLen != refAttrsInSeg->numPoints)
                    {
                        logPrintf(logLevelNotice_e, "ROAD_VEC_GEN", "number points do not match!", FOREGROUND_RED);
                    }

                    double* M = (double*)calloc(2*dbDataLen,sizeof(double));

                    for(int idx = 0; idx < dbDataLen; idx++)
                    {
                        pointRelative3D_t pointRel;
                        calcRelativeLocation(&standPoint, &((*refLineInSeg)[idx]), &pointRel);

                        M[2*idx]   = (double)pointRel.x;
                        M[2*idx+1] = (double)pointRel.y;
                    }

                    // trying to match
                    Matrix R = Matrix::eye(2);
                    Matrix t(2, 1);
                    double indist = 30;
                    std::vector<int32_t> activeInNewData;
                    std::vector<int32_t> matchIdxInDB;
                    std::vector<int32_t> closeIdxInDB;
                    double minDist;
                    double meaDist;

                    IcpPointToPoint icp(M, dbDataLen, 2);
                    icp.setMaxIterations(ICP_MAX_ITER);
                    icp.fit(T, newDataLen, R, t, indist,
                            activeInNewData, matchIdxInDB, closeIdxInDB, &minDist, &meaDist);

                    if(activeInNewData.size() > minPointNum)
                    {
                        isMatchedFlag = true;

                        // Calculate the indexes to stitch the lane
                        std::vector<int32_t> newDataIdxVec, dbDataIdxVec;
                        simpleMatchIdx(activeInNewData, newDataLen, matchIdxInDB, closeIdxInDB, dbDataLen, newDataIdxVec, dbDataIdxVec);

#if (ROAD_VEC_GEN_DEBUG == 1)
                        {
                            int fittedLen = newDataIdxVec.size();
                            std::ostringstream ostr;
                            ostr << "newDataLen: " << newDataLen << ", dbDataLen: " << dbDataLen << ", fittedLen: " << fittedLen;
                            logPrintf(logLevelNotice_e, "ROAD_VEC_GEN", ostr.str(), FOREGROUND_RED);

                            if((fittedLen < dbDataLen) || (fittedLen < newDataLen))
                            {
                                int stop = 1;
                            }

                            if(newDataIdxVec.size() > 3000)
                            {
                                int stop = 1;
                            }

                            {
                                std::stringstream fileName;
                                fileName << "log/roadVectorGenSimpleAlign.txt";
                                FILE *fpOut = fopen(fileName.str().c_str(), "a");

                                fprintf(fpOut, "%s", ostr.str().c_str());
                                fprintf(fpOut, "\n%s", "newIdx: ");
                                for (int idx = 0; idx < activeInNewData.size(); idx++)
                                {
                                    fprintf(fpOut, "%4d,", activeInNewData[idx]);
                                }
                                fprintf(fpOut, "\n%s", "dBIdx:  ");
                                for (int idx = 0; idx < matchIdxInDB.size(); idx++)
                                {
                                    fprintf(fpOut, "%4d,", matchIdxInDB[idx]);
                                }
                                fprintf(fpOut, "\n%s", "fitNewIdx: ");
                                for (int idx = 0; idx < newDataIdxVec.size(); idx++)
                                {
                                    fprintf(fpOut, "%4d,", newDataIdxVec[idx]);
                                }
                                fprintf(fpOut, "\n%s", "fitDBIdx:  ");
                                for (int idx = 0; idx < dbDataIdxVec.size(); idx++)
                                {
                                    fprintf(fpOut, "%4d,", dbDataIdxVec[idx]);
                                }
                                fprintf(fpOut, "\n\n");
                                fclose(fpOut);
                            }
                        }
#endif

                        // Stitch reference lane with lane width
                        stitchLineWithIdx(newDataIdxVec, newDbDataVec, distRVec, distLVec, dbDataIdxVec, *refLineInSeg, *RLineInSeg, *LLineInSeg);

                        // Update newDataLen and T
                        {
                            newDataLen = newDbDataVec.size();

                            free(T);
                            T = (double*)calloc(2*newDataLen,sizeof(double)); // 2 for lat/lon

                            int newDataIdx = 0;
                            
                            for(int idx = 0; idx < newDataLen; idx++)
                            {
                                //if(newDataIdx > 0 && checkGpsInRange(&newDbDataVec[idx], &newDbDataVec[newDataIdx-1], 1))
                                // ignore too close points
                                //{
                                //    continue;
                                //}else
                                {
                                    pointRelative3D_t pointRel;
                                    calcRelativeLocation(&standPoint, &(newDbDataVec[idx]), &pointRel);

                                    T[2*newDataIdx]   = (double)pointRel.x;
                                    T[2*newDataIdx+1] = (double)pointRel.y;
                                }

                                ++newDataIdx;
                            }
                            //newDataLen = newDataIdx;
                            //newDbDataVec.resize(newDataLen);
                        }

#if (ROAD_VEC_GEN_DEBUG == 1)
                        {
                            std::ostringstream ostr;
                            ostr << "--- Erase Number Point: " << refLineInSeg->size();
                            logPrintf(logLevelNotice_e, "ROAD_VEC_GEN", ostr.str(), FOREGROUND_RED);
                        }
#endif

                        // Free DB
                        linesInSeg = allLines.begin()->erase(refLineInSeg, linesInSeg);
                        attrsInSeg = lineAttr.begin()->erase(refAttrsInSeg, attrsInSeg);
                    }

                    free(M);
                }
            }
        }

#if (ROAD_VEC_GEN_DEBUG == 1)
        {
            std::ostringstream ostr;
            ostr << "+++ Add Number Point: " << newDbDataVec.size();
            logPrintf(logLevelNotice_e, "ROAD_VEC_GEN", ostr.str(), FOREGROUND_RED);
        }
#endif

        //if(!isMatchedFlag)
        // push new data to DB
        {
            lineAttributes_t lineAttrTmp;
            {
                lineAttrTmp.segmentId = SEGMENT_ID_DEMO; // FIXME: only one segment exist for demo
                lineAttrTmp.lineId = 1;
                lineAttrTmp.width  = 0.1;
                lineAttrTmp.lineStyle  = 0; // reference line
                lineAttrTmp.segVersion = 0;
                lineAttrTmp.numPoints  = newDbDataVec.size();
            }

            // Generate road vectors with road sides
            vector<point3D_t> lineR, lineL;

            roadSideVectorGen(newDbDataVec, distRVec, distLVec, lineR, lineL);

            if (allLines.size() != 0)
            {
                allLines.begin()->push_back(newDbDataVec);	
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

                allLinesInSeg.push_back(newDbDataVec);	
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
            database_gp->getAllVectors_clear(allLines, lineAttr);
        }

        free(T);
    }

    return true;
}
