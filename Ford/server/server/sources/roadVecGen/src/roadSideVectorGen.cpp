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

#include <math.h>
#include "roadSideVectorGen.h"
#include "database.h" // tlvCommon_t
#include "LogInfo.h"

using namespace ns_database;

void roadSideVectorGen(IN  vector<point3D_t>& line, 
                       IN  vector<double>& distanceInMeterR, 
                       IN  vector<double>& distanceInMeterL, 
                       OUT vector<point3D_t>& lineR, 
                       OUT vector<point3D_t>& lineL)
{
    point3D_t point3D;
    lineR.assign(line.size(), point3D);
    lineL.assign(line.size(), point3D);

    int lineIndex = 0;
    int lineLen = line.size();
    point3D_t point1, point2;
    double rou, cosAlpha, sinAlpha;
    double lonDis, latDis;
    double latitude, coeffDd2MeterLon;

    // deal with lineLen-1 point
    for(lineIndex=0; lineIndex<lineLen-1; lineIndex++)
    {
        point1 = line[lineIndex];
        point2 = line[lineIndex + 1];

        double distR = distanceInMeterR[lineIndex];
        double distL = distanceInMeterL[lineIndex];

        latitude = (point1.lat)*PI/180;
        coeffDd2MeterLon = (111413*cos(latitude)-94*cos(3*latitude));

        lonDis = (point1.lon - point2.lon) * coeffDd2MeterLon;
        latDis = (point1.lat - point2.lat) * COEFF_DD2METER;
        
        rou = sqrt((lonDis * lonDis + latDis * latDis));
        
        cosAlpha = latDis / rou;
        
        sinAlpha = lonDis / rou;
        
        lineR[lineIndex].lat = point1.lat + (distR)*(sinAlpha) / COEFF_DD2METER;
        lineL[lineIndex].lat = point1.lat - (distL)*(sinAlpha) / COEFF_DD2METER;
        lineR[lineIndex].lon = point1.lon - (distR)*(cosAlpha) / coeffDd2MeterLon;
        lineL[lineIndex].lon = point1.lon + (distL)*(cosAlpha) / coeffDd2MeterLon;

        lineR[lineIndex].alt = 0;
        lineL[lineIndex].alt = 0;
    }

    //deal with last point
    double distR = distanceInMeterR[lineLen-1];
    double distL = distanceInMeterL[lineLen-1];
    point1 = line[lineIndex];
    lineR[lineIndex].lat = point1.lat + (distR)*(sinAlpha) / COEFF_DD2METER;
    lineR[lineIndex].lon = point1.lon - (distR)*(cosAlpha) / coeffDd2MeterLon;    
    lineR[lineIndex].alt = 0; 
    lineL[lineIndex].lat = point1.lat - (distL)*(sinAlpha) / COEFF_DD2METER;
    lineL[lineIndex].lon = point1.lon + (distL)*(cosAlpha) / coeffDd2MeterLon;
    lineL[lineIndex].alt = 0; 
}

void roadSideDistGen(IN  vector<laneType_t>& dataVec, 
                     OUT vector<point3D_t>& line,
                     OUT vector<double>& distR, 
                     OUT vector<double>& distL)
{
    point3D_t point3D;
    int newDataLen = dataVec.size();
    
    line.assign(newDataLen, point3D);

    double tmpDouble = 0.0;
    distR.assign(newDataLen, tmpDouble);
    distL.assign(newDataLen, tmpDouble);

    for(int idx = 0; idx < newDataLen; idx++)
    {
        laneType_t laneGps = dataVec[idx];

        if(laneGps.laneId == 0) // 0: left, 1: right
        {
            line[idx] = laneGps.gpsR;
            distR[idx] = 0;
            distL[idx] = laneGps.laneWidth;
        }else if(laneGps.laneId == 1)
        {
            line[idx] = laneGps.gpsL;
            distR[idx] = laneGps.laneWidth;
            distL[idx] = 0;
        }
    }



    // calculate the dist in db

    // calculate the dist of new data


    // update dist

}

void simpleMatchIdx(IN std::vector<int32> &activeInNewData, 
                    IN int newDataLen, 
                    IN std::vector<int32> &matchIdxInDB, 
                    IN std::vector<int32> &closeIdxInDB, 
                    IN int dbDataLen, 
                    OUT std::vector<int32> &outputNewDataIdx,
                    OUT std::vector<int32> &outputDBIdx)
{
    outputNewDataIdx.clear();
    outputDBIdx.clear();

    const int startThresh = 7;
    const int endThresh   = dbDataLen - 8;

    // Find begin and end index in matchIdxInDB which is not -1
    int indexBegin = 0;
    int indexEnd = matchIdxInDB.size() - 1;

    while(matchIdxInDB[indexBegin] == -1)
    {
        ++indexBegin;
    }

    while(matchIdxInDB[indexEnd] == -1)
    {
        --indexEnd;
    }

    // Check if a close loop or not
    bool isCloseLoop = false;
    int maxIdx, minIdx;
    int maxVal = 0, minVal = 10000000;

    for(int i = 0; i < matchIdxInDB.size(); ++i)
    {
        if(matchIdxInDB[i] > maxVal)
        {
            maxVal = matchIdxInDB[i];
            maxIdx = i;
        }

        if((matchIdxInDB[i] >= 0) && (matchIdxInDB[i] < minVal))
        {
            minVal = matchIdxInDB[i];
            minIdx = i;
        }
    }

    /*
    if( (dbDataLen > 1000) && 
        (minVal < startThresh) &&
        (maxVal > endThresh))
    {
        isCloseLoop = true;
    }*/

    if(isCloseLoop == true)
    {
        if(maxIdx < minIdx)
        // tail in front of head
        {
            // head
            for(int i = minIdx; i <= indexEnd; ++i)
            {
                outputNewDataIdx.push_back(activeInNewData[i]);
                outputDBIdx.push_back(closeIdxInDB[i]);
            }

            // unchanged DB
            for(int i = closeIdxInDB[indexEnd]+1; i < closeIdxInDB[indexBegin]; ++i)
            {
                outputNewDataIdx.push_back(-1);
                outputDBIdx.push_back(i);
            }

            // tail
            for(int i = indexBegin; i <= maxIdx; ++i)
            {
                outputNewDataIdx.push_back(activeInNewData[i]);
                outputDBIdx.push_back(closeIdxInDB[i]);
            }

            // new points
            for(int i = activeInNewData[maxIdx]+1; i < activeInNewData[minIdx]; ++i)
            {
                outputNewDataIdx.push_back(i);
                outputDBIdx.push_back(-1);
            }

            // close the loop
            outputNewDataIdx.push_back(outputNewDataIdx[0]);
            outputDBIdx.push_back(outputDBIdx[0]);
        }else
        // head in front of tail
        {
            // Do not support, keep data in DB
            logPrintf(logLevelNotice_e, "ROAD_VEC_GEN", "Close loop: head in front of tail", FOREGROUND_RED);

            for(int i = 0; i < dbDataLen; ++i)
            {
                outputNewDataIdx.push_back(-1);
                outputDBIdx.push_back(i);
            }
        }
    }else
    {
        if(closeIdxInDB[indexBegin] < startThresh)
        // start with new data
        {
            for(int i = 0; i < activeInNewData[indexBegin]; ++i)
            {
                outputNewDataIdx.push_back(i);
                outputDBIdx.push_back(-1);
            }
        }else
        // keep data in DB
        {
            for(int i = 0; i < closeIdxInDB[indexBegin]; ++i)
            {
                outputNewDataIdx.push_back(-1);
                outputDBIdx.push_back(i);
            }
        }

        // indexs for merge
        for(int i = indexBegin; i <= indexEnd; ++i)
        {
            outputNewDataIdx.push_back(activeInNewData[i]);
            outputDBIdx.push_back(closeIdxInDB[i]);
        }

        // indexs for the tail
        if(closeIdxInDB[indexEnd] > endThresh)
        // tail from new data
        {
            for(int i = activeInNewData[indexEnd]+1; i < newDataLen; ++i)
            {
                outputNewDataIdx.push_back(i);
                outputDBIdx.push_back(-1);
            }
        }else
        // tail from DB
        {
            for(int i = closeIdxInDB[indexEnd]+1; i < dbDataLen; ++i)
            {
                outputNewDataIdx.push_back(-1);
                outputDBIdx.push_back(i);
            }
        }
    }
}

void simpleStitch(IN  vector<int32>    &newDataIdxVec,
                  IN  vector<point3D_t> &newDataVec,
                  IN  vector<int32>    &dbDataIdxVec, 
                  IN  vector<point3D_t> &dbDataVec, 
                  OUT vector<point3D_t> &mergedVec)
{
    point3D_t gpsTemp;
    mergedVec.clear();
    mergedVec.assign(newDataIdxVec.size(), gpsTemp);

    for(int i = 0; i < newDataIdxVec.size(); ++i)
    {
        if(newDataIdxVec[i] == -1)
        // data from DB
        {
            mergedVec[i] = dbDataVec[dbDataIdxVec[i]];
        }else if(dbDataIdxVec[i] == -1)
        // data from new data
        {
            mergedVec[i] = newDataVec[newDataIdxVec[i]];
        }else
        // data from both
        {
            point3D_t ndPoint = newDataVec[newDataIdxVec[i]];
            point3D_t dbPoint = dbDataVec[dbDataIdxVec[i]];

            point3D_t mergedGps;
            mergedGps.lon = (ndPoint.lon + dbPoint.lon) / 2;
            mergedGps.lat = (ndPoint.lat + dbPoint.lat) / 2;
            mergedGps.alt = 0;
            mergedGps.paintFlag = (ndPoint.paintFlag + dbPoint.paintFlag) / 2;

            mergedVec[i] = mergedGps;
        }
    }
}


void stitchLineWithIdx(IN    vector<int32>    &newDataIdxVec, 
                       INOUT vector<point3D_t>  &newDbDataVec,
                       INOUT vector<double>     &distRVec,
                       INOUT vector<double>     &distLVec,
                       IN    vector<int32>    &dbDataIdxVec, 
                       IN    vector<point3D_t>  &linesInSegVec,
                       IN    vector<point3D_t>  &linesRInSegVec,
                       IN    vector<point3D_t>  &linesLInSegVec)
{
    vector<point3D_t> outDbDataVec;
    vector<double>    outDistRVec;
    vector<double>    outDistLVec;

    point3D_t gpsTemp;
    double    distTemp;

    outDbDataVec.assign(newDataIdxVec.size(), gpsTemp);
    outDistRVec.assign(newDataIdxVec.size(), distTemp);
    outDistLVec.assign(newDataIdxVec.size(), distTemp);

    for(int i = 0; i < newDataIdxVec.size(); ++i)
    {
        if(newDataIdxVec[i] == -1)
        // data from DB
        {
            outDbDataVec[i] = linesInSegVec[dbDataIdxVec[i]];

            calcRelDistance(&linesInSegVec[dbDataIdxVec[i]], &linesRInSegVec[dbDataIdxVec[i]], &distTemp);
            outDistRVec[i] = distTemp;
            calcRelDistance(&linesInSegVec[dbDataIdxVec[i]], &linesLInSegVec[dbDataIdxVec[i]], &distTemp);
            outDistLVec[i] = distTemp;
        }else if(dbDataIdxVec[i] == -1)
        // data from new data
        {
            outDbDataVec[i] = newDbDataVec[newDataIdxVec[i]];
            outDistRVec[i]  = distRVec[newDataIdxVec[i]];
            outDistLVec[i]  = distLVec[newDataIdxVec[i]];
        }else
        // data from both
        {
            outDbDataVec[i].lat = (newDbDataVec[newDataIdxVec[i]].lat + (STITCH_WEIGHT - 1) * linesInSegVec[dbDataIdxVec[i]].lat) / STITCH_WEIGHT;
            outDbDataVec[i].lon = (newDbDataVec[newDataIdxVec[i]].lon + (STITCH_WEIGHT - 1) * linesInSegVec[dbDataIdxVec[i]].lon) / STITCH_WEIGHT;
            outDbDataVec[i].alt = (newDbDataVec[newDataIdxVec[i]].alt + (STITCH_WEIGHT - 1) * linesInSegVec[dbDataIdxVec[i]].alt) / STITCH_WEIGHT;

            calcRelDistance(&linesInSegVec[dbDataIdxVec[i]], &linesRInSegVec[dbDataIdxVec[i]], &distTemp);

            if(distRVec[newDataIdxVec[i]] == 0)
                outDistRVec[i] = distTemp;
            else if(distTemp == 0)
                outDistRVec[i] = distRVec[newDataIdxVec[i]];
            else
                outDistRVec[i] = (distRVec[newDataIdxVec[i]] + (STITCH_WEIGHT - 1) * distTemp) / STITCH_WEIGHT;

            calcRelDistance(&linesInSegVec[dbDataIdxVec[i]], &linesLInSegVec[dbDataIdxVec[i]], &distTemp);

            if(distLVec[newDataIdxVec[i]] == 0)
                outDistLVec[i] = distTemp;
            else if(distTemp == 0)
                outDistLVec[i] = distLVec[newDataIdxVec[i]];
            else
                outDistLVec[i] = (distLVec[newDataIdxVec[i]] + (STITCH_WEIGHT - 1) * distTemp) / STITCH_WEIGHT;
        }
    }

    newDbDataVec = outDbDataVec;
    distRVec = outDistRVec;
    distLVec = outDistLVec;
}

void getLocationArray(IN  vector<point3D_t>& dataVec, 
                      IN  point3D_t& standPoint,
                      OUT double **T)
{
    *T = (double*)calloc(2 * dataVec.size(),sizeof(double)); // 2 for lat/lon
    for(int idx = 0; idx < dataVec.size(); ++idx)
    {
        //pointRelative3D_t pointRel;
        //calcRelativeLocation(&standPoint, &dataVec[idx], &pointRel);

        // input dataVec is relative location
        (*T)[2*idx]   = dataVec[idx].lon;
        (*T)[2*idx+1] = dataVec[idx].lat;
    }
}
