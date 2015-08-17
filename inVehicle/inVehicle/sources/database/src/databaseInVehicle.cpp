/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  databaseMath.cpp
* @brief Math functions for database
*
* Change Log:
*      Date                Who             What
*      2015/1/13         Linkun Xu        Create
*******************************************************************************
*/

#include "database.h"
#include "databaseInVehicle.h"

#include <stdio.h>    // FILE
#include <windows.h>  // CreateMutex, FOREGROUND_RED

#include "LogInfo.h"  // logPrintf

using std::list;
using std::vector;
using std::string;

namespace ns_database
{
    databaseInVehicle::databaseInVehicle(string inFileStr) : database(inFileStr)
    {}

    databaseInVehicle::databaseInVehicle() : database()
    {}

    databaseInVehicle::~databaseInVehicle() // FIXME
    {
        _furnitureList.clear();
    }

    void databaseInVehicle::initDb()
    {
        database::initDb();

        _furnitureList.clear();
    }

    void databaseInVehicle::getFurnitureByGps(IN  point3D_t*  gpsP, 
                                     IN  double distThresh,
                                     OUT list<furAttributesInVehicle_t>& furnitureAttrList)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        uint8 segExistFlag;
        segAttributes_t segAttr;

        getSegmentByGps(gpsP, &segExistFlag, &segAttr);

        if(segExistFlag == 1)
        {
            uint8 furExistFlag = 0;

            list<list<furAttributesInVehicle_t>>::iterator segIter = _furnitureList.begin();
            
            // For each segment
            while(segIter != _furnitureList.end())
            {
                // Find the segment containing the furniture
                uint8  segId_used = (*(*segIter).begin()).segId_used;
                uint32 segIdTmp   = (*(*segIter).begin()).segId;

                // TODO: Check all segments now
                //if((segId_used != 1) || (segAttr.segId != segIdTmp))
                //{
                //    segIter++;
                //    continue;
                //}

                list<furAttributesInVehicle_t>::iterator furIter = (*segIter).begin();

                // For each furniture in the segment
                while(furIter != (*segIter).end())
                {
                    
                    if((*furIter).location_used == 1)
                    {
                        point3D_t gpsTmp = (*furIter).location;

                        // Push furniture to output list if in GPS range
                        if(checkGpsInRange(&gpsTmp, gpsP, distThresh))
                        {
                            furnitureAttrList.push_back(*furIter);
                        }
                    }

                    furIter++;
                }

                segIter++;
            }
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseInVehicle::getFurnitureById(IN  uint32 segmentIdIn, 
                                    IN  uint32 furnitureIdIn,
                                    OUT uint8* existFlag, 
                                    OUT furAttributesInVehicle_t* furnitureAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        *existFlag = 0;

        list<list<furAttributesInVehicle_t>>::iterator segIter = _furnitureList.begin();

        while(segIter != _furnitureList.end())
        {
            list<furAttributesInVehicle_t>::iterator furIter = (*segIter).begin();

            if(((*furIter).segId_used == 0) || ((*furIter).segId != segmentIdIn))
            {
                segIter++;
                continue;
            }

            while(furIter != (*segIter).end())
            {
                if(((*furIter).furId_used == 1) && ((*furIter).furId == furnitureIdIn))
                {
                    *existFlag = 1;
                    *furnitureAttr = *furIter;

                    ReleaseMutex(_hMutexMemory);
                    return;
                }

                furIter++;
            }

            segIter++;
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseInVehicle::getFurnitureBySegId(IN  uint32 segmentIdIn, 
                                       OUT std::list<furAttributesInVehicle_t>& furnitureAttrList)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        list<list<furAttributesInVehicle_t>>::iterator segIter = _furnitureList.begin();

        // For each segment
        while(segIter != _furnitureList.end())
        {
            // Find the segment containing the furniture
            uint8  segId_used = (*(*segIter).begin()).segId_used;
            uint32 segIdTmp   = (*(*segIter).begin()).segId;

            // TODO: need to check linked segment also
            if((segId_used != 1) || (segmentIdIn != segIdTmp))
            {
                segIter++;
                continue;
            }

            list<furAttributesInVehicle_t>::iterator furIter = (*segIter).begin();

            // For each furniture in the segment
            while(furIter != (*segIter).end())
            {
                furnitureAttrList.push_back(*furIter);

                furIter++;
            }

            segIter++;
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseInVehicle::getNearPointOnVector(IN point3D_t* gpsCurrP, 
                                        OUT uint32 *minSegId, 
                                        OUT int *minPointIdx, 
                                        OUT double *minDist)
    {
        // Locate current location in vectors
        *minDist = 1000;
        
        list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIter = _vectorList.begin();

        // For each segment
        while(vecListIter != _vectorList.end())
        {
            list<vector<vector<tlvCommon_t>>>::iterator pointListIter = (*vecListIter).begin();

            // FIXME: assume the first vector is center of road
            // TODO:  locate the segment first to reduce complexity
            //while(pointListIter != (*vecListIter).end())
            {
                // Get vector info from the first element
                lineAttributes_t lineAttrTmp;

                lineAttrTmp.segmentId = (*pointListIter)[0][0].value;
                lineAttrTmp.lineId = (*pointListIter)[0][data_lineId_e - data_lineBase_e + 1].value;  // +1 for skipping segment ID info
                lineAttrTmp.width  = *(float*)(&(*pointListIter)[0][data_lineWidth_e - data_lineBase_e + 1].value);
                lineAttrTmp.lineStyle  = (*pointListIter)[0][data_lineStyle_e - data_lineBase_e + 1].value;
                lineAttrTmp.segVersion = (*pointListIter)[0][data_lineSegVersion_e - data_lineBase_e + 1].value;
                lineAttrTmp.numPoints  = (*pointListIter)[0][data_linePointList_e - data_lineBase_e + 1].length;

                // Dash line
                //if(lineAttrTmp.lineStyle == 1)
                {
                    point3D_t point3D;
                    point3D_t gpsAhead1Tmp, gpsAhead2Tmp;
                    double dist;

                    // first point in vector
                    gpsAhead1Tmp.lat = *(double *)((*pointListIter)[1][0].value);
                    gpsAhead1Tmp.lon = *(double *)((*pointListIter)[1][1].value);
                    gpsAhead1Tmp.alt = *(double *)((*pointListIter)[1][2].value);

                    for(int pointIdx = 1; pointIdx < lineAttrTmp.numPoints; pointIdx++)
                    {
                        point3D.lat = *(double *)((*pointListIter)[pointIdx+1][0].value);
                        point3D.lon = *(double *)((*pointListIter)[pointIdx+1][1].value);
                        point3D.alt = *(double *)((*pointListIter)[pointIdx+1][2].value);

                        gpsAhead2Tmp = gpsAhead1Tmp;
                        gpsAhead1Tmp = point3D;

                        calcDistancePointToLine(&gpsAhead1Tmp, &gpsAhead2Tmp, gpsCurrP, &dist);

                        if(dist < *minDist)
                        {
                            *minSegId = lineAttrTmp.segmentId;
                            *minPointIdx = pointIdx;

                            *minDist = dist;
                        }
                    }
                }

                //pointListIter++;
            }

            vecListIter++;
        }
    }

    void databaseInVehicle::getLookAheadView(IN point3D_t* gpsCurrP, IN float distanceIn, OUT point3D_t* gpsAhead)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        uint8 existFlag;
        segAttributes_t segmentAttr;
        getSegmentByGps(gpsCurrP, &existFlag, &segmentAttr);

        if(existFlag == 1)
        {
            uint32 minSegId;
            int    minPointIdx;
            double minDist;

            point3D_t projectPoint;
            double distSum = 0;

            getNearPointOnVector(gpsCurrP, &minSegId, &minPointIdx, &minDist);

            // Calculate look ahead points
            list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIter = _vectorList.begin();

            // For each segment
            while(vecListIter != _vectorList.end())
            {
                list<vector<vector<tlvCommon_t>>>::iterator pointListIter = (*vecListIter).begin();

                // FIXME: assume the first vector is center of road
                //while(pointListIter != (*vecListIter).end())
                {
                    // Get vector info from the first element
                    lineAttributes_t lineAttrTmp;

                    lineAttrTmp.segmentId = (*pointListIter)[0][0].value;
                    lineAttrTmp.lineId = (*pointListIter)[0][data_lineId_e - data_lineBase_e + 1].value;  // +1 for skipping segment ID info
                    lineAttrTmp.width  = *(float*)(&(*pointListIter)[0][data_lineWidth_e - data_lineBase_e + 1].value);
                    lineAttrTmp.lineStyle  = (*pointListIter)[0][data_lineStyle_e - data_lineBase_e + 1].value;
                    lineAttrTmp.segVersion = (*pointListIter)[0][data_lineSegVersion_e - data_lineBase_e + 1].value;
                    lineAttrTmp.numPoints  = (*pointListIter)[0][data_linePointList_e - data_lineBase_e + 1].length;

                    if(lineAttrTmp.segmentId != minSegId)
                    {
                        vecListIter++;
                        continue;
                    }

                    point3D_t point3D;
                    point3D_t gpsAhead1Tmp, gpsAhead2Tmp;

                    // Locate the project point
                    {
                        gpsAhead2Tmp.lat = *(double *)((*pointListIter)[minPointIdx][0].value);
                        gpsAhead2Tmp.lon = *(double *)((*pointListIter)[minPointIdx][1].value);
                        gpsAhead2Tmp.alt = *(double *)((*pointListIter)[minPointIdx][2].value);

                        gpsAhead1Tmp.lat = *(double *)((*pointListIter)[minPointIdx+1][0].value);
                        gpsAhead1Tmp.lon = *(double *)((*pointListIter)[minPointIdx+1][1].value);
                        gpsAhead1Tmp.alt = *(double *)((*pointListIter)[minPointIdx+1][2].value);

                        // Calculate project point
                        calcProjectPointOnLine(&gpsAhead1Tmp, &gpsAhead2Tmp, gpsCurrP, &projectPoint);

                        double dist;
                        calcDistance(&projectPoint, &gpsAhead1Tmp, &dist);
                        distSum += dist;
                    }

                    // Look ahead
                    list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIterTmp = vecListIter;
                    list<vector<vector<tlvCommon_t>>>::iterator pointListIterTmp = pointListIter;
                    uint32 pointIdxAhead = minPointIdx;

                    while(distSum < distanceIn)
                    {
                        pointIdxAhead++;

                        if(pointIdxAhead >= (*pointListIterTmp)[0][data_linePointList_e - data_lineBase_e + 1].length)
                        {
                            vecListIterTmp++;

                            if(vecListIterTmp == _vectorList.end())
                            {
                                vecListIterTmp = _vectorList.begin();
                            }

                            pointIdxAhead = 1;

                            pointListIterTmp = (*vecListIterTmp).begin();
                        }

                        point3D.lat = *(double *)((*pointListIterTmp)[pointIdxAhead+1][0].value);
                        point3D.lon = *(double *)((*pointListIterTmp)[pointIdxAhead+1][1].value);
                        point3D.alt = *(double *)((*pointListIterTmp)[pointIdxAhead+1][2].value);

                        gpsAhead2Tmp = gpsAhead1Tmp;
                        gpsAhead1Tmp = point3D;

                        double dist;
                        calcDistance(&gpsAhead2Tmp, &gpsAhead1Tmp, &dist);
                        distSum += dist;
                    }

                    // Back
                    double distBack = distSum - distanceIn;

                    calcGpsBackOnLine(&gpsAhead2Tmp, &gpsAhead1Tmp, distBack, gpsAhead);

                    ReleaseMutex(_hMutexMemory);
                    return;

                    //pointListIter++;
                }

                vecListIter++;
            }
        }else
        {
            logPrintf(logLevelNotice_e, "DATABASE", "GPS out of segment range", FOREGROUND_RED);
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseInVehicle::getLookAheadFurnitures(IN point3D_t* gpsCurrP, 
                                          IN float distanceInAhead, 
                                          IN float distanceInBack, 
                                          OUT list<furAttributesInVehicle_t>& furnitureAttrList, 
                                          OUT list<point3D_t>& pointInRangeList)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        furnitureAttrList.clear();
        pointInRangeList.clear();

        uint8 existFlag;
        segAttributes_t segmentAttr;
        getSegmentByGps(gpsCurrP, &existFlag, &segmentAttr);

        if(existFlag == 1)
        {
            uint32 minSegId;
            int    minPointIdx;
            double minDist;

            point3D_t projectPoint;
            double distAheadSum = 0;
            double distBackSum = 0;

            getNearPointOnVector(gpsCurrP, &minSegId, &minPointIdx, &minDist);

            // Find vectors in the segment
            list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIter = _vectorList.begin();

            // For each segment
            while(vecListIter != _vectorList.end())
            {
                list<vector<vector<tlvCommon_t>>>::iterator pointListIter = (*vecListIter).begin();

                // FIXME: assume the first vector is center of road
                //while(pointListIter != (*vecListIter).end())
                {
                    // Get vector info from the first element
                    lineAttributes_t lineAttrTmp;

                    lineAttrTmp.segmentId = (*pointListIter)[0][0].value;
                    lineAttrTmp.lineId = (*pointListIter)[0][data_lineId_e - data_lineBase_e + 1].value;  // +1 for skipping segment ID info
                    lineAttrTmp.width  = *(float*)(&(*pointListIter)[0][data_lineWidth_e - data_lineBase_e + 1].value);
                    lineAttrTmp.lineStyle  = (*pointListIter)[0][data_lineStyle_e - data_lineBase_e + 1].value;
                    lineAttrTmp.segVersion = (*pointListIter)[0][data_lineSegVersion_e - data_lineBase_e + 1].value;
                    lineAttrTmp.numPoints  = (*pointListIter)[0][data_linePointList_e - data_lineBase_e + 1].length;

                    if(lineAttrTmp.segmentId != minSegId)
                    {
                        vecListIter++;
                        continue;
                    }

                    point3D_t point3D;
                    point3D_t gps1Tmp, gps2Tmp;

                    // Locate the project point
                    {
                        gps2Tmp.lat = *(double *)((*pointListIter)[minPointIdx][0].value);
                        gps2Tmp.lon = *(double *)((*pointListIter)[minPointIdx][1].value);
                        gps2Tmp.alt = *(double *)((*pointListIter)[minPointIdx][2].value);

                        gps1Tmp.lat = *(double *)((*pointListIter)[minPointIdx+1][0].value);
                        gps1Tmp.lon = *(double *)((*pointListIter)[minPointIdx+1][1].value);
                        gps1Tmp.alt = *(double *)((*pointListIter)[minPointIdx+1][2].value);

                        // Calculate project point
                        calcProjectPointOnLine(&gps1Tmp, &gps2Tmp, gpsCurrP, &projectPoint);

                        // Log project GPS points
                        pointInRangeList.push_back(projectPoint);

                        double dist;
                        calcDistance(&projectPoint, &gps1Tmp, &dist);
                        distAheadSum += dist;

                        calcDistance(&projectPoint, &gps2Tmp, &dist);
                        distBackSum += dist;
                    }

                    // Look ahead
                    {
                        list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIterTmp = vecListIter;
                        list<vector<vector<tlvCommon_t>>>::iterator pointListIterTmp = pointListIter;
                        uint32 pointIdxAhead = minPointIdx;
                        point3D_t gpsAhead1Tmp = gps1Tmp, gpsAhead2Tmp = gps2Tmp;

                        while(distAheadSum < distanceInAhead)
                        {
                            pointIdxAhead++;

                            // Log GPS points
                            pointInRangeList.push_back(gpsAhead1Tmp);

                            if(pointIdxAhead >= (*pointListIterTmp)[0][data_linePointList_e - data_lineBase_e + 1].length)
                            {
                                vecListIterTmp++;

                                if(vecListIterTmp == _vectorList.end())
                                {
                                    vecListIterTmp = _vectorList.begin();
                                }

                                pointIdxAhead = 1;

                                pointListIterTmp = (*vecListIterTmp).begin();
                            }

                            point3D.lat = *(double *)((*pointListIterTmp)[pointIdxAhead+1][0].value);
                            point3D.lon = *(double *)((*pointListIterTmp)[pointIdxAhead+1][1].value);
                            point3D.alt = *(double *)((*pointListIterTmp)[pointIdxAhead+1][2].value);

                            gpsAhead2Tmp = gpsAhead1Tmp;
                            gpsAhead1Tmp = point3D;

                            double dist;
                            calcDistance(&gpsAhead2Tmp, &gpsAhead1Tmp, &dist);
                            distAheadSum += dist;
                        }

                        // Back
                        {
                            double distBack = distAheadSum - distanceInAhead;

                            point3D_t gpsAhead;
                            calcGpsBackOnLine(&gpsAhead2Tmp, &gpsAhead1Tmp, distBack, &gpsAhead);

                            // Log GPS points
                            pointInRangeList.push_back(gpsAhead);
                        }
                    }

                    // Look back
                    {
                        list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIterTmp = vecListIter;
                        list<vector<vector<tlvCommon_t>>>::iterator pointListIterTmp = pointListIter;
                        uint32 pointIdxBack = minPointIdx - 1;
                        point3D_t gpsBack1Tmp = gps1Tmp, gpsBack2Tmp = gps2Tmp;

                        while(distBackSum < distanceInBack)
                        {
                            // Log GPS points
                            pointInRangeList.push_front(gpsBack2Tmp);

                            // pointIdxBack should >= 0.  Go previous segment if == 0.
                            if(pointIdxBack == 0)
                            {
                                if(vecListIterTmp == _vectorList.begin())
                                {
                                    vecListIterTmp = _vectorList.end();
                                }
                                --vecListIterTmp;

                                pointListIterTmp = (*vecListIterTmp).begin();

                                pointIdxBack = (*pointListIterTmp)[0][data_linePointList_e - data_lineBase_e + 1].length - 1;
                            }
                            --pointIdxBack;

                            point3D.lat = *(double *)((*pointListIterTmp)[pointIdxBack+1][0].value);
                            point3D.lon = *(double *)((*pointListIterTmp)[pointIdxBack+1][1].value);
                            point3D.alt = *(double *)((*pointListIterTmp)[pointIdxBack+1][2].value);

                            gpsBack1Tmp = gpsBack2Tmp;
                            gpsBack2Tmp = point3D;

                            double dist;
                            calcDistance(&gpsBack2Tmp, &gpsBack1Tmp, &dist);
                            distBackSum += dist;
                        }

                        // Back
                        {
                            double distBack = distBackSum - distanceInBack;

                            point3D_t gpsBack;
                            calcGpsBackOnLine(&gpsBack1Tmp, &gpsBack2Tmp, distBack, &gpsBack);

                            // Log GPS points
                            pointInRangeList.push_front(gpsBack);
                        }
                    }

                    // Check if furniture in range
                    list<list<furAttributesInVehicle_t>>::iterator segIter = _furnitureList.begin();
                    while(segIter != _furnitureList.end())
                    {
                        uint32 segId = (*(*segIter).begin()).segId;

                        list<furAttributesInVehicle_t> furnitureAttrListTmp;

                        getFurnitureBySegId(segId, furnitureAttrListTmp);

                        if(furnitureAttrListTmp.size() > 0)
                        {
                            list<furAttributesInVehicle_t>::iterator furnIter = furnitureAttrListTmp.begin();

                            // Check all furnitures in the segment
                            while(furnIter != furnitureAttrListTmp.end())
                            {
                                if((*furnIter).location_used)
                                {
                                    list<point3D_t>::iterator pointIter = pointInRangeList.begin();
                                    point3D_t prePoint = (*pointIter);
                                    point3D_t curPoint = prePoint;
                                    pointIter++;

                                    while(pointIter != pointInRangeList.end())
                                    {
                                        prePoint = curPoint;
                                        curPoint = (*pointIter);

                                        double distFurToLine;

                                        calcDistancePointToLine(&curPoint, &prePoint, &(*furnIter).location, &distFurToLine);

                                        if(distFurToLine < _distThreshMid * COEFF_DD2METER)
                                        {
                                            if((*furnIter).reliabRating > 0)
                                            {
                                                furnitureAttrList.push_back((*furnIter));
                                                break;
                                            }
                                        }
                                        pointIter++;
                                    }
                                }
                                furnIter++;
                            }
                        }

                        segIter++;
                    }

                    ReleaseMutex(_hMutexMemory);
                    return;

                    //pointListIter++;
                }

                vecListIter++;
            }
        }else
        {
            logPrintf(logLevelNotice_e, "DATABASE", "GPS out of segment range", FOREGROUND_RED);
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseInVehicle::addFurnitureTlv(IN uint8* tlvBuff, IN uint32 buffLen)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        _mEndPosition = tlvBuff + buffLen;

        furAttributesInVehicle_t furAttr;

        readTlvToFurniture(input, memory_e, &furAttr);

        addFurniture(&furAttr);

        string furTypeString = "Furniture \"";  furTypeString += ID2Name(furAttr.type);
        logPrintf(logLevelInfo_e, "DB_UPDATE", furTypeString, FOREGROUND_GREEN);
    }

    void databaseInVehicle::addFurniture(IN furAttributesInVehicle_t* furnitureIn)
    {        
        if(0 == furnitureIn->segId_used)
        {
            logPrintf(logLevelNotice_e, "DATABASE", "Segment ID should not be empty", FOREGROUND_RED);
            return;
        }

        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        furAttributesInVehicle_t furnitureElement = *furnitureIn;

        {
            int segIdFound = 0;
			// Check if the furniture already exist in database
            uint8 furExist = 0;
            list<list<furAttributesInVehicle_t>>::iterator segIter = _furnitureList.begin();
			list<list<furAttributesInVehicle_t>>::iterator segIterFound = _furnitureList.begin();

            // Check Segment ID in furniture list
            while(segIter != _furnitureList.end())
            {
			    // ID found
                if(((*(*segIter).begin()).segId_used == 1) && 
                    ((*(*segIter).begin()).segId == furnitureElement.segId))
            	{					            
            		if(0 == segIdFound)
					{
						segIterFound = segIter;
					}										
	                segIdFound = 1;
            	}

                list<furAttributesInVehicle_t>::iterator furIter = (*segIter).begin();

				// For each furniture in the segment
                while(furIter != (*segIter).end())
                {					
                    if((*furIter).furId == furnitureElement.furId)
                    {
                        furExist = 1;

						// ID found, replace directly
				        if(((*(*segIter).begin()).segId_used == 1) && 
				            ((*(*segIter).begin()).segId == furnitureElement.segId))
			            {										
                            *furIter = furnitureElement;
			            }
						else // move the furniture to another segment
						{
							int furReplaceFlag = 0;
							std::list<std::list<furAttributesInVehicle_t>>::iterator segIterCurrent = _furnitureList.begin();
							while(segIterCurrent != _furnitureList.end())
							{
								list<furAttributesInVehicle_t>::iterator furIterCurrent = (*segIterCurrent).begin();

								// For each furniture in the segment
						        while(furIterCurrent != (*segIterCurrent).end())
						        {
									// erase furniture from current segment
							        if((*(*segIter).begin()).segId == (*(*segIterCurrent).begin()).segId)
						            {
						                furReplaceFlag++;
						                furIterCurrent = (*segIterCurrent).erase(furIterCurrent);
						            }
									// add furniture to current segment
									else if(furnitureElement.segId == (*(*segIterCurrent).begin()).segId)
									{
										furReplaceFlag++;
										(*segIterCurrent).push_back(furnitureElement);
                                        furIterCurrent++;
									}
                                    else
                                    {
                                        furIterCurrent++;
                                    }
											

									if(furReplaceFlag >= 2)
									{
										break;
									}
						        }
								segIterCurrent++;
										
								if(furReplaceFlag >= 2)
								{
									break;
								}
							}
						}

                        break;
                    }

                    furIter++;

					if(1 == furExist)
					{
						break;
					}
                }
                segIter++;

				if(1 == furExist)
				{
					break;
				}
            }

			if(furExist == 0)
			// Add the furniture
			{
				if(segIdFound)
	            // Add furniture
	            {
	                (*segIterFound).push_back(furnitureElement);
	            }
	            else
	            // Create a new furniture list
	            {
	                std::list<furAttributesInVehicle_t> furListTmp;
	                furListTmp.push_back(furnitureElement);

	                _furnitureList.push_back(furListTmp);
	            }
			}            
        }
        ReleaseMutex(_hMutexMemory);
    }

    void databaseInVehicle::reduceFurnitureTlv(IN uint8* tlvBuff, IN uint32 buffLen)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;
		
        uint8 segExistFlag;
        segAttributes_t segAttr;		

        _mEndPosition = tlvBuff + buffLen;

        furAttributesInVehicle_t furAttr;

        readTlvToFurniture(input, memory_e, &furAttr);

        getSegmentByGps(&(furAttr.location), &segExistFlag, &segAttr);

        if(segExistFlag == 1)
        {
        	reduceFurnitureByFurId(&furAttr);

			string furTypeString = "Furniture \"";  furTypeString += ID2Name(furAttr.type);
        	logPrintf(logLevelInfo_e, "DB_UPDATE", furTypeString, FOREGROUND_BLUE | FOREGROUND_GREEN);
        }
		else
		{
			logPrintf(logLevelError_e, "DATABASE", "Segment ID not found when reduce", FOREGROUND_RED);
		}        
    }

    void databaseInVehicle::reduceFurnitureByFurId(IN  furAttributesInVehicle_t* furnitureAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        uint8 furExistFlag = 0;

        if(furnitureAttr->furId_used == 1)
        {
            list<list<furAttributesInVehicle_t>>::iterator segIter = _furnitureList.begin();
            
            // For each segment
            while(segIter != _furnitureList.end())
            {
                list<furAttributesInVehicle_t>::iterator furIter = (*segIter).begin();

                // For each furniture in the segment
                while(furIter != (*segIter).end())
                {
                    if((*furIter).furId == furnitureAttr->furId)
                    {
                        // Update furniture to existing furniturelist in database
                        furExistFlag = 1;

						*furIter = *furnitureAttr;

                        break;
                    }

                    furIter++;
                }

                if(furExistFlag == 1)
                {
                    break;
                }

                segIter++;
            }

            if(furExistFlag == 0)
            {
                logPrintf(logLevelNotice_e, "DATABASE", "Furniture ID not found when reduce", FOREGROUND_RED);
            }
        }else
        {
            logPrintf(logLevelNotice_e, "DATABASE", "Furniture ID is invalid when reduce", FOREGROUND_RED);
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseInVehicle::resetFurniture()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        _furnitureList.clear();

        ReleaseMutex(_hMutexMemory);

        logPrintf(logLevelInfo_e, "DB_UPDATE", "Reseting furnitures", FOREGROUND_BLUE | FOREGROUND_GREEN);
    }

    void databaseInVehicle::getAllFurnitures(OUT std::list<std::list<furAttributesInVehicle_t>>& furnitureListOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        furnitureListOut = _furnitureList;

        ReleaseMutex(_hMutexMemory);
    }
}
