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

#include <iostream>   // cout

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

        //getSegmentByGps(gpsP, &segExistFlag, &segAttr);

        //if(segExistFlag == 1)
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
                        if(checkRelGpsInRange(&gpsTmp, gpsP, distThresh))
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
        
        list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
        list<list<lineAttributes_t>>::iterator attrListIter = _lineAttrList.begin();

        // For each segment
        while(vecListIter != _vectorList.end())
        {
            list<vector<point3D_t>>::iterator pointListIter = (*vecListIter).begin();
            list<lineAttributes_t>::iterator attrIter = attrListIter->begin();

            // FIXME: assume the first vector is center of road
            // TODO:  locate the segment first to reduce complexity
            //while(pointListIter != (*vecListIter).end())
            {
                // Get vector info from the first element
                lineAttributes_t lineAttrTmp = (*attrIter);

                // Dash line
                //if(lineAttrTmp.lineStyle == 1)
                {
                    point3D_t point3D;
                    point3D_t gpsAhead1Tmp, gpsAhead2Tmp;
                    double dist;

                    // first point in vector
                    gpsAhead1Tmp = (*pointListIter)[0];

                    for(int pointIdx = 1; pointIdx < lineAttrTmp.numPoints; pointIdx++)
                    {
                        point3D = (*pointListIter)[pointIdx];

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
            attrListIter++;
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
            list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
            list<list<lineAttributes_t>>::iterator attrListIter = _lineAttrList.begin();

            // For each segment
            while(vecListIter != _vectorList.end())
            {
                list<vector<point3D_t>>::iterator pointListIter = (*vecListIter).begin();
                list<lineAttributes_t>::iterator attrIter = attrListIter->begin();

                // FIXME: assume the first vector is center of road
                //while(pointListIter != (*vecListIter).end())
                {
                    // Get vector info from the first element
                    lineAttributes_t lineAttrTmp = (*attrIter);

                    if(lineAttrTmp.segmentId != minSegId)
                    {
                        vecListIter++;
                        continue;
                    }

                    point3D_t point3D;
                    point3D_t gpsAhead1Tmp, gpsAhead2Tmp;

                    // Locate the project point
                    {
                        gpsAhead2Tmp = (*pointListIter)[minPointIdx];

                        gpsAhead1Tmp = (*pointListIter)[minPointIdx+1];;

                        // Calculate project point
                        calcProjectPointOnLine(&gpsAhead1Tmp, &gpsAhead2Tmp, gpsCurrP, &projectPoint);

                        double dist;
                        calcRelDistance(&projectPoint, &gpsAhead1Tmp, &dist);
                        distSum += dist;
                    }

                    // Look ahead
                    list<list<vector<point3D_t>>>::iterator vecListIterTmp = vecListIter;
                    list<vector<point3D_t>>::iterator pointListIterTmp = pointListIter;
                    uint32 pointIdxAhead = minPointIdx;

                    while(distSum < distanceIn)
                    {
                        pointIdxAhead++;

                        if(pointIdxAhead >= (*pointListIterTmp).size())
                        {
                            vecListIterTmp++;

                            if(vecListIterTmp == _vectorList.end())
                            {
                                vecListIterTmp = _vectorList.begin();
                            }

                            pointIdxAhead = 1;

                            pointListIterTmp = (*vecListIterTmp).begin();
                        }

                        point3D = ((*pointListIterTmp)[pointIdxAhead+1]);

                        gpsAhead2Tmp = gpsAhead1Tmp;
                        gpsAhead1Tmp = point3D;

                        double dist;
                        calcRelDistance(&gpsAhead2Tmp, &gpsAhead1Tmp, &dist);
                        distSum += dist;
                    }

                    // Back
                    double distBack = distSum - distanceIn;

                    calcRelBackOnLine(&gpsAhead2Tmp, &gpsAhead1Tmp, distBack, gpsAhead);

                    ReleaseMutex(_hMutexMemory);
                    return;

                    //pointListIter++;
                }

                vecListIter++;
                attrListIter++;
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
            list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
            list<list<lineAttributes_t>>::iterator attrListIter = _lineAttrList.begin();

            // For each segment
            while(vecListIter != _vectorList.end())
            {
                list<vector<point3D_t>>::iterator pointListIter = (*vecListIter).begin();
                list<lineAttributes_t>::iterator attrIter = attrListIter->begin();

                // FIXME: assume the first vector is center of road
                //while(pointListIter != (*vecListIter).end())
                {
                    // Get vector info from the first element
                    lineAttributes_t lineAttrTmp = (*attrIter);

                    if(lineAttrTmp.segmentId != minSegId)
                    {
                        vecListIter++;
                        continue;
                    }

                    point3D_t point3D;
                    point3D_t gps1Tmp, gps2Tmp;

                    // Locate the project point
                    {
                        gps2Tmp = ((*pointListIter)[minPointIdx]);

                        gps1Tmp = ((*pointListIter)[minPointIdx+1]);

                        // Calculate project point
                        calcProjectPointOnLine(&gps1Tmp, &gps2Tmp, gpsCurrP, &projectPoint);

                        // Log project GPS points
                        pointInRangeList.push_back(projectPoint);

                        double dist;
                        calcRelDistance(&projectPoint, &gps1Tmp, &dist);
                        distAheadSum += dist;

                        calcRelDistance(&projectPoint, &gps2Tmp, &dist);
                        distBackSum += dist;
                    }

                    // Look ahead
                    {
                        list<list<vector<point3D_t>>>::iterator vecListIterTmp = vecListIter;
                        list<vector<point3D_t>>::iterator pointListIterTmp = pointListIter;
                        uint32 pointIdxAhead = minPointIdx;
                        point3D_t gpsAhead1Tmp = gps1Tmp, gpsAhead2Tmp = gps2Tmp;

                        while(distAheadSum < distanceInAhead)
                        {
                            pointIdxAhead++;

                            // Log GPS points
                            pointInRangeList.push_back(gpsAhead1Tmp);

                            if(pointIdxAhead >= (*pointListIterTmp).size())
                            {
                                vecListIterTmp++;

                                if(vecListIterTmp == _vectorList.end())
                                {
                                    vecListIterTmp = _vectorList.begin();
                                }

                                pointIdxAhead = 1;

                                pointListIterTmp = (*vecListIterTmp).begin();
                            }

                            point3D = ((*pointListIterTmp)[pointIdxAhead+1]);

                            gpsAhead2Tmp = gpsAhead1Tmp;
                            gpsAhead1Tmp = point3D;

                            double dist;
                            calcRelDistance(&gpsAhead2Tmp, &gpsAhead1Tmp, &dist);
                            distAheadSum += dist;
                        }

                        // Back
                        {
                            double distBack = distAheadSum - distanceInAhead;

                            point3D_t gpsAhead;
                            calcRelBackOnLine(&gpsAhead2Tmp, &gpsAhead1Tmp, distBack, &gpsAhead);

                            // Log GPS points
                            pointInRangeList.push_back(gpsAhead);
                        }
                    }

                    // Look back
                    {
                        list<list<vector<point3D_t>>>::iterator vecListIterTmp = vecListIter;
                        list<vector<point3D_t>>::iterator pointListIterTmp = pointListIter;
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

                                pointIdxBack = (*pointListIterTmp).size();
                            }
                            --pointIdxBack;

                            point3D = ((*pointListIterTmp)[pointIdxBack+1]);

                            gpsBack1Tmp = gpsBack2Tmp;
                            gpsBack2Tmp = point3D;

                            double dist;
                            calcRelDistance(&gpsBack2Tmp, &gpsBack1Tmp, &dist);
                            distBackSum += dist;
                        }

                        // Back
                        {
                            double distBack = distBackSum - distanceInBack;

                            point3D_t gpsBack;
                            calcRelBackOnLine(&gpsBack1Tmp, &gpsBack2Tmp, distBack, &gpsBack);

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

                                        if(distFurToLine < _distThreshMid)
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

    void databaseInVehicle::addFurnitureListTlv(IN uint8* tlvBuff, IN uint32 buffLen)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        _mEndPosition = tlvBuff + buffLen;

        // Add all furnitures in the tlv buffer
        while(inputLoc < _mEndPosition)
        {
            furAttributesInVehicle_t furAttr;

            readTlvToFurniture(input, memory_e, &furAttr);

            addFurniture(&furAttr);
        }

        //string furTypeString = "Furniture List";
        //logPrintf(logLevelInfo_e, "DB_UPDATE", furTypeString, FOREGROUND_GREEN);
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

        //logPrintf(logLevelInfo_e, "DB_UPDATE", "Reseting furnitures", FOREGROUND_BLUE | FOREGROUND_GREEN);
    }

    void databaseInVehicle::getAllFurnitures(OUT std::list<std::list<furAttributesInVehicle_t>>& furnitureListOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        furnitureListOut = _furnitureList;

        ReleaseMutex(_hMutexMemory);
    }

    bool databaseInVehicle::getAllFurnituresAsync(OUT std::list<std::list<furAttributesInVehicle_t>>& furnitureListOut)
    {
        if(WAIT_OBJECT_0 == WaitForSingleObject(_hMutexMemory,1))
        {
            furnitureListOut = _furnitureList;

            ReleaseMutex(_hMutexMemory);

            return true;
        }else
        {
            // faled to get furnitures
            return false;
        }
    }

	void databaseInVehicle::getLaneGpsTlv(IN list<laneType_t> *laneInfo,
										  IN resource_e sourceFlag, 
										  OUT void** output, 
                                          OUT int32* length)
	{
		list<laneType_t>::iterator listIdx = laneInfo->begin();
		int byteNum = 0;
		while(listIdx != laneInfo->end())
		{
			
			tlvCommon_t tlvTmp;
			tlvCfg_t* tlvCfgP;
			
			// lane id
			tlvCfgP = &_tlvCfg_dataLine_a[data_lineLaneId_e - data_lineBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, 1, (uint32)&(listIdx->laneId));//FIXME:Qin
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			// lane width
			tlvCfgP = &_tlvCfg_dataLine_a[data_lineLaneWidth_e - data_lineBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, *(uint32*)&(listIdx->laneWidth));
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			// line style
			tlvCfgP = &_tlvCfg_dataLine_a[data_lineStyle_e - data_lineBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, listIdx->lineStyle);
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			// lane change flag
			tlvCfgP = &_tlvCfg_dataLine_a[data_lineLaneChFlag_e - data_lineBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, listIdx->laneChangeFlag);
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			// left line paint flag
			tlvCfgP = &_tlvCfg_dataLine_a[data_linePaintFlagL_e - data_lineBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, listIdx->linePaintFlagL);
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			// right line paint flag
			tlvCfgP = &_tlvCfg_dataLine_a[data_linePaintFlagR_e - data_lineBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, listIdx->linePaintFlagR);
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			//GPS pointer L
			tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLatitudeL_e - data_pointBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsL.lat));
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLongitudeL_e - data_pointBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsL.lon));
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointAltitudeL_e - data_pointBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsL.alt));
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			//GPS pointer R
			tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLatitudeR_e - data_pointBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsR.lat));
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLongitudeR_e - data_pointBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsR.lon));
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointAltitudeR_e - data_pointBase_e];
			setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsR.alt));
			byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            // GPS track
            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointGpsTrackLat_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsTrack.lat));
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointGpsTrackLon_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsTrack.lon));
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            tlvCfgP = &_tlvCfg_dataPoint_a[data_pointGpsTrackAlt_e - data_pointBase_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)&(listIdx->gpsTrack.alt));
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

			++listIdx;
		}
		*length = byteNum;
	}

	void databaseInVehicle::resetAllVectors(void)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        // Clear vectors in current database
        _vectorList.clear();
        _lineAttrList.clear();

        ReleaseMutex(_hMutexMemory);
    }

    bool databaseInVehicle::checkDbCompleteByGps(IN point3D_t *gpsInP)
    {
        list<list<vector<point3D_t>>> allLines;
        list<list<lineAttributes_t>> lineAttr;

        getAllVectors(allLines, lineAttr);

        bool isCompleteFlag = false;

        vector<double> distRVec;
        vector<double> distLVec;

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

                    int dbDataLen = (*refLineInSeg).size();

                    if(dbDataLen != refAttrsInSeg->numPoints)
                    {
                        logPrintf(logLevelNotice_e, "ROAD_VEC_GEN", "number points do not match!", FOREGROUND_RED);
                    }

                    // For each point on the reference line
                    for(int idx = 0; idx < dbDataLen; idx++)
                    {
                        point3D_t dbPoint = (*refLineInSeg)[idx];
                        
                        if(checkRelGpsInRange(&dbPoint, gpsInP, 5))
                        {
                            double distR, distL;

                            calcRelDistance(&dbPoint, &(*RLineInSeg)[idx], &distR);
                            calcRelDistance(&dbPoint, &(*LLineInSeg)[idx], &distL);

                            distRVec.push_back(distR);
                            distLVec.push_back(distL);
                        }
                    }
                }
            }
        }

        if(!distRVec.empty())
        {
            isCompleteFlag = true;

            for(int idx = 0; idx < distRVec.size(); ++idx)
            {
                if((distRVec[idx] < 2) || (distLVec[idx] < 2))
                // db is complete at this point
                {
                    isCompleteFlag = false;
                    break;
                }
            }
        }

        //std::cout << "isCompleteFlag = " << isCompleteFlag << std::endl;

        return isCompleteFlag;
    }
}
