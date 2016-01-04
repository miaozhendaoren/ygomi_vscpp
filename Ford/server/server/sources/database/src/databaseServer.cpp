/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  databaseServer.cpp
* @brief Source file for database, definitions for server side
*
* Change Log:
*      Date                Who             What
*      2015/3/5           Linkun Xu       Create
*******************************************************************************
*/

#include "database.h"
#include "databaseServer.h"
#include "apiDataStruct.h"
#include "appInitCommon.h"

#include <stdio.h>    // FILE
#include <windows.h>  // CreateMutex, FOREGROUND_RED

#include "LogInfo.h"  // logPrintf

using std::list;
using std::vector;
using std::string;

namespace ns_database
{
    volatile uint32 furAttributesServer_t::_furIdMax = 0;

    furAttributesServer_t::furAttributesServer_t(furAttributes_t* furAttrIn) : furAttributes_t()
    {
        segId_used      = furAttrIn->segId_used;
        segId           = furAttrIn->segId;
        furId_used      = furAttrIn->furId_used;
        furId           = furAttrIn->furId;
        segVersion_used = furAttrIn->segVersion_used;
        segVersion      = furAttrIn->segVersion;
        location_used   = furAttrIn->location_used;
        location        = furAttrIn->location;
        angle_used      = furAttrIn->angle_used;
        angle           = furAttrIn->angle;
        type_used       = furAttrIn->type_used;
        type            = furAttrIn->type;
        side_used       = furAttrIn->side_used;
        side[0]         = furAttrIn->side[0];
        side[1]         = furAttrIn->side[1];
        sideFlag_used   = furAttrIn->sideFlag_used;
        sideFlag        = furAttrIn->sideFlag;
        offset_used     = furAttrIn->offset_used;
        offset          = furAttrIn->offset;
        reliabRating_used = furAttrIn->reliabRating_used;
        reliabRating    = furAttrIn->reliabRating;
        inLoopIdx_used  = furAttrIn->inLoopIdx_used;
        inLoopIdx       = furAttrIn->inLoopIdx;
    }

    furAttributesServer_t::furAttributesServer_t() : furAttributes_t()
    {}

    furAttributesServer_t::~furAttributesServer_t()
    {
        format();
    }

    void furAttributesServer_t::calcReliability()
    {
        // Keep resonable history size
        if (_detectedFlagHistory.size() > MAX_HISTORY_NUM)
        {
            _detectedFlagHistory.erase(_detectedFlagHistory.begin());
        }

        if (reliabRating > 99)
        {
            reliabRating = 99;
        }
    }

    void furAttributesServer_t::calcLocation()
    {
        // Keep resonable history size
        if (_locationHistory.size() > MAX_HISTORY_NUM)
        {
            _locationHistory.erase(_locationHistory.begin());
        }

        // Calculate location
        if (_locationHistory.size() > 0)
        {
            list<point3D_t>::iterator locationHistoryIter = _locationHistory.begin();
            double altSum = 0, latSum = 0, lonSum = 0;

            while(locationHistoryIter != _locationHistory.end())
            {
                altSum += (*locationHistoryIter).alt;
                latSum += (*locationHistoryIter).lat;
                lonSum += (*locationHistoryIter).lon;

                ++locationHistoryIter;
            }

            location.alt = altSum / _locationHistory.size();
            location.lat = latSum / _locationHistory.size();
            location.lon = lonSum / _locationHistory.size();
        }
    }

    void furAttributesServer_t::update(INOUT furAttributesServer_t* furAttrIn)
    {
        _detectedFlagHistory.push_back(true);
        _locationHistory.push_back(furAttrIn->location);

        reliabRating = reliabRating + 1;

        calcReliability();
        calcLocation();

        *furAttrIn = *this;
    }

    void furAttributesServer_t::reduce(OUT furAttributesServer_t* furAttrIn)
    {
        _detectedFlagHistory.push_back(false);

        if (reliabRating > 0)
        {
            reliabRating = reliabRating - 1;
        }
        calcReliability();

        *furAttrIn = *this;
    }

    void furAttributesServer_t::format()
    {
        furAttributes_t::format();

        _detectedFlagHistory.clear();
        _locationHistory.clear();
    }

    bool furAttributesServer_t::checkFurId(uint32 furIdIn)
    {
        bool output = false;
        if (furId_used == 1)
        {
            if (furId == furIdIn)
            {
                output = true;
            }
        }

        return output;
    }

    uint32 furAttributesServer_t::getFurIdMaxIncrease()
    {
        uint32 output = _furIdMax++;
        return output;
    }

    databaseServer::databaseServer() : database()
    {}

    databaseServer::databaseServer(string inFileStr) : database(inFileStr)
    {}

    databaseServer::~databaseServer() // FIXME
    {
        _furnitureList.clear();
        _newDataVec.clear();
    }

    void databaseServer::initDb()
    {
        database::initDb();

        _furnitureList.clear();
    }

    void databaseServer::getFurnitureByGps(IN  point3D_t*  gpsP, 
                                     IN  double distThreshMeter,
                                     OUT list<furAttributesServer_t>& furnitureAttrList)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();
            
        // For each segment
        while(segIter != _furnitureList.end())
        {
            list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

            // For each furniture in the segment
            while(furIter != (*segIter).end())
            {
                if((*furIter).location_used == 1)
                {
                    point3D_t gpsTmp = (*furIter).location;

                    // Push furniture to output list if in GPS range
                    if(checkRelGpsInRange(&gpsTmp, gpsP, distThreshMeter))
                    {
                        furnitureAttrList.push_back(*furIter);
                    }
                }

                furIter++;
            }

            segIter++;
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::getFurnitureById(IN  uint32 segmentIdIn, 
                                    IN  uint32 furnitureIdIn,
                                    OUT uint8* existFlag, 
                                    OUT furAttributesServer_t* furnitureAttr)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        *existFlag = 0;

        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

        while(segIter != _furnitureList.end())
        {
            list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

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

    void databaseServer::getFurnitureBySegId(IN  uint32 segmentIdIn, 
                                       OUT std::list<furAttributesServer_t>& furnitureAttrList)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

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

            list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

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

    void databaseServer::calcFurnitureHeight(IN point3D_t *locIn, IN uint8 sideFlagIn, OUT double *altOut)
    {
        // Input height is a value from 0 to 1 representing the sign height on the screen
        double height = (locIn->alt * 0.6 + 0.5); // *0.6 for the real sign surface height is about 0.6m
                                                  // +0.5 for the half sign surface height

        // Limit the traffic sign height in [0, 5]
        if(height <= 1)
        {
            height = 1;
        }else if(height >= 5)
        {
            height = 5;
        }
                
        *altOut = height;
    }

    void databaseServer::addFurnitureTlv(IN uint8* tlvBuff, 
                                         IN uint32 buffLen,
                                         OUT uint8*  tlvOutBuff,
                                         OUT uint32* outBuffLen)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        // test
        //memcpy(tlvOutBuff, tlvBuff, buffLen);
        //*outBuffLen = buffLen;
        // end test

        _mEndPosition = tlvBuff + buffLen;

        furAttributes_t furAttr, furAttrOut;

        readTlvToFurniture(input, memory_e, &furAttr);

        addFurniture(&furAttr, &furAttrOut);

        void*  outputLoc = tlvOutBuff;
        void** output = &outputLoc;
        convFurnitureToTlv(&furAttrOut, memory_e, output, (int32 *)outBuffLen);

        string furTypeString = "Furniture \"";  furTypeString += ID2Name(furAttr.type); furTypeString += "\" reliability ++";
        logPrintf(logLevelInfo_e, "DB_UPDATE", furTypeString, FOREGROUND_GREEN);
    }

    void databaseServer::addFurnitureListTlv(IN uint8* tlvBuff, IN uint32 buffLen)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        _mEndPosition = tlvBuff + buffLen;

        // Add all furnitures in the tlv buffer
        while(inputLoc < _mEndPosition)
        {
            furAttributes_t furAttr, furAttrOut;

            readTlvToFurniture(input, memory_e, &furAttr);

            addFurniture(&furAttr, &furAttrOut);
        }

        //string furTypeString = "Furniture List";
        //logPrintf(logLevelInfo_e, "DB_UPDATE", furTypeString, FOREGROUND_GREEN);
    }

    void databaseServer::addFurnitureListAsIsTlv(IN uint8* tlvBuff, IN uint32 buffLen)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        _mEndPosition = tlvBuff + buffLen;

        WaitForSingleObject(_hMutexMemory,INFINITE);

        // Add all furnitures in the tlv buffer
        while(inputLoc < _mEndPosition)
        {
            furAttributes_t furAttr, furAttrOut;

            readTlvToFurniture(input, memory_e, &furAttr);

            furAttributesServer_t furServ(&furAttr);

            insertOneFurniture(furServ);
        }

        ReleaseMutex(_hMutexMemory);

        //string furTypeString = "Furniture List";
        //logPrintf(logLevelInfo_e, "DB_UPDATE", furTypeString, FOREGROUND_GREEN);
    }

    void databaseServer::resetSingleFurSegId(IN  furAttributesServer_t* furnitureIn, IN  uint32 newSegId)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

        // For each segment
        while( segIter != _furnitureList.end() )
        {
            list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

            if(furIter->segId == furnitureIn->segId)
            {
                // For each furniture in the segment
                while( furIter != (*segIter).end() )
                {
                    if(furIter->furId == furnitureIn->furId)
                    {
                        // Merge the two furnitures
                        // TODO: currently delete the old furniture in DB
                        list<furAttributesServer_t>::iterator furIterTmp = furIter++;
                        segIter->erase(furIterTmp);
                        break;
                    }
                    else
                    {
                        furIter++;
                    }
                }
            }
            else if(furIter->segId == newSegId)
            {
                furnitureIn->furId = newSegId;
                (*segIter).push_back(*furnitureIn);
            }

            segIter++;
        }
        ReleaseMutex(_hMutexMemory);
    }

    
    void databaseServer::resetAllFurSegId(void)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

        // For each segment
        while( segIter != _furnitureList.end() )
        {
            list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

            // For each furniture in the segment
            while( furIter != (*segIter).end() )
            {
                // relocate furniture's segment ID
                uint8 segLocated = 0;
                segAttributes_t segAttr;
                getSegmentIdByGps((*furIter), &segLocated, &segAttr);

                uint32 newSegId = segAttr.segId;
                if(newSegId != furIter->segId)
                {
                    int furResetFlag = 0;
                    //erase fur in current segment
                    list<furAttributesServer_t>::iterator furIterTmp = furIter++;
                    furAttributesServer_t furTmp = *furIterTmp;
                    segIter->erase(furIterTmp);

                    //add fur to another segment
                    furTmp.furId = newSegId;
                    insertOneFurniture(furTmp);
                }
                furIter++;
            }
            segIter++;
        }
        ReleaseMutex(_hMutexMemory);
    }

    bool databaseServer::resetFurnitureSideFlag(IN furAttributesServer_t* furnitureIn, IN uint32 segId)
    {
        bool angleReverseFlag = false;
        
        WaitForSingleObject(_hMutexMemory,INFINITE);
        if (furnitureIn->angle_used == 1)
        {
            list<segAttributes_t>::iterator segAttrListIter = _segmentList.begin(); 
            while(segAttrListIter != _segmentList.end())
            {
                // for each seg
                uint32 segmentIdDb = segAttrListIter->segId;
                if(segmentIdDb == segId)
                {
                    float angleSeg = 0;
                    calcNormalAngle(&(segAttrListIter->ports[0]), &(segAttrListIter->ports[1]), &angleSeg);

                    float angleFur = furnitureIn->angle;
                    double angleDiff;

                    if (angleFur > angleSeg)
                    {
                        angleDiff = angleFur - angleSeg;
                    }else
                    {
                        angleDiff = angleSeg - angleFur;
                    }

                    if( (angleDiff > (PI/2)) && (angleDiff < (3*PI/2)) )
                    {
                        angleReverseFlag = true;
                    }
                    break;
                }
                
                ++segAttrListIter;
            }
        }

        if(true == angleReverseFlag)
        {
            switch(furnitureIn->sideFlag)
            {
            case rightSide_e:
                furnitureIn->sideFlag = leftSide_e;
                break;
            case leftSide_e:
                furnitureIn->sideFlag = middleLeft_e;
                break;
            default:
                break;
            }
        }
        else
        {
            switch(furnitureIn->sideFlag)
            {
            case rightSide_e:
                furnitureIn->sideFlag = rightSide_e;
                break;
            case leftSide_e:
                furnitureIn->sideFlag = middleRight_e;
                break;
            default:
                break;
            }
        }
        
        ReleaseMutex(_hMutexMemory);
        return angleReverseFlag;
    }
                                      
    void databaseServer::addFurniture(IN furAttributes_t* furnitureIn,
                                      OUT furAttributes_t* furnitureOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        furAttributesServer_t furnitureServerIn(furnitureIn);

        // Check if same furniture exist in DB
        bool furIdFoundFlag = false;
        list<list<furAttributesServer_t>>::iterator segIter;
        list<furAttributesServer_t>::iterator furIter;

        {
            // Locate segment and change side flag
            uint8 segLocated = 0;
            segAttributes_t segAttr;
            getSegmentIdByGps(furnitureServerIn, &segLocated, &segAttr);
            if (segLocated == 0)
            // Failed to found the segment by GPS of the furniture
            {
                ReleaseMutex(_hMutexMemory);
                logPrintf(logLevelError_e, "DATABASE", "Furniture location not in any segment", FOREGROUND_RED);
                return;
            }
            else
            {
                furnitureServerIn.segId_used = 1;
                furnitureServerIn.segId = segAttr.segId;
                resetFurnitureSideFlag(&furnitureServerIn, segAttr.segId);
            }
        }

        if (furnitureServerIn.furId_used == 1)
        {
            // Check furId for existing furniture
            segIter = _furnitureList.begin();

            // Check Segment ID in furniture list
            while (segIter != _furnitureList.end())
            {
                furIter = (*segIter).begin();

                while (furIter != (*segIter).end())
                {
                    if ((*furIter).checkFurId(furnitureServerIn.furId))
                    {
                        furIdFoundFlag = true;                        
                        break;
                    }

                    furIter++;
                }

                if (furIdFoundFlag == true)
                {
                    break;
                }

                segIter++;
            }
        }

        if (0) // bugfix of (furIdFoundFlag == true): do not check furniture ID.  Always use location and type.
        // Update existing one
        {
            // Update with new location and reliability
            (*furIter).update(&furnitureServerIn);

            // Check if need to merge other funitures in DB after updating location
            mergeFurInSameRange(*furIter);
            
            // Put furniture to the side of road
            calcFurnitureRoadSideLoc3(&(*furIter));

            // Calculate height of the furniture
            double altOut;
            calcFurnitureHeight(&furIter->location, furIter->sideFlag, &altOut);
            furIter->location.alt = altOut;
                
            // Update local structure
			furnitureServerIn = *furIter;
        }
        else
        {
            // Check same fur by location
            bool furFoundFlag = false;

            segIter = _furnitureList.begin();

            // Check Segment ID in furniture list
            while (segIter != _furnitureList.end())
            {
                furIter = (*segIter).begin();

                while (furIter != (*segIter).end())
                {                    
                    if ( checkTwoFurnitureSame(&(*furIter), &furnitureServerIn, _distThreshNear, _angleThresh) )
                    {
                        furFoundFlag = true;
                        break;
                    }

                    furIter++;
                }

                if (furFoundFlag == true)
                {
                    break;
                }

                segIter++;
            }

            if (furFoundFlag)
            // Update existing furniture
            {
                //resetFurnitureSideFlag(&furnitureServerIn, furnitureServerIn.segId);
                
                // Update with new location and reliability
                (*furIter).update(&furnitureServerIn);
                
                // Update segId in furniture after location updated
                {
                    uint8 segLocated = 0;
                    segAttributes_t segAttr;
                    getSegmentIdByGps((*furIter), &segLocated, &segAttr);
                    if (segLocated == 0)
                    // Failed to found the segment by GPS of the furniture
                    {
                        ReleaseMutex(_hMutexMemory);
                        logPrintf(logLevelError_e, "DATABASE", "Furniture location not in any segment", FOREGROUND_RED);
                        return;
                    }
                    else
                    {
                        if((*furIter).segId != segAttr.segId)
                        {
                            (*furIter).segId = segAttr.segId;

                            list<furAttributesServer_t>::iterator furIterNew;

                            furIterNew = insertOneFurniture((*furIter));

                            segIter->erase(furIter);

                            furIter = furIterNew;
                        }
                    }
                }

                // Check if need to merge other funitures in DB after updating location
                mergeFurInSameRange(*furIter);
                
                // Put furniture to the side of road
                calcFurnitureRoadSideLoc3(&(*furIter));

                mergeFurInSameRange(*furIter);

                // Calculate height of the furniture
                double altOut;
                calcFurnitureHeight(&furIter->location, furIter->sideFlag, &altOut);
                furIter->location.alt = altOut;
                
                // Update local structure
				furnitureServerIn = *furIter;
            }
            else
            // Add new furniture
            {
                // Assign new furniture ID
                // TODO: Use mutex to protect _furIdMax
                furnitureServerIn.furId_used = 1;
                furnitureServerIn.furId      = furnitureServerIn.getFurIdMaxIncrease();

                furnitureServerIn.reliabRating_used = 1;
                furnitureServerIn.reliabRating = 1;

                // Locate segment
                /*uint8 segLocated = 0;
                segAttributes_t segAttr;
                getSegmentIdByGps(furnitureServerIn, &segLocated, &segAttr);

                if (segLocated == 0)
                // Failed to found the segment by GPS of the furniture
                {
                    ReleaseMutex(_hMutexMemory);
                    logPrintf(logLevelError_e, "DATABASE", "Furniture location not in any segment", FOREGROUND_RED);
                    return;
                }
                else
                {
                    furnitureServerIn.segId_used = 1;
                    furnitureServerIn.segId = segAttr.segId;
                    resetFurnitureSideFlag(&furnitureServerIn, furnitureServerIn.segId);
                }*/

                calcFurnitureRoadSideLoc3(&furnitureServerIn);

                mergeFurInSameRange(furnitureServerIn);

                // Calculate altitude from input height
                double altOut;
                calcFurnitureHeight(&furnitureServerIn.location, furnitureServerIn.sideFlag, &altOut);
                furnitureServerIn.location.alt = altOut;

                insertOneFurniture(furnitureServerIn);
            }

        }

        *furnitureOut = furnitureServerIn;

        if (furnitureServerIn.segId_used == 0)
        {
            int stop = 1;
        }

        ReleaseMutex(_hMutexMemory);
    }

    list<furAttributesServer_t>::iterator databaseServer::insertOneFurniture(IN furAttributesServer_t &furnitureServerIn)
    {
        // Check Segment ID in furniture list
        bool segIdFound = false;
        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();
        while(segIter != _furnitureList.end())
        {
            // ID found
            if( (*(*segIter).begin()).segId == furnitureServerIn.segId )
            {
                segIdFound = true;
                break;
            }

            segIter++;
        }

        if(segIdFound)
        // Add furniture
        {
            (*segIter).push_back(furnitureServerIn);

            return (--segIter->end());
        }
        else
        // Create a new furniture list
        {
            std::list<furAttributesServer_t> furListTmp;
            furListTmp.push_back(furnitureServerIn);

            _furnitureList.push_back(furListTmp);

            return --(--_furnitureList.end())->end();
        }
    }

    void databaseServer::resetFurnitureRoadSideLoc3()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        vector<point3D_t> pointListPolygonL;    
        vector<point3D_t> pointListPolygonR;
        
        vector<point3D_t> *firstLine   = &pointListPolygonL;
        vector<point3D_t> *middle1Line = &pointListPolygonL;
        vector<point3D_t> *middle2Line = &pointListPolygonL;
		vector<point3D_t> *lastLine    = &pointListPolygonL;
        
        list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
        list<segAttributes_t>::iterator segAttrListIter = _segmentList.begin(); 

        if((_updateSectionIdList.size() <= 0) || (_furnitureList.size() <= 0))
        {
            ReleaseMutex(_hMutexMemory);
            return;
        }
        
        // For each segment
        while(vecListIter != _vectorList.end())
        {
            int vectorNumInSeg = (*vecListIter).size();
            if(2 > vectorNumInSeg)
            {
                ++vecListIter;
                ++segAttrListIter;
                continue;
            }

            // for each fur
            uint32 segmentIdDb = segAttrListIter->segId;
            if(checkIdInUpdateIdList(_updateSectionIdList, segmentIdDb))
            {
                list<vector<point3D_t>>::iterator pointListIter = vecListIter->begin();
                list<vector<point3D_t>>::iterator pointListIterLast = vecListIter->end();
                --pointListIterLast;

                while(pointListIter != vecListIter->end())
                {
                    int count = (*pointListIter)[0].count;
                    
                    if(pointListIter == vecListIter->begin()) // first line
                    {
                        firstLine = &(*pointListIter);
                    }
                    else if( (LANE_END_LINE_FLAG == count) && (pointListIter != pointListIterLast) )
                    {
                        middle1Line = &(*pointListIter);
                        ++pointListIter;
                        middle2Line = &(*pointListIter);
                    }
                    else if(pointListIter == pointListIterLast) // last line
                    {
                        lastLine = &(*pointListIter);
                    }

                    ++pointListIter;
                }

                int pointNumFist = firstLine->size();
                int pointNumMiddle1 = middle1Line->size();
                int pointNumMiddle2 = middle2Line->size();
                int pointNumLast = lastLine->size();
                
                int pointIdx = 0;
                for(pointIdx = 0; pointIdx < pointNumFist; pointIdx++)
                {
                    point3D_t pointTmp = (*firstLine)[pointIdx];
                    pointListPolygonL.push_back(pointTmp);
                }                    
                for(pointIdx = pointNumLast-1; pointIdx >= 0; pointIdx--)
                {
                    point3D_t pointTmp = (*lastLine)[pointIdx];
                    pointListPolygonL.push_back(pointTmp);
                }
                
                if(0 < pointNumMiddle1)
                {   
                    for(pointIdx = 0; pointIdx < pointNumMiddle1; pointIdx++)
                    {
                        point3D_t pointTmp = (*middle1Line)[pointIdx];
                        pointListPolygonR.push_back(pointTmp);
                    }
                    for(pointIdx = pointNumMiddle2-1; pointIdx >= 0; pointIdx--)
                    {
                        point3D_t pointTmp = (*middle2Line)[pointIdx];
                        pointListPolygonR.push_back(pointTmp);
                    }                    
                }

                list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

                // For each Furniture
                while(segIter != _furnitureList.end())
                {
                    list<furAttributesServer_t>::iterator furIter = (*segIter).begin();
                    
                    uint32 segIdIn = furIter->segId;
                    // Segment ID match
                    if(segmentIdDb == segIdIn)
                    {
                    
                        // For each furniture in the segment
                        while(furIter != (*segIter).end())
                        {
                            point3D_t furLocation = furIter->location;
                            point3D_t nearFurLocation = furLocation;
                            uint8  furSideFlag_used = furIter->sideFlag_used;
                            uint8  furSideFlag = furIter->sideFlag; // 1: right side, 2: left side, 3: both sides, 4: on the road
                            if(1 != furSideFlag_used)
                            {
                                furSideFlag = 4;
                            }
                            
                            // not reset location for case 3/4
                            if((3 == furSideFlag) || (4 == furSideFlag))
                            {
                                ++furIter;
                                continue;
                            }

                            // check loop index
                            if((1 == segAttrListIter->loopIdx_used) && (1 == furIter->inLoopIdx_used))
                            {
                                if(furIter->inLoopIdx != segAttrListIter->loopIdx) // if furniture not in loop
                                {
                                    ++furIter;
                                    continue;
                                }
                            }
                            
                            double minDist = DBL_MAX;

                            if( (furIter->sideFlag == rightSide_e) || (furIter->sideFlag == leftSide_e) || (0 == pointNumMiddle1) )
                            {
                                bool pointInFlag = pointInPolygon(furLocation, pointListPolygonL);
                                if(true == pointInFlag)
                                {
                                    int numPoints = pointListPolygonL.size();
                                    // For each point
                                    for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                                    {
                                        point3D_t pointTmp = pointListPolygonL[pointIdx];

                                        //uint8 pointSideFlag = pointListPolygonFlagL[pointIdx];

                                        double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                                        //1: right side, or  4: on the road
                                        //if(pointSideFlag == furSideFlag)
                                        {
                                            if(distance < minDist)
                                            {
                    							minDist = distance;
                                                nearFurLocation = pointTmp;
                                            }
                                        }
                                    }
                                    
                                    furIter->location = nearFurLocation;
                                    furIter->location.alt = furLocation.alt;
                                    furIter->location.count = furLocation.count;
                                    furIter->location.paintFlag = furLocation.paintFlag;

                                    mergeFurInSameRange(*furIter);
                                }
                            }
                            if( ((furIter->sideFlag == middleLeft_e) || (furIter->sideFlag == middleRight_e)) && (0 < pointNumMiddle1) )
                            {
                                bool pointInFlag = pointInPolygon(furLocation, pointListPolygonR);
                                if(false == pointInFlag)
                                {
                                    int numPoints = pointListPolygonR.size();
                                    // For each point
                                    for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                                    {
                                        point3D_t pointTmp = pointListPolygonR[pointIdx];

                                        //uint8 pointSideFlag = pointListPolygonFlagR[pointIdx];

                                        double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                                        //1: right side, or  4: on the road
                                        //if(pointSideFlag == furSideFlag)
                                        {
                                            if(distance < minDist)
                                            {
                    							minDist = distance;
                                                nearFurLocation = pointTmp;
                                            }
                                        }
                                    }
                                    furIter->location = nearFurLocation;
                                    furIter->location.alt = furLocation.alt;
                                    furIter->location.count = furLocation.count;
                                    furIter->location.paintFlag = furLocation.paintFlag;

                                    mergeFurInSameRange(*furIter);
                                }
                            }
                            
                            ++furIter;
                        }

                    }

                    ++segIter;
                }
            }

            ++vecListIter;
            ++segAttrListIter;
            
            pointListPolygonL.clear();
            pointListPolygonR.clear();

            firstLine   = &pointListPolygonL;
            middle1Line = &pointListPolygonL;
            middle2Line = &pointListPolygonL;
		    lastLine    = &pointListPolygonL;
        }       

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::resetFurnitureRoadSideLoc2()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        vector<point3D_t> pointListPolygon;
        vector<uint8> pointListPolygonFlag;
        
        list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
        list<segAttributes_t>::iterator segAttrListIter = _segmentList.begin(); 

        if((_updateSectionIdList.size() <= 0) || (_furnitureList.size() <= 0))
        {
            ReleaseMutex(_hMutexMemory);
            return;
        }
        
        // For each segment
        while(vecListIter != _vectorList.end())
        {
            if((*vecListIter).empty())
            // Segment with no vector
            {
                ++vecListIter;
                ++segAttrListIter;
                continue;
            }

            // for each fur
            uint32 segmentIdDb = segAttrListIter->segId;
            if(checkIdInUpdateIdList(_updateSectionIdList, segmentIdDb))
            {            
                list<vector<point3D_t>>::iterator pointListIter = vecListIter->begin();

                int numPoints = (*pointListIter).size();
                // For each point
                for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                {
                    point3D_t pointTmp = (*pointListIter)[pointIdx];
                    pointListPolygon.push_back(pointTmp);
                    pointListPolygonFlag.push_back(2);
                }
                pointListIter = vecListIter->end();
                pointListIter--;
                
                numPoints = (*pointListIter).size();
                // For each point
                for(int pointIdx = numPoints-1; pointIdx >= 0; pointIdx--)
                {
                    point3D_t pointTmp = (*pointListIter)[pointIdx];
                    pointListPolygon.push_back(pointTmp);
                    pointListPolygonFlag.push_back(1);
                }            

                list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

                // For each segment
                while(segIter != _furnitureList.end())
                {
                    list<furAttributesServer_t>::iterator furIter = (*segIter).begin();
                    
                    uint32 segIdIn = furIter->segId;
                    // Segment ID match
                    if(segmentIdDb == segIdIn)
                    {
                    
                        // For each furniture in the segment
                        while(furIter != (*segIter).end())
                        {
                            point3D_t furLocation = furIter->location;
                            point3D_t nearFurLocation = furLocation;
                            uint8  furSideFlag_used = furIter->sideFlag_used;
                            uint8  furSideFlag = furIter->sideFlag; // 1: right side, 2: left side, 3: both sides, 4: on the road
                            if(1 != furSideFlag_used)
                            {
                                furSideFlag = 4;
                            }
                            
                            // not reset location for case 3/4
                            if((3 == furSideFlag) || (4 == furSideFlag))
                            {
                                ++furIter;
                                continue;
                            }

                            // check loop index
                            if((1 == segAttrListIter->loopIdx_used) && (1 == furIter->inLoopIdx_used))
                            {
                                if(furIter->inLoopIdx != segAttrListIter->loopIdx) // if furniture not in loop
                                {
                                    ++furIter;
                                    continue;
                                }
                            }
                            
                            double minDist = DBL_MAX;

                            bool pointInFlag = pointInPolygon(furLocation, pointListPolygon);
                            if(true == pointInFlag)
                            {
                                int numPoints = pointListPolygon.size();
                                // For each point
                                for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                                {
                                    point3D_t pointTmp = pointListPolygon[pointIdx];

                                    uint8 pointSideFlag = pointListPolygonFlag[pointIdx];

                                    double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                                    //1: right side, or  4: on the road
                                    if(pointSideFlag == furSideFlag)
                                    {
                                        if(distance < minDist)
                                        {
                							minDist = distance;
                                            nearFurLocation = pointTmp;
                                        }
                                    }
                                }
                                
                                furIter->location = nearFurLocation;
                                furIter->location.alt = furLocation.alt;
                                furIter->location.count = furLocation.count;
                                furIter->location.paintFlag = furLocation.paintFlag;
                            }
                            
                            ++furIter;
                        }

                    }

                    ++segIter;
                }
            }

            ++vecListIter;
            ++segAttrListIter;
            
            pointListPolygon.clear();
            pointListPolygonFlag.clear();
        }       

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::resetFurnitureRoadSideLoc1()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        vector<point3D_t> pointListPolygon;
        vector<uint8> pointListPolygonFlag;
        
        vector<point3D_t> pointListPolygonL;
        vector<point3D_t> pointListPolygonR;

        list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
        
        // For each segment
        while(vecListIter != _vectorList.end())
        {
            if((*vecListIter).empty())
            // Segment with no vector
            {
                ++vecListIter;
                continue;
            }

            int vectorNumInSeg = (*vecListIter).size();
            if(2 > vectorNumInSeg)
            {
                ++vecListIter;
                continue;
            }
            
            list<vector<point3D_t>>::iterator pointListIter = vecListIter->begin();

            int numPoints = (*pointListIter).size();
            // For each point
            for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
            {
                point3D_t pointTmp = (*pointListIter)[pointIdx];

                pointListPolygonL.push_back(pointTmp);
            }
            ++vecListIter;
        }

        vecListIter = _vectorList.end();
        // For each segment
        while(vecListIter != _vectorList.begin())
        {
            if((*vecListIter).empty())
            // Segment with no vector
            {
                --vecListIter;
                continue;
            }

            int vectorNumInSeg = (*vecListIter).size();
            if(2 > vectorNumInSeg)
            {
                --vecListIter;
                continue;
            }
            
            list<vector<point3D_t>>::iterator pointListIter = vecListIter->end();
             
            int numPoints = (*pointListIter).size();
            // For each point
            for(int pointIdx = numPoints-1; pointIdx >= 0; pointIdx--)
            {
                point3D_t pointTmp = (*pointListIter)[pointIdx];
                
                pointListPolygonR.push_back(pointTmp);
            }

            --vecListIter;
        }

        //sequence the Polygon
        {
            for(int index = 0; index<pointListPolygonL.size(); index++)
            {
                point3D_t pointTmp = pointListPolygonL[index];
                pointListPolygon.push_back(pointTmp);
                pointListPolygonFlag.push_back(2);
            }
            pointListPolygonL.clear();
            
            for(int index = 0; index<pointListPolygonR.size(); index++)
            {
                point3D_t pointTmp = pointListPolygonR[index];
                pointListPolygon.push_back(pointTmp);
                pointListPolygonFlag.push_back(1);
            }
            pointListPolygonR.clear();

        }

        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();
            
        // For each segment
        while(segIter != _furnitureList.end())
        {
            list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

            // For each furniture in the segment
            while(furIter != (*segIter).end())
            {
                uint8  segId_usedIn = furIter->segId_used;
                if(0 == segId_usedIn)
                {
                    furIter++;
                    continue;
                }
                point3D_t furLocation = furIter->location;
                point3D_t nearFurLocation = furLocation;
                uint8  furSideFlag_used = furIter->sideFlag_used;
                uint8  furSideFlag = furIter->sideFlag; // 1: right side, 2: left side, 3: both sides, 4: on the road
                if(1 != furSideFlag_used)
                {
                    furSideFlag = 4;
                }
                
                // not reset location for case 3/4
                if((3 == furSideFlag) || (4 == furSideFlag))
                {
                    furIter++;
                    continue;
                }
                
                double minDist = DBL_MAX;

                bool pointInFlag = pointInPolygon(furLocation, pointListPolygon);
                if(true == pointInFlag)
                {
                    int numPoints = pointListPolygon.size();
                    // For each point
                    for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                    {
                        point3D_t pointTmp = pointListPolygon[pointIdx];

                        uint8 pointSideFlag = pointListPolygonFlag[pointIdx];

                        double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                        //1: right side, or  4: on the road
                        if(pointSideFlag == furSideFlag)
                        {
                            if(distance < minDist)
                            {
    							minDist = distance;
                                nearFurLocation = pointTmp;
                            }
                        }
                    }
                    
                    furIter->location = nearFurLocation;
                    furIter->location.alt = furLocation.alt;
                    furIter->location.count = furLocation.count;
                    furIter->location.paintFlag = furLocation.paintFlag;
                }
                
                furIter++;
            }
            
            segIter++;
        }
        
        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::resetFurnitureRoadSideLoc()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        list<list<vector<point3D_t>>>::iterator vecListIter = _vectorList.begin();
        list<segAttributes_t>::iterator segAttrListIter = _segmentList.begin();        
        
        // For each segment
        while(vecListIter != _vectorList.end())
        {
            if((*vecListIter).empty())
            // Segment with no vector
            {
                ++vecListIter;
                ++segAttrListIter;
                continue;
            }

            uint32 segmentIdDb = segAttrListIter->segId;

            if(checkIdInUpdateIdList(_updateSectionIdList, segmentIdDb))
            {
                list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();
                
                // For each segment
                while(segIter != _furnitureList.end())
                {
                    list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

                    // For each furniture in the segment
                    while(furIter != (*segIter).end())
                    {
                        uint8  segId_usedIn = furIter->segId_used;
                        if(0 == segId_usedIn)
                        {
                            furIter++;
                            continue;
                        }
                        uint32 segIdIn = furIter->segId;
                        // Segment ID match
                        if(segmentIdDb == segIdIn)
                        {                
                            int vectorNumInSeg = (*vecListIter).size();
                            if(2 > vectorNumInSeg)
                            {
                                furIter++;
                                continue;
                            }
                            
                            if(0 == furIter->location_used)
                            {
                                furIter++;
                                continue;
                            }
                            point3D_t furLocation = furIter->location;
                            point3D_t nearFurLocation = furLocation;
                            uint8  furSideFlag_used = furIter->sideFlag_used;
                            uint8  furSideFlag = furIter->sideFlag; // 1: right side, 2: left side, 3: both sides, 4: on the road
                            if(1 != furSideFlag_used)
                            {
                                furSideFlag = 4;
                            }
                            
                            // not reset location for case 3/4
                            if((3 == furSideFlag) || (4 == furSideFlag))
                            {
                                furIter++;
                                continue;
                            }
                            
                            double minDist = DBL_MAX;
                            
                            vector<point3D_t> pointListPolygon;
                            list<vector<point3D_t>>::iterator pointListIter = vecListIter->begin();

                            // For each vector
                            //while(pointListIter != vecListIter->end())
                            {
                                int numPoints = (*pointListIter).size();
                                // For each point
                                for(int pointIdx = 0; pointIdx < numPoints; pointIdx++)
                                {
                                    point3D_t pointTmp = (*pointListIter)[pointIdx];

                                    double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                                    //2: left side, or  4: on the road
                                    if(1 != furSideFlag)
                                    {
                                        if(distance < minDist)
                                        {
                							minDist = distance;
                                            nearFurLocation = pointTmp;
                                        }
                                    }
                                        
                                    pointListPolygon.push_back(pointTmp);
                                }

                                for(int pointListIdx = 1; pointListIdx < vectorNumInSeg; pointListIdx++)
                                {
                                    ++pointListIter;
                                }
                                
                                numPoints = (*pointListIter).size();
                                // For each point
                                for(int pointIdx = numPoints-1; pointIdx >= 0; pointIdx--)
                                {
                                    point3D_t pointTmp = (*pointListIter)[pointIdx];

                                    double distance = (furLocation.lat - pointTmp.lat) * (furLocation.lat - pointTmp.lat) + (furLocation.lon - pointTmp.lon) * (furLocation.lon - pointTmp.lon);

                                    //1: right side, or  4: on the road
                                    if(2 != furSideFlag)
                                    {
                                        if(distance < minDist)
                                        {
                							minDist = distance;
                                            nearFurLocation = pointTmp;
                                        }
                                    }
                                    
                                    pointListPolygon.push_back(pointTmp);
                                }

                                bool pointInFlag = pointInPolygon(furLocation, pointListPolygon);
                                if(true == pointInFlag)
                                {
                                    furIter->location = nearFurLocation;
                                    furIter->location.alt = furLocation.alt;
                                    furIter->location.count = furLocation.count;
                                    furIter->location.paintFlag = furLocation.paintFlag;
                                }
                            }
                            
                            pointListPolygon.clear();                        
                        }

                        furIter++;
                    }

                    segIter++;
                }
            }

            ++vecListIter;
            ++segAttrListIter;            
        }       

        ReleaseMutex(_hMutexMemory);
    }

    int databaseServer::getFurUpdateFlag()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        int flag =  _furUpdateFlag;

        ReleaseMutex(_hMutexMemory);

        return flag;
    }

    void databaseServer::setFurUpdateFlag(int flag)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        _furUpdateFlag = flag;

        ReleaseMutex(_hMutexMemory);
    }
    
    void databaseServer::resetFurUpdateFlag()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        _furUpdateFlag = 0;

        ReleaseMutex(_hMutexMemory);
    }

    bool databaseServer::getUpateMsgToTlv(OUT uint8 **tlvP, 
                                        OUT uint32 *pduOutOffset, 
                                        OUT vector<uint32> &vecPduOffset, 
                                        OUT vector<uint32> &furPduOffset)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);
        
        bool flag = false;
        
        int furUpdateFlag = _furUpdateFlag;
        int vecUpdateListSize = _updateSectionIdList.size();
    
        if((1 == furUpdateFlag) || (vecUpdateListSize > 0))
        {            
            int32 totalBufLen = MAX_PAYLOAD_BYTE_NUM + MAX_ROAD_POINT_BYTES;
            uint8 *payloadFromDb = new uint8[totalBufLen];
            if(payloadFromDb == NULL)
            {
                ReleaseMutex(_hMutexMemory);
                logPrintf(logLevelCrit_e, "DBAccess", "Cannot new buffer", FOREGROUND_RED);
                return false;
            }
    
            uint8 *outBuffPtr = payloadFromDb;
            int32 outBuffOffset = 0;
            int32 outPduIdx = 0;
            bool updateFlag = false;
            
            if(vecUpdateListSize > 0)
            {            
                {
                    int payloadBytes = 0;
    
                    // Get updated road vector from database
                    uint8* bufferAddr = outBuffPtr;
                    int32 outBuffLen;
                    database_gp->getSpecificIdVectorsTlv(memory_e,
                                                    _updateSectionIdList,
                                                    (void**)&bufferAddr, 
                                                    &outBuffLen);
    
                    if (outBuffLen > MAX_ROAD_POINT_BYTES)
                    {
                        logPrintf(logLevelCrit_e, "DBAccess", "Buffer overflow!", FOREGROUND_RED);
                    }

                    vecPduOffset.push_back(outBuffOffset);
    
                    outBuffPtr += outBuffLen;
                    outBuffOffset += outBuffLen;
    
                    ++outPduIdx;
                }
                
                database_gp->resetUpateSectionIdList();
            }
                // pack the fur update message
            if(1 == furUpdateFlag)
            {
                // Convert all furniture to TLVs
                int32 segNumOfFur;
                database_gp->getSegNumOfFurniture(&segNumOfFur);
    
                for(int segIndex = 0; segIndex < segNumOfFur; ++segIndex)
                {
                    uint8* bufferAddr = outBuffPtr;
                    int32 furMsgLen;
                    int32 furNum;
                    int segId;
                    int getFlag = database_gp->getSegIdInFurList(segIndex, &segId);
    
                    if(0 != getFlag)
                    {
                        database_gp->getFurnitureTlvInSeg(segId,
                                                          totalBufLen - outBuffOffset,
                                                          bufferAddr, 
                                                          &furMsgLen, 
                                                          &furNum);
    
                        if(furNum > 0)
                        {
                            furPduOffset.push_back(outBuffOffset);
                            
                            outBuffPtr += furMsgLen;
                            outBuffOffset += furMsgLen;
                            ++outPduIdx;
                        }
                    }
                }
    
                database_gp->resetFurUpdateFlag();
            }

            *tlvP = payloadFromDb;
            *pduOutOffset = outBuffOffset;
            flag = true;
        }
        
        ReleaseMutex(_hMutexMemory);
        return flag;
    }

    void databaseServer::reduceFurnitureTlv(IN uint8* tlvBuff, 
                                            IN uint32 buffLen,
                                            OUT uint8*  tlvOutBuff,
                                            OUT uint32* outBuffLen)
    {
        void*  inputLoc = tlvBuff;
        void** input = &inputLoc;

        // test
        //memcpy(tlvOutBuff, tlvBuff, buffLen);
        //*outBuffLen = buffLen;
        // end test

        _mEndPosition = tlvBuff + buffLen;

        furAttributes_t furAttr, furAttrOut;

        readTlvToFurniture(input, memory_e, &furAttr);

        reduceFurnitureByFurId(&furAttr, &furAttrOut);

        void*  outputLoc = tlvOutBuff;
        void** output = &outputLoc;
        convFurnitureToTlv(&furAttrOut, memory_e, output, (int32 *)outBuffLen);

        string furTypeString = "Furniture \"";  furTypeString += ID2Name(furAttr.type); furTypeString += "\" reliability --";
        logPrintf(logLevelInfo_e, "DB_UPDATE", furTypeString, FOREGROUND_BLUE | FOREGROUND_GREEN);
    }

    void databaseServer::reduceFurnitureByFurId(IN  furAttributes_t* furnitureIn, 
                                                OUT furAttributes_t* furnitureOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        furAttributesServer_t furnitureServerIn(furnitureIn);

        bool furIdFoundFlag = false;
        list<list<furAttributesServer_t>>::iterator segIter;
        list<furAttributesServer_t>::iterator furIter;

        if (furnitureServerIn.furId_used == 1)
        {
            // Check furId for existing furniture
            segIter = _furnitureList.begin();

            // Check Segment ID in furniture list
            while (segIter != _furnitureList.end())
            {
                furIter = (*segIter).begin();

                while (furIter != (*segIter).end())
                {
                    if ((*furIter).checkFurId(furnitureServerIn.furId))
                    {
                        furIdFoundFlag = true;
                        break;
                    }

                    furIter++;
                }

                if (furIdFoundFlag == true)
                {
                    break;
                }

                segIter++;
            }
        }else
        {
            // error
            logPrintf(logLevelError_e, "DATABASE", "No furniture ID specified when reduce", FOREGROUND_RED);
        }

        if (furIdFoundFlag == true)
        {
            (*furIter).reduce(&furnitureServerIn);
        }else
        {
            // error
            logPrintf(logLevelInfo_e, "DATABASE", "Furniture ID not found when reduce", FOREGROUND_RED);
        }

        *furnitureOut = furnitureServerIn;

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::getAllFurnitures(OUT std::list<std::list<furAttributesServer_t>>& furnitureListOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        furnitureListOut = _furnitureList;

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::getSegNumOfFurniture(OUT int32 *numSegOfFur)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        *numSegOfFur = _furnitureList.size();

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::resetFurniture()
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        _furnitureList.clear();

        ReleaseMutex(_hMutexMemory);

        logPrintf(logLevelInfo_e, "DB_UPDATE", "Reseting furnitures", FOREGROUND_BLUE | FOREGROUND_GREEN);
    }

    void databaseServer::mergeFurInSameRange(INOUT furAttributes_t& furnitureInOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

        // For each segment
        while( segIter != _furnitureList.end() )
        {
            list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

            // For each furniture in the segment
            while( furIter != (*segIter).end() )
            {
                // Same furniture nearby with different furId
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
                double distThresh;
                if(furnitureInOut.inLoopIdx == 1 || furnitureInOut.inLoopIdx == 2) // small loop or T cross
                {
                    distThresh = 10.0;
                }else // big loop
                {
                    distThresh = 20.0;
                }
#else
                double distThresh = 6.0;
#endif
                if( checkTwoFurnitureSame( &(*furIter), &furnitureInOut, distThresh, _angleThresh) ) 
                {
                    if(furIter->furId != furnitureInOut.furId)
                    {
                        // Merge the two furnitures
                        // TODO: currently delete the old furniture in DB
                        furnitureInOut.reliabRating += furIter->reliabRating;
                        furIter = segIter->erase(furIter);
                    }else
                    {
                        furIter++;
                    }
                }
                else
                {
                    furIter++;
                }
            }

            // Delete the segment if no furniture in this segment
            if(segIter->size() == 0)
            {
                segIter = _furnitureList.erase(segIter);
            }else
            {
                segIter++;
            }
        }

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::getFurnitureTlvInSeg(IN  int32 segIdIn,
                                              IN  int32 maxPayloadLen,
                                              OUT uint8 *furnitureListP, 
                                              OUT int32 *msgLenOut, 
                                              OUT int32 *furNumOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        int32 payloadLen = 0, furNum = 0;

        list<list<furAttributesServer_t>>::iterator segIter;
        list<furAttributesServer_t>::iterator furIter;
        furAttributes_t furAttri;
        // Check furId for existing furniture
        segIter = _furnitureList.begin();

        // Check Segment ID in furniture list
        while (segIter != _furnitureList.end())
        {
            furIter = (*segIter).begin();

            // report furnitures with specified segment ID
            if ((furIter != (*segIter).end()) && 
                (furIter->segId_used == 1) && 
                (furIter->segId == segIdIn))
            {
                while (furIter != (*segIter).end())
                {                
                    void*  outputLoc = furnitureListP + payloadLen;
                    void** output = &outputLoc;
                    int32 msgLen;

                    if(payloadLen >= maxPayloadLen)
                    {
                        logPrintf(logLevelError_e, "DB_SYNC", "Payload buffer overflow", FOREGROUND_RED);
                        break;
                    }
                    furAttri = *furIter;
                    convFurnitureToTlv(&furAttri, memory_e, output, &msgLen);
                    payloadLen += msgLen;
                    furNum++;

                    furIter++;
                }
            }
            segIter++;

            //string furTypeString = "Synchronize furnitures in segment ID ";  furTypeString += segIdIn;
            //logPrintf(logLevelInfo_e, "DB_SYNC", furTypeString, FOREGROUND_BLUE | FOREGROUND_GREEN);
        }
        
        *msgLenOut = payloadLen;
        *furNumOut = furNum;

        ReleaseMutex(_hMutexMemory);
    }

	uint32 databaseServer::getFurnitureVersion()
    {    
		uint32 version = 0xFFFF; // default version
		
		return version;
    }
	void databaseServer::readTlvToLaneInfo(IN  void** input,
						   IN  resource_e sourceFlag,
						   IN  int buffLen,
						   OUT list<laneType_t>* laneInfo)
		{
			_mEndPosition = (uint8*)(*input) + buffLen;
			int fileEnd = 0;
			tlvCommon_t tlvTmp;
			typeId_e idBase;
			int numByteRead;
        
			logPosition(input, sourceFlag);

			readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
			// Parse TLVs
            laneType_t laneInfoTemp;
			while(numByteRead)
			{
				switch(idBase)
				{
					case data_lineBase_e:
					{
						switch(tlvTmp.typeId)
						{
							case data_lineLaneWidth_e:
								laneInfoTemp.laneWidth = *((float*)&tlvTmp.value);
								break;
							case data_lineStyle_e:
								laneInfoTemp.lineStyle = tlvTmp.value;
								break;
							case data_lineLaneChFlag_e:
								laneInfoTemp.laneChangeFlag = tlvTmp.value;
								break;
							case data_linePaintFlagL_e:
								laneInfoTemp.linePaintFlagL = tlvTmp.value;
								break;
							case data_linePaintFlagR_e:
								laneInfoTemp.linePaintFlagR = tlvTmp.value;
								break;
							case data_lineLaneId_e:
								laneInfoTemp.laneId = *((uint32*)_dataTmpBuf);//FIXMW:Qin
								break;
						}
					}
					break;
					case data_pointBase_e:
					{
						switch(tlvTmp.typeId)
						{
							case data_pointLatitudeL_e:
								laneInfoTemp.gpsL.lat = *((double*)_dataTmpBuf);
								break;
							case data_pointLongitudeL_e:
								laneInfoTemp.gpsL.lon = *((double*)_dataTmpBuf);
								break;
							case data_pointAltitudeL_e:
								laneInfoTemp.gpsL.alt = *((double*)_dataTmpBuf);
								break;

							case data_pointLatitudeR_e:
								laneInfoTemp.gpsR.lat = *((double*)_dataTmpBuf);
								break;
							case data_pointLongitudeR_e:
								laneInfoTemp.gpsR.lon = *((double*)_dataTmpBuf);
								break;
							case data_pointAltitudeR_e:
								laneInfoTemp.gpsR.alt = *((double*)_dataTmpBuf);
								break;

                            case data_pointGpsTrackLat_e:
                                laneInfoTemp.gpsTrack.lat = *((double*)_dataTmpBuf);
                                break;
                            case data_pointGpsTrackLon_e:
                                laneInfoTemp.gpsTrack.lon = *((double*)_dataTmpBuf);
                                break;
                            case data_pointGpsTrackAlt_e:
                                laneInfoTemp.gpsTrack.alt = *((double*)_dataTmpBuf);
                                laneInfo->push_back(laneInfoTemp);
                                break;
						}
					}
					break;
					default:
					{
						fileEnd = 1;

						break;
					}
				}
				if(fileEnd == 1)
				{
					resetPosition(input, sourceFlag);
					break; // while
				}

				logPosition(input, sourceFlag);

				readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
			}
		}

    void databaseServer::getNewDataVec(list<vector<point3D_t>> &newDataVec)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        newDataVec.clear();
        newDataVec = _newDataVec;

        ReleaseMutex(_hMutexMemory);
    }

    void databaseServer::setNewDataVec(list<vector<point3D_t>> &newDataVec)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        _newDataVec.clear();
        _newDataVec = newDataVec;

        ReleaseMutex(_hMutexMemory);
    }

	int databaseServer::getSegIdInFurList(int furListIndex, int *furSegId)
    {
		WaitForSingleObject(_hMutexMemory, INFINITE);

        int returnFlag = 0;
		*furSegId = 0;

		int furListNum = _furnitureList.size();
		if(furListIndex < furListNum)
		{
			list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();

			for(int index=0; index<furListIndex; index++)
			{
				segIter++;
			}

            int furNum = (*segIter).size();
            if(0 < furNum)
            {
                list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

                if((*furIter).segId_used == 1)
                {
                    *furSegId = (*furIter).segId;
                    returnFlag = 1;
                }
            }
		}

		ReleaseMutex(_hMutexMemory);

		return returnFlag;
    }

    bool databaseServer::loadFurFromFile(IN string fileName)
    {
        // Load furnitures from file
        int32 totalBufLen = _MAX_PAYLOAD_BYTE_NUM;
        uint8 *payloadFromDb = new uint8[totalBufLen];
        if(payloadFromDb == NULL)
        {
            return false;
        }

        FILE* fp = fopen(fileName.c_str(), "rb");
        if (fp == NULL)
        {
            delete payloadFromDb;
            return false;
        }

        int buffLen = fread(payloadFromDb, 1, totalBufLen, fp);

        fclose(fp);

        // Reset furnitures in DB
        WaitForSingleObject(_hMutexMemory, INFINITE);

        resetFurniture();

        addFurnitureListAsIsTlv(payloadFromDb, buffLen);

        ReleaseMutex(_hMutexMemory);

        delete payloadFromDb;

        return true;
    }

    bool databaseServer::saveFurToFile(IN string fileName)
    {
        // Get all furnitures
        int32 totalBufLen = _MAX_PAYLOAD_BYTE_NUM;
        uint8 *payloadFromDb = new uint8[totalBufLen];
        if(payloadFromDb == NULL)
        {
            return false;
        }

        uint8 *outBuffPtr = payloadFromDb;
        int32 outBuffOffset = 0;

        WaitForSingleObject(_hMutexMemory, INFINITE);
        
        int segNumOfFur;
        getSegNumOfFurniture(&segNumOfFur);

        for(int segIndex = 0; segIndex < segNumOfFur; ++segIndex)
        {
            uint8* bufferAddr = outBuffPtr;
            int32 furMsgLen;
            int32 furNum;
            int segId;
            int getFlag = getSegIdInFurList(segIndex, &segId);

            if(0 != getFlag)
            {
                getFurnitureTlvInSeg(segId,
                                     totalBufLen - outBuffOffset,
                                     bufferAddr, 
                                     &furMsgLen, 
                                     &furNum);

                if(furNum > 0)
                {
                    outBuffPtr += furMsgLen;
                    outBuffOffset += furMsgLen;
                }
            }
        }

        ReleaseMutex(_hMutexMemory);

        // Save furnitures to file
        FILE* fp = fopen(fileName.c_str(), "wb");
        if (fp == NULL)
        {
            delete payloadFromDb;
            return false;
        }

        fwrite(payloadFromDb, outBuffOffset, 1, fp);

        fclose(fp);

        delete payloadFromDb;

        return true;
    }

    bool databaseServer::loadRoadVecFromFile(IN string fileName,
        IN bool bRevDir/* = false*/)
    {
        // Load furnitures from file
        int32 totalBufLen = _MAX_ROAD_POINT_BYTES;
        uint8 *payloadFromDb = new uint8[totalBufLen];
        if(payloadFromDb == NULL)
        {
            return false;
        }

        FILE* fp = fopen(fileName.c_str(), "rb");
        if (fp == NULL)
        {
            delete payloadFromDb;
            return false;
        }

        int buffLen = fread(payloadFromDb, 1, totalBufLen, fp);

        fclose(fp);

        list<backgroundSectionData> bgVec;
        bool status = convTlvToBgRoadVec(payloadFromDb, buffLen, bgVec);

        // Reset furnitures in DB
        if (status)
        {
            roadVecGen2_gp->setBgRoadVec(bgVec, bRevDir);
        }

        delete payloadFromDb;

        return true;
    }

    bool databaseServer::saveRoadVecToFile(IN string fileName,
        IN bool bRevDir/* = false*/)
    {
        // Get all vectors
        int32 totalBufLen = _MAX_ROAD_POINT_BYTES;
        uint8 *payloadFromDb = new uint8[totalBufLen];
        if(payloadFromDb == NULL)
        {
            return false;
        }

        uint8 *outBuffPtr = payloadFromDb;
        int32 outBuffOffset = 0;
        
        list<backgroundSectionData> bgVecOut;
        roadVecGen2_gp->getBgRoadVec(bgVecOut, bRevDir);

        // Convert back ground vectors to TLV
        convBgRoadVecToTlv(bgVecOut, payloadFromDb, &outBuffOffset);

        // Save furnitures to file
        FILE* fp = fopen(fileName.c_str(), "wb");
        if (fp == NULL)
        {
            delete payloadFromDb;
            return false;
        }

        fwrite(payloadFromDb, outBuffOffset, 1, fp);

        fclose(fp);

        delete payloadFromDb;

        return true;
    }

    bool databaseServer::saveRoadVecAndFurToKml(IN list<list<vector<point3D_t>>> &allLines,
        										IN list<list<furAttributesServer_t>> &furnitureList,
												IN std::string fileName, 
												IN point3D_t &standPoint)
    {
		//
        //list<list<vector<point3D_t>>> allLines;
        //list<list<lineAttributes_t>>  lineAttr;
        //getAllVectors(allLines, lineAttr);
		//

        // Output to kml file
        int sectionIdx = 1;
        int furIdx = 1;

        double offsetX = 0;
        double offsetY = 0;
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
        // shift meters to match better with google earth
        offsetX = 1.5;
        offsetY = -2;
#endif

        FILE* fp = fopen(fileName.c_str(), "wt");
        if (fp == NULL)
        {
            return false;
        }

        // kml file header tags: <xml>, <kml>, <Document>
        fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml>\n<Document>\n");

        // tags: <Style>, <StyleMap>
        fprintf(fp, "    <Style id=\"greenLineStyle\"><LineStyle><color>ff00ff00</color></LineStyle></Style>\n");
        fprintf(fp, "    <StyleMap id=\"greenLine\"><Pair><key>normal</key><styleUrl>#greenLineStyle</styleUrl></Pair></StyleMap>\n");

        // Vectors
        // For each segment
        list<list<vector<point3D_t>>>::iterator lineIter = allLines.begin();
        while (lineIter != allLines.end())
        {
            int lineIdx = 1;

            // For each line
            list<vector<point3D_t>>::iterator lineInSegIter = lineIter->begin();
            while (lineInSegIter != lineIter->end())
            {
                // tags: <Placemark>, <name>
                fprintf(fp, "    <Placemark>\n        <name>Section-%d-Line-%d</name>\n", sectionIdx, lineIdx);
                
                // tag: styleUrl
                fprintf(fp, "        <styleUrl>#greenLine</styleUrl>\n");

                // tag: <LineString>
                fprintf(fp, "        <LineString>\n");

#if (KML_PAINT_ONLY_FLAG == ON)
                // tag: <altitudeMode>
                fprintf(fp, "            <altitudeMode>relativeToGround</altitudeMode>\n");
#endif

                // tag: <coordinates>
                fprintf(fp, "            <coordinates>\n                ");

                // For each point
                vector<point3D_t>::iterator pointIter = lineInSegIter->begin();
                while (pointIter != lineInSegIter->end())
                {
                    double alt = 0;
                    
                    point3D_t outGpsPoint;

                    pointRelative3D_t relPoint;
                    relPoint.x = pointIter->lon + offsetX;
                    relPoint.y = pointIter->lat + offsetY;
                    calcGpsFromRelativeLocation(&standPoint, &relPoint, &outGpsPoint);
                    
#if (KML_PAINT_ONLY_FLAG == ON)
                    if (pointIter->paintFlag < 0.5)
                        alt = -0.5;
                    else
                        alt = 0;
#endif

                    fprintf(fp, "%.7f,%.7f,%f ", outGpsPoint.lon, outGpsPoint.lat, alt);

                    ++pointIter;
                }

                fprintf(fp, "\n            </coordinates>\n        </LineString>\n    </Placemark>\n");

                ++lineInSegIter;
                ++lineIdx;
            }

            ++lineIter;
            ++sectionIdx;
        }

        // Traffic signs
        // For each segment
        list<list<furAttributesServer_t>>::iterator furIter = furnitureList.begin();
        while (furIter != furnitureList.end())
        {
            // For each furniture
            list<furAttributesServer_t>::iterator furInSegIter = furIter->begin();
            while (furInSegIter != furIter->end())
            {
                fprintf(fp, "    <Placemark>\n        <name>Sign-%d</name>\n        <Point>\n            <coordinates>\n                ", furIdx);

                double alt = 0;
                point3D_t outGpsPoint;

                pointRelative3D_t relPoint;
                relPoint.x = furInSegIter->location.lon + offsetX;
                relPoint.y = furInSegIter->location.lat + offsetY;
                calcGpsFromRelativeLocation(&standPoint, &relPoint, &outGpsPoint);

                fprintf(fp, "%.7f,%.7f,%f", outGpsPoint.lon, outGpsPoint.lat, alt);

                fprintf(fp, "\n            </coordinates>\n        </Point>\n    </Placemark>\n");

                ++furIdx;
                ++furInSegIter;
            }

            ++furIter;
        }

        // End tags
        fprintf(fp, "</Document>\n</kml>");

        fclose(fp);

        return true;
    }

    void databaseServer::convBgRoadVecToTlv(IN list<backgroundSectionData> &bgVec, 
                                            OUT uint8 *tlvBuf, 
                                            OUT int *bufLen)
    {
        WaitForSingleObject(_hMutexMemory, INFINITE);

        tlvCommon_t tlvTmp;
        tlvCfg_t* tlvCfgP;
        int byteNum = 0;

        resource_e sourceFlag = memory_e;
        void **output = (void **) &tlvBuf;

        // Segment Number
        tlvCfgP = &_tlvCfg_vec_a[vec_segNum_e - vec_base_e];
        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, bgVec.size());
        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

        list<backgroundSectionData>::iterator bgVecIter = bgVec.begin();
        // for each section
        while (bgVecIter != bgVec.end())
        {
            // Segment ID
            tlvCfgP = &_tlvCfg_vec_a[vec_segId_e - vec_base_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, bgVecIter->sectionId);
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            // Lane Num
            tlvCfgP = &_tlvCfg_vec_a[vec_laneNum_e - vec_base_e];
            setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, bgVecIter->bgSectionData.size());
            byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

            list<list<vector<point3D_t>>>::iterator bgInSegIter = bgVecIter->bgSectionData.begin();
            // for each lane
            while (bgInSegIter != bgVecIter->bgSectionData.end())
            {
                // Vector list
                tlvCfgP = &_tlvCfg_vec_a[vec_vecList_e - vec_base_e];
                setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, bgInSegIter->size(), 0);
                byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                list<vector<point3D_t>>::iterator bgInLaneIter = bgInSegIter->begin();
                // for each line
                while (bgInLaneIter != bgInSegIter->end())
                {
                    int numPoints = bgInLaneIter->size();

                    // Point number
                    tlvCfgP = &_tlvCfg_dataLine_a[data_linePointList_e - data_lineBase_e];
                    setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, numPoints, 0);
                    byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                    vector<point3D_t>::iterator bgPointIter = bgInLaneIter->begin();
                    // for each point
                    while (bgPointIter != bgInLaneIter->end())
                    {
                        point3D_t pointTmp = (*bgPointIter);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLatitude_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)(&(pointTmp.lat)));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointLongitude_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)(&(pointTmp.lon)));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointAltitude_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, (uint32)(&(pointTmp.alt)));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointPaintFlag_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, *(uint32*)(&pointTmp.paintFlag));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_mergeCounter_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, pointTmp.count);
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        tlvCfgP = &_tlvCfg_dataPoint_a[data_pointPaintLength_e - data_pointBase_e];
                        setTvlCommon(&tlvTmp, tlvCfgP->typeId, 1, tlvCfgP->tlvType, tlvCfgP->length, *(uint32*)&(pointTmp.paintLength));
                        byteNum += writeTlvCommon(&tlvTmp, output, sourceFlag);

                        ++bgPointIter;
                    }

                    ++bgInLaneIter;
                }

                ++bgInSegIter;
            }

            ++bgVecIter;
        }

        *bufLen = byteNum;

        ReleaseMutex(_hMutexMemory);
    }

    bool databaseServer::convTlvToBgRoadVec(IN  uint8 *tlvBuf,
                                            IN  int bufLen,
                                            OUT list<backgroundSectionData> &bgVec)
    {
        WaitForSingleObject(_hMutexMemory, INFINITE);

        tlvCommon_t tlvTmp;
        uint32 segmentId = 0;

        vector<point3D_t> vectorElement;
        list<vector<point3D_t>> laneElement;
        list<list<vector<point3D_t>>> segmentElement;
        backgroundSectionData bgElement;

        bgVec.clear();

        // Read TLV from DB file
        typeId_e idBase;
        int numByteRead;
        
        _mEndPosition = tlvBuf + bufLen;

        void** input = (void**) &tlvBuf;
        resource_e sourceFlag = memory_e;

        logPosition(input, sourceFlag);

        // Segment Number
        readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
        if(tlvTmp.typeId != vec_segNum_e)
        {
            ReleaseMutex(_hMutexMemory);
            return false;
        }

        int segNum = tlvTmp.value;
        for(int segIdx = 0; segIdx < segNum; ++segIdx)
        {
            // Segment ID
            logPosition(input, sourceFlag);
            readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
            if(tlvTmp.typeId != vec_segId_e)
            {
                ReleaseMutex(_hMutexMemory);
                return false;
            }

            int segId = tlvTmp.value;

            // Lane Num
            logPosition(input, sourceFlag);
            readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
            if(tlvTmp.typeId != vec_laneNum_e)
            {
                ReleaseMutex(_hMutexMemory);
                return false;
            }

            int laneNum = tlvTmp.value;

            for(int laneIdx = 0; laneIdx < laneNum; ++laneIdx)
            {
                // Vector list
                logPosition(input, sourceFlag);
                readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
                if(tlvTmp.typeId != vec_vecList_e)
                {
                    ReleaseMutex(_hMutexMemory);
                    return false;
                }

                int vecNum = tlvTmp.length;

                for(int vecIdx = 0; vecIdx < vecNum; ++vecIdx)
                {
                    // Point number
                    logPosition(input, sourceFlag);
                    readTlv(input, sourceFlag, &tlvTmp, &idBase, &numByteRead, _dataTmpBuf);
                    if(tlvTmp.typeId != data_linePointList_e)
                    {
                        ReleaseMutex(_hMutexMemory);
                        return false;
                    }

                    int pointNum = tlvTmp.length;

                    for(int pointIdx = 0; pointIdx < pointNum; ++pointIdx)
                    {
                        int numBytes;
                        point3D_t point;

                        readTlvToPoint(input, sourceFlag, point, &numBytes);

                        vectorElement.push_back(point);
                    }

                    laneElement.push_back(vectorElement);
                    vectorElement.clear();
                }

                segmentElement.push_back(laneElement);
                laneElement.clear();
            }

            bgElement.sectionId = segId;
            bgElement.bgSectionData = segmentElement;
            segmentElement.clear();

            bgVec.push_back(bgElement);
        }

        ReleaseMutex(_hMutexMemory);
        return true;
    }
}
