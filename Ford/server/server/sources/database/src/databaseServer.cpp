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

        if (reliabRating > 5)
        {
            reliabRating = 5;
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

        uint8 segExistFlag = 0;
        segAttributes_t segAttr;

        //getSegmentByGps(gpsP, &segExistFlag, &segAttr); // FIXME: replace by new get segment func

        if(segExistFlag == 1)
        {
            uint8 furExistFlag = 0;

            list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();
            
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

                list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

                // For each furniture in the segment
                while(furIter != (*segIter).end())
                {
                    
                    if((*furIter).location_used == 1)
                    {
                        point3D_t gpsTmp = (*furIter).location;

                        // Push furniture to output list if in GPS range
                        if(checkGpsInRange(&gpsTmp, gpsP, distThreshMeter))
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
        double height = floor(locIn->alt * 4.0 + 0.5); // x4 so the height is from 0 to 4 meter with step 1 meter

        // No sign on the ground
        if(height == 0)
        {
            height = 1;
        }

        // Check if the height is same with other furnitures in the same location
        list<furAttributesServer_t> furAttrNearby;
        getFurnitureByGps(locIn, 5.0, furAttrNearby); // 5.0m
        int heightArray[20] = {0};

        if(furAttrNearby.size() > 0)
        {
            list<furAttributesServer_t>::iterator furIter = furAttrNearby.begin();
            while(furIter != furAttrNearby.end())
            {
                if(((*furIter).sideFlag_used == 1) && 
                    ((*furIter).sideFlag == sideFlagIn))
                {
                    int heightIdx = (int)(*furIter).location.alt;
                    heightArray[heightIdx] ++;
                }

                furIter++;
            }

            while(heightArray[(int)height])
            {
                height += 1;
            }
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

    void databaseServer::addFurniture(IN furAttributes_t* furnitureIn,
                                      OUT furAttributes_t* furnitureOut)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

        furAttributesServer_t furnitureServerIn(furnitureIn);

        // Check if same furniture exist in DB
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
        }

        if (furIdFoundFlag == true)
        // Update existing one
        {
            (*furIter).update(&furnitureServerIn);

            double altOut;
            calcFurnitureHeight(&furnitureServerIn.location, furnitureServerIn.sideFlag, &altOut);
            (*furIter).location.alt = altOut;
			furnitureServerIn.location.alt = altOut;
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
                (*furIter).update(&furnitureServerIn);

                double altOut;
                calcFurnitureHeight(&furnitureServerIn.location, furnitureServerIn.sideFlag, &altOut);
                (*furIter).location.alt = altOut;
				furnitureServerIn.location.alt = altOut;
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
                /*
                uint8 segLocated = 0;
                segAttributes_t segAttr;
                getSegmentByGps(&furnitureServerIn.location, &segLocated, &segAttr);

                if (segLocated == 0)
                // Failed to found the segment by GPS of the furniture
                {
                    ReleaseMutex(_hMutexMemory);
                    logPrintf(logLevelError_e, "DATABASE", "Furniture location not in any segment", FOREGROUND_RED);
                    return;
                }

                furnitureServerIn.segId_used = 1;
                furnitureServerIn.segId = segAttr.segId;
                */

                // Calculate altitude from input height
                double altOut;
                calcFurnitureHeight(&furnitureServerIn.location, furnitureServerIn.sideFlag, &altOut);
                furnitureServerIn.location.alt = altOut;

                // Check Segment ID in furniture list
                bool segIdFound = false;
                segIter = _furnitureList.begin();
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
                }
                else
                // Create a new furniture list
                {
                    std::list<furAttributesServer_t> furListTmp;
                    furListTmp.push_back(furnitureServerIn);

                    _furnitureList.push_back(furListTmp);
                }
            }

        }

        *furnitureOut = furnitureServerIn;

        if (furnitureServerIn.segId_used == 0)
        {
            int stop = 1;
        }

        ReleaseMutex(_hMutexMemory);
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

            string furTypeString = "Synchronize furnitures in segment ID ";  furTypeString += segIdIn;
            logPrintf(logLevelInfo_e, "DB_SYNC", furTypeString, FOREGROUND_BLUE | FOREGROUND_GREEN);
        }
        
        *msgLenOut = payloadLen;
        *furNumOut = furNum;

        ReleaseMutex(_hMutexMemory);
    }

	uint32 databaseServer::getFurnitureVersion()
    {    
		uint32 version = 0xFFFE; // default version
				
		if(!(_furnitureList.empty()))
		{
			WaitForSingleObject(_hMutexMemory,INFINITE);

			list<list<furAttributesServer_t>>::iterator segIter = _furnitureList.begin();
			list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

			if((*furIter).segVersion_used == 1)
			{
				version = (*furIter).segVersion;
			}
			else
			{
				version = 0;
			}

			ReleaseMutex(_hMutexMemory);
		}
		
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

	void databaseServer::sectionProc(IN std::list<segAttributes_t> &sectionConfig, 
			IN std::list<std::vector<point3D_t>> &newData,
			//IN std::list<std::list<furAttributesServer_t>> &dataBaseLandmark,
			IN std::list<std::list<std::vector<point3D_t>>> &dataBasePoint,
			OUT std::list<std::list<std::vector<point3D_t>>> &dataBasePointMerged
			//OUT std::list<std::list<furAttributesServer_t>> &dataBaseLandmarkMerged
       )
	{
		return;
	}
}
