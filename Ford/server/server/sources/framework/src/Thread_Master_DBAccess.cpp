/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_Master_DBAccesss.cpp
* @brief process all the operations with server'sdatabase.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#include <Windows.h>

#include "databaseServer.h" // databaseServer
#include "appInitCommon.h"
#include "laneQueueClass.h"
#include "RoadVecGen2.h" // CRoadVecGen2
#include "LogInfo.h"  // logPrintf
#include "configure.h"
#include "TimeStamp.h"

#pragma comment(lib,"winmm.lib")

using namespace ns_database;
using namespace laneSpace;
laneQueueClass laneQueueBuff;
void storeLaneInfoInBuffer(uint8* tlvBuff,int bufferLen,int laneId);
bool processLaneBuffer();
void calcMsgPduHeaderLen(messageProcessClass *msgIn,uint32 *pduHeaderLen);
bool checkLineFusionCondition(queue<laneType_t> *laneQueuePtr);
bool prepareAllDataToVehicle(IN messageProcessClass &sendMsg);
void preprocStopLines(INOUT vector<furAttributes_t> &furUpdateVec);

extern uint8 *test;

#if (RD_LOCATION == RD_US_DETROIT)
	point3D_t standPoint={42.296855933108084, -83.213250649943689, 0, 0, 0, 0};
#elif (RD_LOCATION == RD_GERMAN_LEHRE)
	point3D_t standPoint = {52.352999550000001, 10.693509536666666, 0, 0, 0, 0};
#elif (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
	point3D_t standPoint = {48.354788999999997, 11.758086000000000, 0, 0, 0, 0};
#elif (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
    point3D_t standPoint = { 48.350662000000000, 11.733637999999999, 0, 0, 0, 0};
#elif (RD_LOCATION == RD_US_PALO_ALTO)
	point3D_t standPoint={37.39630022, -122.05374589, 0, 0, 0, 0};
#elif (RD_LOCATION == RD_US_MOUNTAINVIEW)
	point3D_t standPoint = { 37.398907028, -122.039406609, 0, 0, 0, 0};
#else
	#error ("undfined RD_LOCATION");
#endif

#if SERVER_PLAY_BACK_MODE==1
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace laneSpace;
using namespace ns_database;
#endif

unsigned int __stdcall Thread_Master_DBAccess(void *data)
{
#if SERVER_PLAY_BACK_MODE==1
    std::stringstream fileName;
    fileName << "log/messages.bin";
    FILE *fpMsg = fopen(fileName.str().c_str(), "rb");
#endif

    while(1)
    {
#if SERVER_PLAY_BACK_MODE==0
        WaitForSingleObject(g_readySema_msgQueue,INFINITE);
#else
        messageProcessClass recvMsg;

        diffRptMsg_t *diffMsg = recvMsg.getDiffRptMsg();
        
        uint8* recvBuffP = (uint8*)diffMsg;

        // message header size
        fread(&(diffMsg->msgHeader.headerLen), sizeof(diffMsg->msgHeader.headerLen), 1, fpMsg);

        recvBuffP += sizeof(diffMsg->msgHeader.headerLen);

        // message header
        fread(recvBuffP, diffMsg->msgHeader.headerLen - sizeof(diffMsg->msgHeader.headerLen), 1, fpMsg);

        // message payload
        diffMsg->payload = new uint8[diffMsg->msgHeader.payloadLen];

        fread(diffMsg->payload, diffMsg->msgHeader.payloadLen, 1, fpMsg);

        messageQueue_gp->push(&recvMsg);

		// print out message count infor
		static int msgCount = 1;

		cout << "message Count = " << msgCount++ << endl;
#endif
		RD_ADD_TS(tsFunId_eThread_DBUpdate,1);
        // check message queue is empty
        if( !messageQueue_gp->empty())
        {
            //uint64 vehicleId;
            int32 msgType;
            diffRptMsg_t* diffRptMsgPtr;
			messageProcessClass sendMsg;
            messageProcessClass currentMsg;
            messageQueue_gp->top(&currentMsg);
			messageQueue_gp->pop();
     
            diffRptMsgPtr = currentMsg.getDiffRptMsg();
            msgType = diffRptMsgPtr->msgHeader.msgTypeID;

            diffRptMsg_t* diffMsgPtr = currentMsg.getDiffRptMsg();
            diffRptMsg_t* updateMsgPtr = sendMsg.getUpdateRptMsg();
            memcpy((void*)updateMsgPtr,(void*)diffMsgPtr,sizeof(diffMsgHeader_t));

			RD_ADD_TS(tsFunId_eThread_DBUpdate,2);
            if( (msgType & 0x1000) == 0x0000)
            {
                int pduIdx;
                switch(msgType)
                {
					case STATUS_UPDATE_RPT_MSG:
					{    
						RD_ADD_TS(tsFunId_eThread_DBUpdate,3);
						for(pduIdx = 0;pduIdx < diffRptMsgPtr->msgHeader.numPDUs;pduIdx++)
						{
							int8 tag = diffRptMsgPtr->payloadHeader.tlvArray[pduIdx].tag;
							int16 value = diffRptMsgPtr->payloadHeader.tlvArray[pduIdx].value;
							switch(tag)
							{
								case loadSaveDatabase_e:
									if(1 == value) //save
									{
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
                                        roadVecGen2_gp->BGSectionDataSave("./log/airportBackSave/");
#else
                                        database_gp->saveRoadVecToFile("log/roadVecSave.bin");
                                        database_gp->saveRoadVecToFile("log/roadVecRevDirSave.bin", true);
#endif

                                        database_gp->saveFurToFile("log/furnitureSave.bin");
                                        
										list<list<vector<point3D_t>>> allLines;
										list<list<lineAttributes_t>>  lineAttr;
                                        list<list<furAttributesServer_t>> furnitureList;
										database_gp->getAllVectors(allLines, lineAttr);
                                        database_gp->getAllFurnitures(furnitureList);
                                        database_gp->saveRoadVecAndFurToKml(allLines, furnitureList, "log/roadVec_furniture.kml", standPoint);

									}else if(2 == value) //load
									{
#if(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
                                        roadVecGen2_gp->BGSectionDataLoad( "./log/airportBackLoad/");
#else
                                        database_gp->loadRoadVecFromFile("log/roadVec.bin");
									    database_gp->loadRoadVecFromFile("log/roadVecRevDir.bin", true);
                                        #if(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
										roadVecGen2_gp->loadDefaultSegData(11, "log/section11.txt");// 11: empty section ID
                                        #endif
#endif
                                        database_gp->loadFurFromFile("log/furniture.bin");

                                        // Update front ground DB
                                        list<list<vector<point3D_t>>> fgData;
                                        bool flag = roadVecGen2_gp->roadSectionsGen(fgData);
                                        if (flag == true)
                                        {
                                            list<list<lineAttributes_t>> lineAttr;
                                            database_gp->resetAllVectors(fgData, lineAttr);
                                        }

                                        // Prepare messages to sync to vehicle
                                        if(prepareAllDataToVehicle(sendMsg))
                                        {
                                            // Semaphores
                                            database_gp->resetFurUpdateFlag();
                                            database_gp->resetUpateSectionIdList();
                                            ReleaseSemaphore(g_readySema_readDb,1,NULL);
                                            ReleaseSemaphore(g_readySema_Redraw,1,NULL);
                                        }
									}
									
								break;
								case communitationStatus_e:
									break;
								case resetDatabaseFurniture_e:
									if(3 == value)
									{
										// Reset all furnitures in database
										database_gp->resetFurniture();
                                        database_gp->resetFurUpdateFlag();

										// Reset all vectors in database
										list<list<vector<point3D_t>>> allLines;
										list<list<lineAttributes_t>>  lineAttr;
										list<vector<point3D_t>> newDataVec;

										database_gp->resetAllVectors(allLines, lineAttr);
                                        database_gp->resetUpateSectionIdList();

                                        // Reset new data vectors (red lines) in database
										database_gp->setNewDataVec(newDataVec);

                                        // Reset front ground and back ground road vectors in RoadVecGen
                                        roadVecGen2_gp->resetDatabase();
									}else if(4 == value)
									{
										// Reset all furnitures in database
										database_gp->resetFurniture();
                                        database_gp->resetFurUpdateFlag();
									}else if(5 == value)
									{
										// Reset all vectors in database
										list<list<vector<point3D_t>>> allLines;
										list<list<lineAttributes_t>>  lineAttr;
										list<vector<point3D_t>> newDataVec;

										database_gp->resetAllVectors(allLines, lineAttr);
                                        database_gp->resetUpateSectionIdList();

                                        // Reset new data vectors (red lines) in database
										database_gp->setNewDataVec(newDataVec);

                                        // Reset front ground and back ground road vectors in RoadVecGen
                                        roadVecGen2_gp->resetDatabase();
									}
									// for debug and test client
									updateMsgPtr->payloadHeader.tlvArray[pduIdx] = diffRptMsgPtr->payloadHeader.tlvArray[pduIdx];
									updateMsgPtr->msgHeader.headerLen = sizeof(updateMsgPtr->msgHeader) + sizeof(updateMsgPtr->payloadHeader.tlvArray[pduIdx]);
									updateMsgPtr->msgHeader.payloadLen = 0;
									updateMsgPtr->msgHeader.vehicleID = 0xFFFFFFFF;
									databaseQueue_gp->push(&sendMsg);
									ReleaseSemaphore(g_readySema_readDb,1,NULL);
									ReleaseSemaphore(g_readySema_Redraw,1,NULL);
									break;
								case locationAreaId_e:
									break;
								case databaseVersion_e:
								{
									//get the server's databaseVersion and compared with vehicle's,if different, send the different elements to vehicle.
									int16 version = database_gp->getFurnitureVersion();
									if(value != version)
									{
									    database_gp->addAllIdToUpdateIdList();
                                        database_gp->setFurUpdateFlag(1);
                                        /*if(prepareAllDataToVehicle(sendMsg))
                                        {
                                            database_gp->resetFurUpdateFlag();
                                            database_gp->resetUpateSectionIdList();
										    ReleaseSemaphore(g_readySema_readDb,1,NULL);
                                        }*/
									}
								}
								break;  
							}
						}
					}
                    break;
					case DIFF_RPT_MSG:
					{
						bool laneProcessFlag = false;
                        vector<furAttributes_t> furUpdateVec;
                        vector<furAttributes_t> furDeleteVec;
						RD_ADD_TS(tsFunId_eThread_DBUpdate,4);

						updateMsgPtr->msgHeader.vehicleID = 0xFFFFFFFF;
						for(pduIdx = 0;pduIdx < diffRptMsgPtr->msgHeader.numPDUs;pduIdx++)
						{
							int pduLen;
							uint32 type = (diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].operate << 16) + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduType;
							if(pduIdx == (diffRptMsgPtr->msgHeader.numPDUs - 1))
							{
								pduLen = diffRptMsgPtr->msgHeader.payloadLen - diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
							}
							else
							{
								pduLen = diffRptMsgPtr->payloadHeader.pduHeader[pduIdx+1].pduOffset - diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
							}

							// check PDU type
							switch(type)
							{
								case ADD_NEW_SEGMENT:// add a new segment data
									database_gp->addSegmentTlv(diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
									break;
								case ADD_NEW_FURNITURE:
									// fall through
								case UPDATE_FURNITURE:
									{
										uint8 *pduStartPtr = diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
                                        furAttributes_t furAttr;

                                        database_gp->readTlvToFurniturePublic(pduStartPtr, pduLen, furAttr);
                                        furUpdateVec.push_back(furAttr);

										PlaySound("./resource/sound/detect.wav",NULL, SND_FILENAME|SND_ASYNC);
									}
									break;
								case ADD_LANE_POINT:
									{
										uint8 *pduStartPtr = diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
										storeLaneInfoInBuffer(pduStartPtr,pduLen,0);
										laneProcessFlag = true;
									}
									break;
                                case SIDE_LANE_INFO:
                                    {
                                        uint8 *pduStartPtr = diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
										//TODO
                                        storeLaneInfoInBuffer(pduStartPtr,pduLen,1);
                                        break;
                                    }
								case ADD_ALL_VECTORLIST:
									// add a vector list for specified segment.
									database_gp->addAllVectorsInSegTlv(diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
									break;

								case REDUCE_ONE_FURNITURE:
									{
										uint8 *pduStartPtr = diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
                                        furAttributes_t furAttr;

                                        database_gp->readTlvToFurniturePublic(pduStartPtr, pduLen, furAttr);
                                        furDeleteVec.push_back(furAttr);

										PlaySound("./resource/sound/reduce.wav",NULL, SND_FILENAME|SND_ASYNC);
									}
									break;
							}
						}

                        // Convert info into TLV and prepare update message
                        {
                            bool updateFlag = false;

						    // check which lane queue need to fusion
						    if(laneProcessFlag == true)
						    {
							    if(processLaneBuffer())
							    {
							        uint32 updateListSize = database_gp->getUpateSectionIdListSize();
                                    if(0 < updateListSize)
                                    {
                                        int32 segNumOfFur;
                                        database_gp->getSegNumOfFurniture(&segNumOfFur);
                                        if(segNumOfFur > 0)
                                        {
                                            database_gp->resetFurnitureRoadSideLoc3();
                                            database_gp->setFurUpdateFlag(1);
                                        }
                                    }
                                    
                                    updateFlag = true;
							    }
						    }

							// furntiure update message

                            // Preprocess on the stop lines
                            // For US PALO ALTO temp use only!!
#if (RD_LOCATION == RD_US_PALO_ALTO) && (RD_ROAD_SIGN_DETECT & RD_ROAD_SIGN_DETECT_STOPLINE_MASK)
                            preprocStopLines(furUpdateVec);
#endif

						    if(furUpdateVec.size() > 0)
						    {
#if(KML_RECV_NEW_DATA == ON)
		                        SYSTEMTIME systime;
		                        GetLocalTime(&systime);
		                        std::string strKMLlfilename;
		                        char cKMLlfilename[30];
		                        sprintf(cKMLlfilename, "%d-%02d-%02d_%02d%02d%02d_%03d_furn",
			                        systime.wYear,
			                        systime.wMonth,
			                        systime.wDay,
			                        systime.wHour,
			                        systime.wMinute,
			                        systime.wSecond,
			                        systime.wMilliseconds);
		                        strKMLlfilename = cKMLlfilename;
		                        strKMLlfilename="log/" + strKMLlfilename + ".kml";

                                list<list<furAttributesServer_t>> furnitureList;
                                list<furAttributesServer_t> furnitureInSeg;
                                
                                for(int furIdx = 0; furIdx < furUpdateVec.size(); ++furIdx)
                                {
                                    furAttributes_t furAttr = furUpdateVec[furIdx];
                                    furAttributesServer_t furnSrv(&furAttr);
                                    furnitureInSeg.push_back(furnSrv);
                                }
                                furnitureList.push_back(furnitureInSeg);

                                list<list<vector<point3D_t>>> newDataList;
		                        database_gp->saveRoadVecAndFurToKml(newDataList, furnitureList, strKMLlfilename, standPoint);
#endif

                                for(int furIdx = 0; furIdx < furUpdateVec.size(); ++furIdx)
                                {
                                    furAttributes_t furAttr = furUpdateVec[furIdx];
                                    furAttributes_t furAttrOut;

                                    database_gp->addFurniture(&furAttr, &furAttrOut);
                                }

                                database_gp->setFurUpdateFlag(1);

                                updateFlag = true;
						    }

                            // furniture delete message
                            if(furDeleteVec.size() > 0)
                            {
                                for(int furIdx = 0; furIdx < furDeleteVec.size(); ++furIdx)
                                {
                                    furAttributes_t furAttr = furDeleteVec[furIdx];
                                    furAttributes_t furAttrOut;

                                    database_gp->reduceFurnitureByFurId(&furAttr, &furAttrOut);
                                }

                                database_gp->setFurUpdateFlag(1);

                                updateFlag = true;
                            }

                            // pack the update message
                            if(updateFlag)
                            {
							    ReleaseSemaphore(g_readySema_Redraw,1,NULL);
                            }
                        }
					}
                    break;
                }

                if(diffRptMsgPtr->msgHeader.payloadLen != 0)
                {
    				delete diffRptMsgPtr->payload;
    				diffRptMsgPtr->payload = NULL;
                }
            }
        }
		RD_ADD_TS(tsFunId_eThread_DBUpdate,14);
    }
    return 0;
}
void storeLaneInfoInBuffer(uint8* tlvBuff,int bufferLen,int laneId )
{
	void*  inputLoc = tlvBuff;
    void** input = &inputLoc;
	list<laneType_t> laneInfoList;
	database_gp->readTlvToLaneInfo(input,memory_e,bufferLen,&laneInfoList);
    
    while(laneInfoList.size() > 0)
    {
        laneType_t laneInfoTemp = laneInfoList.front();
	    laneQueueBuff.addLanePoint(laneId,laneInfoTemp); // Always push data to lane 0 // laneInfoTemp.laneId
        laneInfoList.pop_front();
    }
}
bool processLaneBuffer()
{
	vector<int> storeLane;
	//static int count = 0;
	for(int idx = 0;idx < MAX_LANE_NUM;++idx)
	{
		int laneGpsSize;
		queue<laneType_t> * laneInfoPtr = laneQueueBuff.getSpecifiedLane(idx);
		if(checkLineFusionCondition(laneInfoPtr))
		{
			storeLane.push_back(idx);
		}
	}
	if (storeLane.size() != 0)
	{
		// call fusion function and update database
        list<list<vector<point3D_t>>> newDataList;
        list<vector<point3D_t>> newDataGps;
        laneQueueBuff.getAllVectors(newDataList, newDataGps);

#if(KML_RECV_NEW_DATA == ON)
		SYSTEMTIME systime;
		GetLocalTime(&systime);
		std::string strKMLlfilename;
		char cKMLlfilename[30];
		sprintf(cKMLlfilename, "%d-%02d-%02d_%02d%02d%02d_%03d_road",
			systime.wYear,
			systime.wMonth,
			systime.wDay,
			systime.wHour,
			systime.wMinute,
			systime.wSecond,
			systime.wMilliseconds);
		strKMLlfilename = cKMLlfilename;
		strKMLlfilename="log/" + strKMLlfilename + ".kml";

        list<list<furAttributesServer_t>> furnitureList;
		database_gp->saveRoadVecAndFurToKml(newDataList, furnitureList, strKMLlfilename, standPoint);
#endif

        database_gp->setNewDataVec(*(newDataList.begin()));

        list<list<vector<point3D_t>>> fgData;

        list<uint32> modifiedIdList;

        bool flag = false;
        flag = roadVecGen2_gp->roadSectionsGen(newDataList, newDataGps, fgData, modifiedIdList);

        if (flag == false)
        {
            return false;
        }else
        {
            list<list<lineAttributes_t>> lineAttr;
            database_gp->resetAllVectors(fgData, lineAttr);
            
            database_gp->mergeIdListToUpdateIdList(modifiedIdList);
        }

		storeLane.clear();

        return true;
	}
	else
	{
		return false;
	}
}

bool checkLineFusionCondition(queue<laneType_t> *laneQueuePtr)
{
	return ((laneQueuePtr != NULL) && (laneQueuePtr->size() >= 0 ));//MAX_GPS_NUM_PER_LANE
}

bool prepareAllDataToVehicle(IN messageProcessClass &sendMsg)
{
    diffRptMsg_t* updateMsgPtr = sendMsg.getUpdateRptMsg();

    int32 totalBufLen = MAX_PAYLOAD_BYTE_NUM + MAX_ROAD_POINT_BYTES;
    uint8 *payloadFromDb = new uint8[totalBufLen];
    if(payloadFromDb == NULL)
    {
        return false;
    }

    uint8 *outBuffPtr = payloadFromDb;
    int32 outBuffOffset = 0;
    int32 outPduIdx = 0;

    // Get updated road vector from database
    uint8* bufferAddr = outBuffPtr;
    int32 outBuffLen;
    database_gp->getAllVectorsTlv(  memory_e, 
								    (void**)&bufferAddr, 
								    &outBuffLen);

    if (outBuffLen > MAX_ROAD_POINT_BYTES)
    {
	    logPrintf(logLevelCrit_e, "DBAccess", "Buffer overflow!", FOREGROUND_RED);
        return false;
    }

    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].operate = addDatabase_e;
    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduType = vectorList_e;
    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduOffset = outBuffOffset;

    outBuffPtr += outBuffLen;
    outBuffOffset += outBuffLen;

    ++outPduIdx;
                                        
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
			    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].operate = addDatabase_e;
			    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduType = furnitureList_e;
			    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduOffset = outBuffOffset;

			    outBuffPtr += furMsgLen;
			    outBuffOffset += furMsgLen;
			    ++outPduIdx;
		    }
	    }
    }

    int numPDUs = outPduIdx;

    updateMsgPtr->msgHeader.numPDUs = numPDUs;
    updateMsgPtr->msgHeader.headerLen = sizeof(updateMsgPtr->msgHeader) + numPDUs*sizeof(updateMsgPtr->payloadHeader.pduHeader[0]);
    updateMsgPtr->msgHeader.msgTypeID = UPDATE_REQ_MSG;
    updateMsgPtr->msgHeader.priority = highLevel_e;
    updateMsgPtr->msgHeader.payloadLen = outBuffOffset;
    updateMsgPtr->payload = payloadFromDb;
    databaseQueue_gp->push(&sendMsg);

    return true;
}

void preprocStopLines(INOUT vector<furAttributes_t> &furUpdateVec)
{
    list<furAttributes_t> stopLines;
    vector<furAttributes_t>::iterator furIter = furUpdateVec.begin();

    // Get all stop lines from furniture
    while (furIter != furUpdateVec.end())
    {
        // On the road furniture
        if ((furIter->sideFlag_used == 1) && (furIter->sideFlag == onTheRoad_e))
        {
            // If stop line
            if ((furIter->type_used == 1) && (furIter->type == 1000))
            {
                stopLines.push_back(*furIter);
                furIter = furUpdateVec.erase(furIter);
            }else
            {
                ++furIter;
            }
        }else
        {
            ++furIter;
        }
    }

    // Check number of stop lines in range
    list<furAttributes_t> stopLinesInRange;
    list<furAttributes_t>::iterator stoplineIter = stopLines.begin();
    while (stoplineIter != stopLines.end())
    {
        furAttributes_t firstStopline = *stoplineIter;
        stopLinesInRange.push_back(firstStopline);
        stoplineIter = stopLines.erase(stoplineIter);

        if (firstStopline.location_used == 1)
        {
            while (stoplineIter != stopLines.end())
            {
                if ((stoplineIter->location_used == 1) && 
                    (checkRelGpsInRange(&firstStopline.location, &stoplineIter->location, 40)))
                {
                    stopLinesInRange.push_back(*stoplineIter);
                    stoplineIter = stopLines.erase(stoplineIter);
                }else
                {
                    ++stoplineIter;
                }
            }

            if (stopLinesInRange.size() == 4)
            {
                const double LON_THRESH = -300;
                if (stopLinesInRange.begin()->location.lon > LON_THRESH)
                // East side
                {
                    // check vehicle moving direction
                    double lonBegin = stopLinesInRange.front().location.lon;
                    double lonEnd   = stopLinesInRange.back().location.lon;

                    if (lonBegin > lonEnd)
                    // forward
                    {
                        list<furAttributes_t>::iterator stopLinesInRangeIter = stopLinesInRange.end();
                        --stopLinesInRangeIter; // stop line
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                        --stopLinesInRangeIter;
                        stopLinesInRangeIter->type = 1001; // crosswalk
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                    }else
                    // reverse
                    {
                        list<furAttributes_t>::iterator stopLinesInRangeIter = stopLinesInRange.begin(); // stop line
                        stopLinesInRangeIter->type = 1002; // stop line
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                        ++stopLinesInRangeIter;
                        stopLinesInRangeIter->type = 1003; // crosswalk
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                    }
                }else
                // west side
                {
                    // check vehicle moving direction
                    double lonBegin = stopLinesInRange.front().location.lon;
                    double lonEnd   = stopLinesInRange.back().location.lon;

                    if (lonBegin > lonEnd)
                    // forward
                    {
                        list<furAttributes_t>::iterator stopLinesInRangeIter = stopLinesInRange.begin(); // stop line
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                        ++stopLinesInRangeIter;
                        stopLinesInRangeIter->type = 1001; // crosswalk
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                    }else
                    // reverse
                    {
                        list<furAttributes_t>::iterator stopLinesInRangeIter = stopLinesInRange.end();
                        --stopLinesInRangeIter; // stop line
                        stopLinesInRangeIter->type = 1002; // stop line
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                        --stopLinesInRangeIter;
                        stopLinesInRangeIter->type = 1003; // crosswalk
                        furUpdateVec.push_back(*stopLinesInRangeIter);
                    }
                }

            }
        }

        stopLinesInRange.clear();
        stoplineIter = stopLines.begin();
    }
}
