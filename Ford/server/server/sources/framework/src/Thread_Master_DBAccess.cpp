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
#include "RoadVecGen.h" // CRoadVecGen
#include "LogInfo.h"  // logPrintf

#pragma comment(lib,"winmm.lib")

using namespace ns_database;
using namespace laneSpace;
laneQueueClass laneQueueBuff;
void storeLaneInfoInBuffer(uint8* tlvBuff,int bufferLen);
bool processLaneBuffer();
void calcMsgPduHeaderLen(messageProcessClass *msgIn,uint32 *pduHeaderLen);
bool checkLineFusionCondition(queue<laneType_t> *laneQueuePtr);

extern uint8 *test;

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

        // check message queue is empty
        if( !messageQueue_gp->empty())
        {
            //uint64 vehicleId;
            int32 msgType;
            diffRptMsg_t* diffRptMsgPtr;
			messageProcessClass sendMsg;
            messageProcessClass currentMsg;
            messageQueue_gp->front(&currentMsg);
			messageQueue_gp->pop();
     
            diffRptMsgPtr = currentMsg.getDiffRptMsg();
            msgType = diffRptMsgPtr->msgHeader.msgTypeID;

            diffRptMsg_t* diffMsgPtr = currentMsg.getDiffRptMsg();
            diffRptMsg_t* updateMsgPtr = sendMsg.getUpdateRptMsg();
            memcpy((void*)updateMsgPtr,(void*)diffMsgPtr,sizeof(diffMsgHeader_t));

            if( (msgType & 0x1000) == 0x0000)
            {
                int pduIdx;
                switch(msgType)
                {
					case STATUS_UPDATE_RPT_MSG:
					{    
						for(pduIdx = 0;pduIdx < diffRptMsgPtr->msgHeader.numPDUs;pduIdx++)
						{
							int8 tag = diffRptMsgPtr->payloadHeader.tlvArray[pduIdx].tag;
							int16 value = diffRptMsgPtr->payloadHeader.tlvArray[pduIdx].value;
							switch(tag)
							{
								case communitationStatus_e:
									break;
								case resetDatabaseFurniture_e:
									if(3 == value)
									{
										// delete furniture
										database_gp->resetFurniture();
										// delete all vectors
										list<list<vector<point3D_t>>> allLines;
										list<list<lineAttributes_t>>  lineAttr;
										list<vector<point3D_t>> newDataVec;

										database_gp->resetAllVectors(allLines, lineAttr);
										database_gp->setNewDataVec(newDataVec);
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
									/*if(value != version)
									{
										//get the all furniture TLV and send to vehicle's;
										uint32 vehicleId;
										
										currentMsg.getVehicleIDInMsg((uint32*)&currentMsg,&vehicleId);
										updateMsgPtr->msgHeader.msgTypeID = UPDATE_REQ_MSG;
										updateMsgPtr->msgHeader.priority = highLevel_e;
										updateMsgPtr->msgHeader.vehicleID = vehicleId;
                                    
										updateMsgPtr->payload = new uint8[MAX_PAYLOAD_BYTE_NUM];
										//get the payload memory address and pdu info.
                                        int32 msgLen, numPdu;
										database_gp->syncFurnitureToVehicle(updateMsgPtr->payload , &msgLen, &numPdu, MAX_PAYLOAD_BYTE_NUM);
										updateMsgPtr->msgHeader.payloadLen = msgLen;
                                        updateMsgPtr->msgHeader.numPDUs = numPdu;

										int updatePduOffset = 0;
										for(int updatePduIdx=0; updatePduIdx<updateMsgPtr->msgHeader.numPDUs; updatePduIdx++)
										{
											updateMsgPtr->payloadHeader.pduHeader[updatePduIdx].operate = addDatabase_e;
											updateMsgPtr->payloadHeader.pduHeader[updatePduIdx].pduType = furnitureList_e;
											updateMsgPtr->payloadHeader.pduHeader[updatePduIdx].pduOffset = updatePduOffset;
											updatePduOffset += (updateMsgPtr->msgHeader.payloadLen) / (updateMsgPtr->msgHeader.numPDUs);
										}
										updateMsgPtr->msgHeader.headerLen = sizeof(diffMsgHeader_t) + updateMsgPtr->msgHeader.numPDUs * sizeof(updateMsgPtr->payloadHeader.pduHeader[0]);
										databaseQueue_gp->push(&sendMsg);
										ReleaseSemaphore(g_readySema_readDb,1,NULL);
									}*/
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
										storeLaneInfoInBuffer(pduStartPtr,pduLen);
										laneProcessFlag = true;
									}
									break;
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
                            int32 totalBufLen = MAX_PAYLOAD_BYTE_NUM + MAX_ROAD_POINT_BYTES;
						    uint8 *payloadFromDb = new uint8[totalBufLen];
						    if(payloadFromDb == NULL)
						    {
							    break;
						    }

						    uint8 *outBuffPtr = payloadFromDb;
                            int32 outBuffOffset = 0;
                            int32 outPduIdx = 0;
                            bool updateFlag = false;

						    // furntiure update message
						    if(furUpdateVec.size() > 0)
						    {
                                for(int furIdx = 0; furIdx < furUpdateVec.size(); ++furIdx)
                                {
                                    furAttributes_t furAttr = furUpdateVec[furIdx];
                                    furAttributes_t furAttrOut;

                                    database_gp->addFurniture(&furAttr, &furAttrOut);
                                }

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

                                updateFlag = true;
                            }

						    // check which lane queue need to fusion
						    if(laneProcessFlag == true)
						    {
							    if(processLaneBuffer())
							    {
		                            int payloadBytes = 0;

                                    // Get updated road vector from database
                                    uint8* bufferAddr = outBuffPtr;
                                    int32 outBuffLen;
                                    database_gp->getAllVectorsTlv(  memory_e, 
                                                                    (void**)&bufferAddr, 
                                                                    &outBuffLen);

                                    if (outBuffLen > MAX_ROAD_POINT_BYTES)
                                    {
                                        logPrintf(logLevelCrit_e, "DBAccess", "Buffer overflow!", FOREGROUND_RED);
                                    }

                                    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].operate = addDatabase_e;
                                    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduType = vectorList_e;
                                    updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduOffset = outBuffOffset;

                                    outBuffPtr += outBuffLen;
                                    outBuffOffset += outBuffLen;

                                    ++outPduIdx;

                                    updateFlag = true;
							    }
						    }

                            // pack the update message
                            if(updateFlag)
                            {
                                // Convert all furniture to TLVs
                                int32 segNumOfFur;
                                database_gp->getSegNumOfFurniture(&segNumOfFur);

                                for(int segIndex = 0; segIndex < segNumOfFur; ++segIndex)
                                {
                                    uint8* bufferAddr = outBuffPtr;
                                    int32 furMsgLen;
                                    int32 furNum;

                                    database_gp->getFurnitureTlvInSeg(segIndex,
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

                                int numPDUs = outPduIdx;

                                updateMsgPtr->msgHeader.numPDUs = numPDUs;
							    updateMsgPtr->msgHeader.headerLen = sizeof(updateMsgPtr->msgHeader) + numPDUs*sizeof(updateMsgPtr->payloadHeader.pduHeader[0]);
							    updateMsgPtr->msgHeader.msgTypeID = UPDATE_REQ_MSG;
							    updateMsgPtr->msgHeader.priority = highLevel_e;
                                updateMsgPtr->msgHeader.payloadLen = outBuffOffset;
							    updateMsgPtr->payload = payloadFromDb;
							    databaseQueue_gp->push(&sendMsg);
							    ReleaseSemaphore(g_readySema_readDb,1,NULL);
							    ReleaseSemaphore(g_readySema_Redraw,1,NULL);
                            }else 
						    {
							    delete payloadFromDb;
							    payloadFromDb = NULL;
						    }
                        }
					}
                    break;
                }
				delete diffRptMsgPtr->payload;
				diffRptMsgPtr->payload = NULL;
            }
        }
    }
    return 0;
}
void storeLaneInfoInBuffer(uint8* tlvBuff,int bufferLen)
{
	void*  inputLoc = tlvBuff;
    void** input = &inputLoc;
	list<laneType_t> laneInfoList;
	database_gp->readTlvToLaneInfo(input,memory_e,bufferLen,&laneInfoList);
    
    while(laneInfoList.size() > 0)
    {
        laneType_t laneInfoTemp = laneInfoList.front();
	    laneQueueBuff.addLanePoint(0,laneInfoTemp); // Always push data to lane 0 // laneInfoTemp.laneId
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
        laneQueueBuff.getAllVectors(newDataList);

        list<list<vector<point3D_t>>> fgData;
        bool flag = roadVecGen_gp->roadSectionsGen(newDataList, fgData);

        if (flag == false)
        {
            return false;
        }else
        {
            list<list<lineAttributes_t>> lineAttr;
            database_gp->resetAllVectors(fgData, lineAttr);
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
