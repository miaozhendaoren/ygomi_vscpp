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
#pragma comment(lib,"winmm.lib")

unsigned int __stdcall Thread_Master_DBAccess(void *data)
{
    while(1)
    {
        WaitForSingleObject(g_readySema_msgQueue,INFINITE);
        // check message queue is empty
        if( !messageQueue_gp->empty())
        {
            //uint64 vehicleId;
            int32 msgType;
            diffRptMsg_t* diffRptMsgPtr;
            messageProcessClass currentMsg;
            messageQueue_gp->front(&currentMsg);
            
            diffRptMsgPtr = currentMsg.getDiffRptMsg();
            msgType = diffRptMsgPtr->msgHeader.msgTypeID;

            diffRptMsg_t* diffMsgPtr = currentMsg.getDiffRptMsg();
            diffRptMsg_t* updateMsgPtr = currentMsg.getUpdateRptMsg();
            memcpy((void*)updateMsgPtr,(void*)diffMsgPtr,sizeof(diffRptMsg_t));

            uint32 outBuffOffset = 0;

            if( (msgType & 0x1000) == 0x0000)
            {
                int pduIdx;
                switch(msgType)
                {
                case STATUS_UPDATE_RPT_MSG:
                    
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
                            }
                            // for debug and test client
                            currentMsg.setMessageLength((void*)updateMsgPtr,outBuffOffset);
                            databaseQueue_gp->push(&currentMsg);
                            break;
                        case locationAreaId_e:
                            break;
                        case databaseVersion_e:
                            {
                                //get the server's databaseVersion and compared with vehicle's,if different, send the different elements to vehicle.
                                int16 version = database_gp->getFurnitureVersion();
                                if(value != version)
                                {
                                    //get the all furniture TLV and send to vehicle's;
                                    uint64 vehicleId;
                                    int updatePduIdx, updatePduOffset = 0;;
                                    currentMsg.getVehicleIDInMsg((uint32*)&currentMsg,&vehicleId);
                                    updateMsgPtr->msgHeader.msgTypeID = UPDATE_REQ_MSG;
                                    updateMsgPtr->msgHeader.numPDUs = 1;
                                    updateMsgPtr->msgHeader.priority = highLevel_e;
                                    updateMsgPtr->msgHeader.vehicleID = vehicleId;
                                    
                                    database_gp->syncFurnitureToVehicle(updateMsgPtr->payload, &(updateMsgPtr->msgHeader.length), &(updateMsgPtr->msgHeader.numPDUs), MAX_PAYLOAD_BYTE_NUM);

                                    for(updatePduIdx=0; updatePduIdx<updateMsgPtr->msgHeader.numPDUs; updatePduIdx++)
                                    {
                                        updateMsgPtr->payloadHeader.pduHeader[updatePduIdx].operate = addDatabase_e;
                                        updateMsgPtr->payloadHeader.pduHeader[updatePduIdx].pduType = furnitureList_e;
                                        updateMsgPtr->payloadHeader.pduHeader[updatePduIdx].pduOffset = updatePduOffset;
                                        updatePduOffset += (updateMsgPtr->msgHeader.length) / (updateMsgPtr->msgHeader.numPDUs);
                                    }
                                
                                    // for debug and test client
                                    //currentMsg.setMessageLength((void*)updateMsgPtr,outBuffOffset);
                                    databaseQueue_gp->push(&currentMsg);
                                }
                            }
                            
                        }
                    }
                    break;
                case DIFF_RPT_MSG:
                    uint8 *outBuffPtr = updateMsgPtr->payload;
                    
                    for(pduIdx = 0;pduIdx < diffRptMsgPtr->msgHeader.numPDUs;pduIdx++)
                    {
                        int pduLen;
                        uint32 type = (diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].operate << 16) + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduType;
                        if(pduIdx == (diffRptMsgPtr->msgHeader.numPDUs - 1))
                        {
                            pduLen = diffRptMsgPtr->msgHeader.length - diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
                        }
                        else
                        {
                            pduLen = diffRptMsgPtr->payloadHeader.pduHeader[pduIdx+1].pduOffset - diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
                        }

                        // add to  database

                        uint32 outLen;
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
                                    database_gp->addFurnitureTlv(pduStartPtr,pduLen,outBuffPtr,&outLen);
                                    
                                    updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset = outBuffOffset;
                                    outBuffPtr += outLen;
                                    outBuffOffset += outLen;

                                    PlaySound("./resource/sound/detect.wav",NULL, SND_FILENAME|SND_ASYNC);
                                }
                                break;
                            /*case 0x00000005:
                                break;*/
                            case ADD_ALL_VECTORLIST:
                                // add a vector list for specified segment.
                                database_gp->addAllVectorsInSegTlv(diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                break;

                            case REDUCE_ONE_FURNITURE:
                                {
                                    uint8 *pduStartPtr = diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
                                    database_gp->reduceFurnitureTlv(pduStartPtr,pduLen,outBuffPtr,&outLen);
                                    
                                    updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset = outBuffOffset;
                                    outBuffPtr += outLen;
                                    outBuffOffset += outLen;

                                    PlaySound("./resource/sound/reduce.wav",NULL, SND_FILENAME|SND_ASYNC);
                                }
                                break;
                        }
                    }
                    // for debug and test client
                    currentMsg.setMessageLength((void*)updateMsgPtr,outBuffOffset);
                    databaseQueue_gp->push(&currentMsg);
                    break;
                }
            }
            messageQueue_gp->pop();
            ReleaseSemaphore(g_readySema_readDb,1,NULL);
            ReleaseSemaphore(g_readySema_Redraw,1,NULL);
        }
    }
    return 0;
}