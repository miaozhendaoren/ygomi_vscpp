/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_DBUpdate.cpp
* @brief Call Digital Horizon Data Update Manager to update data to Digital Horizon Database
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*      2015/01/04         Qin Shi         Add the update DB function
*******************************************************************************
*/
#include <winsock.h>
#include "typeDefine.h"
#include "database.h"
#include "databaseInVehicle.h"
#include "messageProcessClass.h"
#include "LogInfo.h"
#include "AppInitCommon.h"


#pragma comment(lib, "ws2_32.lib") 

#define RECV_MAX_BYTE_NUM                10000

extern ns_database::databaseInVehicle* database_gp;
extern ns_database::databaseInVehicle* database_gpFinal;

unsigned int __stdcall Thread_DBUpdate(void *data)
{
    uint8 recvBuf[RECV_MAX_BYTE_NUM];

    while(1)
    {
        //receive server messages 
        uint16 errorCode = 0;
        uint16 erroNumPdu = 0;
        memset(recvBuf,0,sizeof(recvBuf));
        sockaddr_in from;
        int nRet;
        messageProcessClass messageProcess;
        nRet = recvfrom(sockClient, (char*)recvBuf, RECV_MAX_BYTE_NUM,0, (SOCKADDR*)&from,&g_SocketLen);
        if ((nRet == SOCKET_ERROR)|| (nRet == 0))
        {  
            logPrintf(logLevelError_e, "DB_UPDATE", "Receive message from socket failed!");
        } 
        else
        {
            int32 msgType;
            logPrintf(logLevelInfo_e, "DB_UPDATE", ">>>> Receive message from server OK");
            //parse message.
            messageProcess.prcocessRecvMsg((uint32*)recvBuf);
            msgType = *((uint32*)recvBuf);
            if( (msgType & 0x1000) == 0x0000)
            {
                diffRptMsg_t* updateMsgPtr = messageProcess.getUpdateRptMsg();
                    
                int pduIdx;

                switch(msgType)
                {
                case STATUS_UPDATE_RPT_MSG:
                    
                    for(pduIdx = 0;pduIdx < updateMsgPtr->msgHeader.numPDUs;pduIdx++)
                    {
                        int8 tag = updateMsgPtr->payloadHeader.tlvArray[pduIdx].tag;
                        int16 value = updateMsgPtr->payloadHeader.tlvArray[pduIdx].value;
                        if((1 == tag) && (3 == value))
                        {
                            // delete furniture
                            database_gp->resetFurniture();
                        }
                    }
                    break;
                case UPDATE_REQ_MSG:
                    {
                        for(pduIdx = 0;pduIdx < updateMsgPtr->msgHeader.numPDUs;pduIdx++)
                        {
                            int pduLen;
                            uint32 type = (updateMsgPtr->payloadHeader.pduHeader[pduIdx].operate << 16) + updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduType;
                            if(pduIdx == (updateMsgPtr->msgHeader.numPDUs - 1))
                            {
                                pduLen = updateMsgPtr->msgHeader.length - updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
                            }
                            else
                            {
                                pduLen = updateMsgPtr->payloadHeader.pduHeader[pduIdx+1].pduOffset - updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
                            }
                            switch(type)
                            {
                                case ADD_NEW_SEGMENT:// add a new segment data
                                    database_gpFinal->addSegmentTlv(updateMsgPtr->payload + updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                    break;
                                case ADD_NEW_FURNITURE:
                                    // fall through
                                case UPDATE_FURNITURE:
                                    database_gp->addFurnitureTlv(updateMsgPtr->payload + updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                    break;
                                case ADD_ALL_FURNITURE:
                                    database_gp->addFurnitureTlv(updateMsgPtr->payload + updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                    break;
                                case ADD_ALL_VECTORLIST:
                                    // add a vector list for specified segment.
                                    database_gpFinal->addAllVectorsInSegTlv(updateMsgPtr->payload + updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                    break;
                                case REDUCE_ONE_FURNITURE:
                                    database_gp->reduceFurnitureTlv(updateMsgPtr->payload + updateMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                    break;
                            }
                        }
                    }
                    break;
                }
            }
        }

    }
    return 0;
}