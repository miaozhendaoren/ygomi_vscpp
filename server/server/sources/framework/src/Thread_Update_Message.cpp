/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_Update_Message.cpp
* @brief packed the update message and send to client.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#include <gl/glut.h>
#include <Windows.h>
#include "appInitCommon.h"
#include "databaseServer.h" // databaseServer
#include "LogInfo.h"

#define TIMER_NUMBER 100
#define TIMER_START_VALUE 10
#define TIMER_INTERVAL                          0//No delay // 5000        //5s timer
#define MAX_SIZE_OF_UPDATE_DATA_BUFF            10000
#define MSG_QUEUE_MAX_SIZE                      8
#define SEND_MAX_BYTE_NUM                       10000

int sendMsgLen[TIMER_NUMBER];

uint8 sendBuff[TIMER_NUMBER][SEND_MAX_BYTE_NUM];

void onTimer(int value)
{
    int   len = sizeof(struct sockaddr);
    if(portToVehiList.size() != 0)
    {
        list<portToVehi_t>::iterator portListIdx = portToVehiList.begin();
        while(portListIdx != portToVehiList.end())
        {        
            //int vehicleIdx;
            //for(vehicleIdx = 0;vehicleIdx < portListIdx->vehiNum; ++vehicleIdx)
            {
                uint64 clientId = portListIdx->vehicleId;
                diffMsgHeader_t*    msgTempPtr = (diffMsgHeader_t*)(sendBuff[value - TIMER_START_VALUE]);
                msgTempPtr->vehicleID = clientId;

                int sendRet = sendto(sockClient,(char*)sendBuff[value - TIMER_START_VALUE],sendMsgLen[value - TIMER_START_VALUE],0,(SOCKADDR*)&(portListIdx->client),len);
                if ((sendRet == INVALID_SOCKET)|| (sendRet == 0))
                {
                    logPrintf(logLevelError_e,"UPDATE_MSG","Send message to client failed!"); 
                }
                else
                {
                    logPrintf(logLevelInfo_e,"UPDATE_MSG","<<<< Send message to client OK"); 
                }
            }
            ++portListIdx;
        }
    }
    else
    {
        list<SOCKADDR_IN>::iterator clientListIdx = clientList.begin();
        while(clientListIdx != clientList.end())
        {
            //uint64 clientId = clientListIdx->vehicleId;
            diffMsgHeader_t*    msgTempPtr = (diffMsgHeader_t*)(sendBuff[value - TIMER_START_VALUE]);
            msgTempPtr->vehicleID = 0;

            int sendRet = sendto(sockClient,(char*)sendBuff[value - TIMER_START_VALUE],sendMsgLen[value - TIMER_START_VALUE],0,(SOCKADDR*)&(*clientListIdx),len);
            if ((sendRet == INVALID_SOCKET)|| (sendRet == 0))
            {
                logPrintf(logLevelError_e,"UPDATE_MSG","Send message to client failed!"); 
            }
            else
            {
                logPrintf(logLevelInfo_e,"UPDATE_MSG","<<<< Send message to client OK"); 
            }
            ++clientListIdx;
        }
    }
}

unsigned int __stdcall Thread_Update_Message(void *data)
{
   unsigned int     counter = TIMER_START_VALUE;
    while(1)
    {
        uint64 vehicleId;
        messageProcessClass updateMessage;
        
        //wait for dataBase access thread to get modificaiton signal
        WaitForSingleObject(g_readySema_readDb,INFINITE);
        if(!databaseQueue_gp->empty())
        {
            int idx;
            diffRptMsg_t* updateMsgPtr;
            databaseQueue_gp->front(&updateMessage);
            databaseQueue_gp->pop();
            updateMsgPtr =  updateMessage.getUpdateRptMsg();
            updateMessage.getVehicleIDInMsg((uint32*)&updateMessage,&vehicleId);
            sendMsgLen[counter - TIMER_START_VALUE] = 0;
            bool timeFlag = false;
            switch(updateMsgPtr->msgHeader.msgTypeID)
            {
                case STATUS_UPDATE_RPT_MSG:
                {
                    updateMessage.setMsgHeader((uint32*)updateMsgPtr,STATUS_UPDATE_RPT_MSG,vehicleId,lowLevel_e,updateMsgPtr->msgHeader.length,updateMsgPtr->msgHeader.numPDUs);
                    updateMessage.packedStatusRptMsg((uint32*)sendBuff[counter - TIMER_START_VALUE]);
                    sendMsgLen[counter - TIMER_START_VALUE] =  sendMsgLen[counter - TIMER_START_VALUE] + sizeof(diffMsgHeader_t) + updateMsgPtr->msgHeader.numPDUs*4;
                    timeFlag = true;
                }
                break;
                case DIFF_RPT_MSG:
                {
                    updateMessage.setMsgHeader((uint32*)updateMsgPtr,UPDATE_REQ_MSG,vehicleId,lowLevel_e,updateMsgPtr->msgHeader.length,updateMsgPtr->msgHeader.numPDUs);

                    updateMessage.packedUpdateRptMsg((uint32*)sendBuff[counter - TIMER_START_VALUE]);
                    for(idx = 0;idx < updateMsgPtr->msgHeader.numPDUs;idx++)
                    {
                        diffRptPduHeader_t pduMsg = updateMsgPtr->payloadHeader.pduHeader[idx];
                        sendMsgLen[counter - TIMER_START_VALUE] += 8;
                        switch(pduMsg.pduType)
                        {
                            case furnitureElement_e:
                            case furnitureSign_e:
                                //updateMessage.setUpdateRptPduFntHeader(pduCounter,pduMsg.pduDiffTypeHeader.fntHeader.segMentID,pduMsgPtr.pduDiffTypeHeader.fntHeader.furnitureID);
                                sendMsgLen[counter - TIMER_START_VALUE] += 8;
                                break;
                            case vectorElement_e:
                                //updateMessage.setUpdateRptPduVectorHeader(pduCounter,pduMsgPtr.pduDiffTypeHeader.vectorHeader.segMentID,pduMsgPtr.pduDiffTypeHeader.vectorHeader.vectorID);
                                sendMsgLen[counter - TIMER_START_VALUE] += 8;
                                break;
                            case dynamicData_e:
                                break;
                            //default:
                                //printf("PDU type is:%d\n",pduMsg.pduType);
                        }
                    }
                    sendMsgLen[counter - TIMER_START_VALUE] =  sendMsgLen[counter - TIMER_START_VALUE] + sizeof(diffMsgHeader_t) + updateMsgPtr->msgHeader.length;
                    timeFlag = true;
                }
                break;
            case UPDATE_REQ_MSG:
                char sendTempBuff[SEND_MAX_BYTE_NUM];
                int sendLen = 8;
                updateMessage.packedUpdateRptMsg((uint32*)sendTempBuff);
                diffRptPduHeader_t pduMsg = updateMsgPtr->payloadHeader.pduHeader[0];
                sendLen +=  (sizeof(diffMsgHeader_t) + updateMsgPtr->msgHeader.length + updateMsgPtr->msgHeader.numPDUs*8);//8 = sizeof(pduType + operate + pduOffset)
                list<portToVehi_t>::iterator portListIdx = portToVehiList.begin();
                SOCKADDR_IN toAddr;
                bool existVehiIdFlag = false;
                while(portListIdx != portToVehiList.end())
                {
                    if(portListIdx->vehicleId == vehicleId)
                    {
                        toAddr = portListIdx->client;
                        existVehiIdFlag = true;
                        break;
                    }
                }
                if(!existVehiIdFlag)
                {
                    logPrintf(logLevelError_e,"UPDATE_MSG","not find this vehicle port!"); 
                }
                else
                {
                    int sendRet = sendto(sockClient,sendTempBuff,sendLen,0,(SOCKADDR*)&toAddr,sizeof(struct sockaddr));
                    if ((sendRet == INVALID_SOCKET)|| (sendRet == 0))
                    {
                        logPrintf(logLevelError_e,"UPDATE_MSG","Send message to client failed!"); 
                    }
                    else
                    {
                        logPrintf(logLevelInfo_e,"UPDATE_MSG","<<<< Send message to client OK"); 
                    }        
                }
                break;
            }
            // start a time to delay the update message for demo
            if(timeFlag)
            {
                onTimer(counter++);
				//glutTimerFunc((unsigned int)(TIMER_INTERVAL),&onTimer,counter++);
                if(counter >= TIMER_NUMBER)
                {
                    counter  = TIMER_START_VALUE;
                }
            }
        }
    }
    return 0;
}
