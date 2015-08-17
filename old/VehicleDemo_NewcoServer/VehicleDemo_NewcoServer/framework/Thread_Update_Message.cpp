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
#include "appInitCommon.h"
#include "LogInfo.h"
#include "Thread_Update_Message.h"



int sendMsgLen[TIMER_NUMBER];

uint8 sendBuff[TIMER_NUMBER][SEND_MAX_BYTE_NUM];

void onTimer(int value)
{
	int   len = sizeof(struct sockaddr);
	int sendRet = sendto(sockClient,(char*)sendBuff[value - TIMER_START_VALUE],sendMsgLen[value - TIMER_START_VALUE],0,(SOCKADDR*)&client,len);
	if ((sendRet == INVALID_SOCKET)|| (sendRet == 0))
	{
		logPrintf(logLevelError_e,"UPDATE_MESSAGE","Send message to client failed!"); 
	}
	else
	{
		logPrintf(logLevelInfo_e,"UPDATE_MESSAGE","Send message to client OK"); 
	}
}

unsigned int __stdcall Thread_Update_Message(void *data)
{
   unsigned int	 counter = TIMER_START_VALUE;
	while(1)
	{
		uint64 vehicleId;
		messageProcessClass updateMessage;
		
		//wait for dataBase access thread to get modificaiton signal
        WaitForSingleObject(g_readySema_readDb,INFINITE);
		if(!databaseQueue_gp->empty())
		{
			int idx;
			int sendRet;
			diffRptMsg_t* updateMsgPtr;
			databaseQueue_gp->front(&updateMessage);
			databaseQueue_gp->pop();
			updateMsgPtr =  updateMessage.getUpdateRptMsg();
			updateMessage.getVehicleIDInMsg((uint32*)&updateMessage,&vehicleId);
			sendMsgLen[counter - TIMER_START_VALUE] = 0;

			switch(updateMsgPtr->msgHeader.msgTypeID)
			{
				case 0x0000:
				{
					updateMessage.setMsgHeader((uint32*)updateMsgPtr,0x0000,vehicleId,lowLevel_e,updateMsgPtr->msgHeader.length,updateMsgPtr->msgHeader.numPDUs);
					updateMessage.packedStatusRptMsg((uint32*)sendBuff[counter - TIMER_START_VALUE]);
					sendMsgLen[counter - TIMER_START_VALUE] =  sendMsgLen[counter - TIMER_START_VALUE] + sizeof(diffMsgHeader_t) + updateMsgPtr->msgHeader.numPDUs*4;
				}
				break;
				case 0x0001:
				{
					updateMessage.setMsgHeader((uint32*)updateMsgPtr,0x0002,vehicleId,lowLevel_e,updateMsgPtr->msgHeader.length,updateMsgPtr->msgHeader.numPDUs);
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
				}
				break;
			}
			// start a time to delay the update message for demo
			glutTimerFunc((unsigned int)(TIMER_INTERVAL),&onTimer,counter++);
			if(counter >= TIMER_NUMBER)
			{
				counter  = TIMER_START_VALUE;
			}
		}
	}
	return 0;
}
