/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_Receive_Message.cpp
* @brief this thread to receive the messages from socket.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#include "appInitCommon.h"
#include "LogInfo.h"
#include "Thread_Receive_Message.h"

unsigned int __stdcall Thread_Receive_Message(void *data)
{
	
	uint8 recvBuff[RECV_MAX_BYTE_NUM];
	messageProcessClass msgRecv;
	while(1)
	{
		int   len = sizeof(struct sockaddr);
		// receive message from clients
		sockaddr_in from;
		int recvRet = recvfrom(sockServer,(char*)recvBuff,RECV_MAX_BYTE_NUM,0,(SOCKADDR*)&from,&len); 
		if ((recvRet == INVALID_SOCKET)  || (recvRet == 0))
		{  
			logPrintf(logLevelError_e,"RECEIVE_MESSAGE","Receive message from client failed!"); 
		}
		else
		{
			int32 msgType = *((uint32*)recvBuff);
			char charBuff[10];
			logPrintf(logLevelInfo_e,"RECEIVE_MESSAGE","Receive message from client OK");
			// process received message
			msgRecv.prcocessRecvMsg((uint32*)recvBuff);
			messageQueue_gp->push(&msgRecv);
			ReleaseSemaphore(g_readySema_msgQueue,1,NULL);
		}
	}
	return 0;
}