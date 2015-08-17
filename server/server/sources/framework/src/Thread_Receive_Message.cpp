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
#include <Windows.h>
#include "appInitCommon.h"
#include "LogInfo.h"

#define RECV_MAX_BYTE_NUM			10000


//struct portToVehi_t
//{
//	u_short portId;
//	uint64_t vehicleId;
//};

list<portToVehi_t> portToVehiList;

unsigned int __stdcall Thread_Receive_Message(void *data)
{
	
	uint8 recvBuff[RECV_MAX_BYTE_NUM];
	messageProcessClass msgRecv;
	uint16 clientNum = 0;
	portToVehiList.clear();
	while(1)
	{
		int   len = sizeof(struct sockaddr);
		// receive message from clients
		sockaddr_in from;
		int recvRet = recvfrom(sockServer,(char*)recvBuff,RECV_MAX_BYTE_NUM,0,(SOCKADDR*)&from,&len); 
		if ((recvRet == INVALID_SOCKET)  || (recvRet == 0))
		{  
			logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message from client failed!"); 
		}
		else
		{
			int32 msgType = *((uint32*)recvBuff);
			logPrintf(logLevelInfo_e,"RECEIVE_MSG",">>>> Receive message from client OK");
			// process received message
			msgRecv.prcocessRecvMsg((uint32*)recvBuff);
			messageQueue_gp->push(&msgRecv);
			uint64 clientId;
			msgRecv.getVehicleIDInMsg((uint32*)&msgRecv,&clientId);
			list<portToVehi_t>::iterator portToVehiListIdx = portToVehiList.begin();
			bool ipExistFlag = false;
			portToVehi_t clientIpAndPort;
			while(portToVehiListIdx != portToVehiList.end())
			{
				if(portToVehiListIdx->client.sin_addr.S_un.S_addr == ((SOCKADDR_IN)from).sin_addr.S_un.S_addr)
				//ip has been exist in tab
				{
					//clientIpAndPort.vehicleId = clientId;
					ipExistFlag = true;
					break;
				}
				++portToVehiListIdx;
			}					
			if(!ipExistFlag)
			{
				
				clientIpAndPort.client.sin_addr.S_un.S_addr = ((SOCKADDR_IN)from).sin_addr.S_un.S_addr;	
				list<SOCKADDR_IN>::iterator clientListIdx = clientList.begin();
				bool existFlag = false;
				while(clientListIdx != clientList.end())
				{
					if(clientListIdx->sin_addr.S_un.S_addr == clientIpAndPort.client.sin_addr.S_un.S_addr)
					{
						existFlag = true;
						clientIpAndPort.vehicleId = clientId;
						clientIpAndPort.client = *clientListIdx;
						portToVehiList.push_back(clientIpAndPort);
						break;
					}
					++clientListIdx;
				}
				if(!existFlag)
				{
					logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message unkown IP!"); 
				}
			
			}

			ReleaseSemaphore(g_readySema_msgQueue,1,NULL);
		}
	}
	return 0;
}