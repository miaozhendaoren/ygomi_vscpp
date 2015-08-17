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
#include <stdio.h>
#include <WinSock.h>
#include <iostream>
#include "appInitCommon.h"
#include "LogInfo.h"


using namespace std;

#define RECV_MAX_BYTE_NUM			(1024*10)

list<portToVehi_t> portToVehiList;
uint8 *test;
unsigned int __stdcall Thread_Receive_Message(void *data)
{
	
	uint8 recvBuff[RECV_MAX_BYTE_NUM];
	uint16 clientNum = 0;
	portToVehiList.clear();

	
	fd_set read_fds;
	fd_set exception_fds;


	while(1)
	{
		//init the read fds and exception_fds to zeros
		FD_ZERO( &read_fds);
		FD_ZERO( &exception_fds);
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		if(clientList.size() != 0)
		{
			list<sockInfo_t>::iterator clientListIdx = clientList.begin();
			while(clientListIdx != clientList.end())
			{
				FD_SET(clientListIdx->sockClient, &read_fds);
				FD_SET(clientListIdx->sockClient, &exception_fds);
				++clientListIdx;
			}

		    int ret = select(sockClient+1, &read_fds, NULL, &exception_fds,&timeout);
			if(ret < 0)
			{
				continue;
			}

			clientListIdx = clientList.begin();
			while(clientListIdx != clientList.end())
			{
				if(FD_ISSET(clientListIdx->sockClient, &read_fds))
				{
					printf("Receive message ip: %d.%d.%d.%d, port:%d\n",
						clientListIdx->client.sin_addr.S_un.S_un_b.s_b1,
						clientListIdx->client.sin_addr.S_un.S_un_b.s_b2,
						clientListIdx->client.sin_addr.S_un.S_un_b.s_b3,
						clientListIdx->client.sin_addr.S_un.S_un_b.s_b4,
						clientListIdx->client.sin_port
						);
					int   len = sizeof(struct sockaddr);
					SOCKET sockTemp = clientListIdx->sockClient;
					sockaddr_in from;
					messageProcessClass recvMsg;
					diffMsgHeader_t *recvHeader = (diffMsgHeader_t*)recvMsg.getDiffRptMsgHeader();
					//step1: receive the message header length
					int headerLen = 0;
					uint8* recvBuffP = recvBuff;
					while(headerLen < sizeof(recvHeader->headerLen))
					{
						int ret = recv(sockTemp,(char*)recvBuffP,sizeof(recvHeader->headerLen),0);
						//int ret = recvfrom(sockServer,(char*)recvBuffP,sizeof(recvHeader->headerLen),0,(SOCKADDR*)&from,&len);
						if(ret  == INVALID_SOCKET)
						{
							int errorCode = WSAGetLastError();
							logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message header length failed!");
							clientList.erase(clientListIdx++);
							goto RESTART_LABEL;
						}
						else
						{
							headerLen += ret;
							recvBuffP += ret;
						}
					}
					// receive one message header length.
					int recvAllSize = 0;
					recvHeader->headerLen = *((uint16*)recvBuff);

					//step2: receive one message header
					//recvBuffP = &recvBuff[headerLen];
					while( recvAllSize < recvHeader->headerLen - headerLen)
					{
						int ret = recv(sockTemp,(char*)recvBuffP,recvHeader->headerLen - recvAllSize - headerLen,0);
						//int ret = recvfrom(sockServer,(char*)recvBuffP,headerLen - recvAllSize,0,(SOCKADDR*)&from,&len); 
						if(ret == INVALID_SOCKET)
						{ 
							int errorCode = WSAGetLastError();
							logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message header failed!"); 
							
							clientList.erase(clientListIdx++);
							goto RESTART_LABEL;
						}
						else
						{
							recvAllSize += ret;
							recvBuffP += ret;
						}
					}
					// all the message header has been received.
					memcpy((void*)&recvMsg,(void*)recvBuff,recvHeader->headerLen);

					//step3: receive all the payload in the message
					if(recvHeader->payloadLen != 0)
					{
						int paylaodSize = 0;
						diffRptMsg_t *recvDiffMsg = (diffRptMsg_t*)recvMsg.getDiffRptMsg();
						recvDiffMsg->payload = new uint8[recvHeader->payloadLen];// malloc memory to store the payload
						if(recvDiffMsg->payload == NULL)
						{
							goto RESTART_LABEL;
						}
						//recvDiffMsg->payload = new uint8[1024*1024];
						uint8* paylaodPtr = recvDiffMsg->payload;
						test =  recvDiffMsg->payload;
						while(paylaodSize < recvHeader->payloadLen)
						{
							int ret = recv(sockTemp,(char*)paylaodPtr,recvHeader->payloadLen - paylaodSize,0);
							//int ret = recvfrom(sockServer,(char*)paylaodPtr,recvHeader->payloadLen - paylaodSize,0,(SOCKADDR*)&from,&len); 
							if(ret == INVALID_SOCKET)
							{ 
								int errorCode = WSAGetLastError();
								logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message payload failed!"); 
								delete recvDiffMsg->payload;
								recvDiffMsg->payload = NULL;
								
								clientList.erase(clientListIdx++);
								goto RESTART_LABEL;
							}
							else
							{
								paylaodSize += ret;
								paylaodPtr += ret;
							}
						}
					}
					messageQueue_gp->push(&recvMsg);
					ReleaseSemaphore(g_readySema_msgQueue,1,NULL);
				}

				if(FD_ISSET(clientListIdx->sockClient, &exception_fds))
				{
					//int shaoxin = 2;
				}

				++clientListIdx;
RESTART_LABEL:
				;
			}




		}

	}  //end while(1)
	return 0;
}