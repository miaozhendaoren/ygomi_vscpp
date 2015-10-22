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
#include <sstream> // for debugging
#include "TimeStamp.h"

using namespace std;


list<portToVehi_t> portToVehiList;
uint8 *test;
unsigned int __stdcall Thread_Receive_Message(void *data)
{
	
	uint16 clientNum = 0;
	portToVehiList.clear();

	
	fd_set read_fds;
	fd_set exception_fds;

    
#if SERVER_LOG_DIFF_MSG==1
    {
        // Clear existing messages.bin file
        std::stringstream fileName;
        fileName << "log/messages.bin";
        FILE *fpOut = fopen(fileName.str().c_str(), "wb");
        fclose(fpOut);
    }
#endif

	while(1)
	{
		//init the read fds and exception_fds to zeros
		FD_ZERO( &read_fds);
		FD_ZERO( &exception_fds);
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		//RD_ADD_TS(tsFunId_eThread_Recevie,1);
		if(clientList.size() != 0)
		{
			list<sockInfo_t>::iterator clientListIdx = clientList.begin();
			while(clientListIdx != clientList.end())
			{
				FD_SET(clientListIdx->sockClient, &read_fds);
				FD_SET(clientListIdx->sockClient, &exception_fds);
				++clientListIdx;
			}

			//RD_ADD_TS(tsFunId_eThread_Recevie,2);
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
					RD_ADD_TS(tsFunId_eThread_Recevie,3);
					int   len = sizeof(struct sockaddr);
					SOCKET sockTemp = clientListIdx->sockClient;
					sockaddr_in from;
					messageProcessClass recvMsg;
					diffRptMsg_t *recvHeader = recvMsg.getDiffRptMsg();
					//step1: receive the message header length
					int headerLen = 0;
					uint8* recvBuffP = (uint8*)recvHeader;
					while(headerLen < sizeof(recvHeader->msgHeader.headerLen))
					{
						int ret = recv(sockTemp,(char*)recvBuffP,sizeof(recvHeader->msgHeader.headerLen),0);
						//int ret = recvfrom(sockServer,(char*)recvBuffP,sizeof(recvHeader->headerLen),0,(SOCKADDR*)&from,&len);
						//if(ret  == INVALID_SOCKET)
						if(ret  == INVALID_SOCKET || ret == 0)
						{
							int errorCode = WSAGetLastError();
							if(ret == 0) //qiuheng
							{
								logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message header length:the client side socket closed.");
							}	
							else
							{
								logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message header length failed!");
							}
							//shutdown(clientListIdx->sockClient,2);
							
							//close socket file
							struct linger so_linger;
							so_linger.l_onoff = 1;
							so_linger.l_linger = 300;
							setsockopt(clientListIdx->sockClient,SOL_SOCKET,SO_LINGER,(char*)&so_linger,sizeof(so_linger));
							closesocket(clientListIdx->sockClient);				
							WaitForSingleObject(g_clientlistMutex,INFINITE);
							clientList.erase(clientListIdx++);
							ReleaseMutex(g_clientlistMutex);
							
							goto RESTART_LABEL;
						}
						else
						{
							headerLen += ret;
							recvBuffP += ret;
							//test
							//logPrintf(logLevelInfo_e,"RECEIVE_MSG","Receive message header length!");
						}
					}
					// receive one message header length.
					int recvAllSize = 0;

					//step2: receive one message header
					//recvBuffP = &recvBuff[headerLen];
					while( recvAllSize < recvHeader->msgHeader.headerLen - sizeof(recvHeader->msgHeader.headerLen))
					{
						int ret = recv(sockTemp,(char*)recvBuffP,recvHeader->msgHeader.headerLen - recvAllSize - sizeof(recvHeader->msgHeader.headerLen),0);
						//int ret = recvfrom(sockServer,(char*)recvBuffP,headerLen - recvAllSize,0,(SOCKADDR*)&from,&len); 
						//if(ret  == INVALID_SOCKET)
						if(ret  == INVALID_SOCKET || ret == 0)
						{ 
							int errorCode = WSAGetLastError();
							if(ret == 0) 
							{
								logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message header:the client side socket closed."); 
							}
							else
							{
								logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message header failed!"); 
							}

							struct linger so_linger;
							so_linger.l_onoff = 1;
							so_linger.l_linger = 300;
							setsockopt(clientListIdx->sockClient,SOL_SOCKET,SO_LINGER,(char*)&so_linger,sizeof(so_linger));
							closesocket(clientListIdx->sockClient);
							WaitForSingleObject(g_clientlistMutex,INFINITE);
							clientList.erase(clientListIdx++);
							ReleaseMutex(g_clientlistMutex);
							goto RESTART_LABEL;
						}
						else
						{
							
							recvAllSize += ret;
							recvBuffP += ret;
							//test
							//logPrintf(logLevelInfo_e,"RECEIVE_MSG","Receive message header!");
						}
					}
					// all the message header has been received.
					//memcpy((void*)&recvMsg,(void*)recvBuff,recvHeader->headerLen);

					//step3: receive all the payload in the message
					if(recvHeader->msgHeader.payloadLen != 0)
					{
						int paylaodSize = 0;
						diffRptMsg_t *recvDiffMsg = (diffRptMsg_t*)recvMsg.getDiffRptMsg();
						recvDiffMsg->payload = new uint8[recvHeader->msgHeader.payloadLen];// malloc memory to store the payload
						if(recvDiffMsg->payload == NULL)
						{
							goto RESTART_LABEL;
						}
						//recvDiffMsg->payload = new uint8[1024*1024];
						uint8* paylaodPtr = recvDiffMsg->payload;
						test =  recvDiffMsg->payload;
						while(paylaodSize < recvHeader->msgHeader.payloadLen)
						{
							int ret = recv(sockTemp,(char*)paylaodPtr,recvHeader->msgHeader.payloadLen - paylaodSize,0);
							//int ret = recvfrom(sockServer,(char*)paylaodPtr,recvHeader->payloadLen - paylaodSize,0,(SOCKADDR*)&from,&len); 
							//if(ret  == INVALID_SOCKET)
							if(ret  == INVALID_SOCKET || ret == 0)
							{ 
								int errorCode = WSAGetLastError();
								if(ret == 0) 
								{
									logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message payload:the client side socket closed.");
								}
								else
								{
									logPrintf(logLevelError_e,"RECEIVE_MSG","Receive message payload failed!"); 
								}

								delete recvDiffMsg->payload;
								recvDiffMsg->payload = NULL;
								
								struct linger so_linger;
								so_linger.l_onoff = 1;
								so_linger.l_linger = 300;
								setsockopt(clientListIdx->sockClient,SOL_SOCKET,SO_LINGER,(char*)&so_linger,sizeof(so_linger));
								closesocket(clientListIdx->sockClient);
								WaitForSingleObject(g_clientlistMutex,INFINITE);
								clientList.erase(clientListIdx++);
								ReleaseMutex(g_clientlistMutex);
								goto RESTART_LABEL;
							}
							else
							{
								paylaodSize += ret;
								paylaodPtr += ret;
								//test
								//logPrintf(logLevelInfo_e,"RECEIVE_MSG","Receive message payload!");
							
							}
						}
					}
					

#if SERVER_LOG_DIFF_MSG==1
                    {
                        std::stringstream fileName;
                        fileName << "log/messages.bin";
                        FILE *fpOut = fopen(fileName.str().c_str(), "ab");

                        diffRptMsg_t *diffMsg = recvMsg.getDiffRptMsg();

                        // All message header
                        fwrite(diffMsg, diffMsg->msgHeader.headerLen, 1, fpOut);

                        // All message payload
                        fwrite(diffMsg->payload, diffMsg->msgHeader.payloadLen, 1, fpOut);

                        fclose(fpOut);
                    }
#endif
#if SERVER_PLAY_BACK_MODE==0
					RD_ADD_TS(tsFunId_eThread_Recevie,10);
					messageQueue_gp->push(&recvMsg);
					ReleaseSemaphore(g_readySema_msgQueue,1,NULL);
					RD_ADD_TS(tsFunId_eThread_Recevie,11);
#endif

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
		//RD_ADD_TS(tsFunId_eThread_Recevie,14);
	}  //end while(1)

	//test
	logPrintf(logLevelInfo_e,"RECEIVE_MSG","Receive message process exit!");
	return 0;
}