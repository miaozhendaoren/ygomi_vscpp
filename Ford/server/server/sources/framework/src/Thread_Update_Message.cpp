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
#include <sstream> // for debugging
#include "appInitCommon.h"
#include "databaseServer.h" // databaseServer
#include "LogInfo.h"

void onTimer(diffRptMsg_t* updateMsgPtr)
{

}

unsigned int __stdcall Thread_Update_Message(void *data)
{
	fd_set write_fds;
	fd_set exception_fds;

    while(1)
    {
        uint32 vehicleId;      
        //wait for dataBase access thread to get modificaiton signal
        WaitForSingleObject(g_readySema_readDb,INFINITE);
        if(!databaseQueue_gp->empty())
        {
						//init the read fds and exception_fds to zeros
		FD_ZERO( &write_fds);
		FD_ZERO( &exception_fds);
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		messageProcessClass updateMessage;
		diffRptMsg_t* updateMsgPtr;
        databaseQueue_gp->front(&updateMessage);
        databaseQueue_gp->pop();
        updateMsgPtr =  updateMessage.getUpdateRptMsg();
        updateMessage.getVehicleIDInMsg((uint32*)&updateMessage,&vehicleId);

		if(clientList.size() != 0)
		{
			list<sockInfo_t>::iterator clientListIdx1 = clientList.begin();
			while(clientListIdx1 != clientList.end())
			{
				FD_SET(clientListIdx1->sockClient, &write_fds);
				FD_SET(clientListIdx1->sockClient, &exception_fds);
				++clientListIdx1;
			}

			//delete updateMsgPtr->payload;
			int ret = select(sockClient+1, NULL, &write_fds, &exception_fds,&timeout);
			if(ret < 0)
			{
				delete updateMsgPtr->payload;
				updateMsgPtr->payload = NULL;
				continue;
			}

			list<sockInfo_t>::iterator clientListIdx = clientList.begin();
			while(clientListIdx != clientList.end())
			{
			if(FD_ISSET(clientListIdx->sockClient, &write_fds))
			{
				int* sendInt = (int*)updateMsgPtr;
				printf("send message ip: %d.%d.%d.%d, port:%d, %x,%x\n",
				clientListIdx->client.sin_addr.S_un.S_un_b.s_b1,
				clientListIdx->client.sin_addr.S_un.S_un_b.s_b2,
				clientListIdx->client.sin_addr.S_un.S_un_b.s_b3,
				clientListIdx->client.sin_addr.S_un.S_un_b.s_b4,
				clientListIdx->client.sin_port,
				sendInt[0],sendInt[1]
				);

				SOCKET sockTemp = clientListIdx->sockClient;
				
				diffMsgHeader_t*    msgTempPtr = &(updateMsgPtr->msgHeader);
				int nNetTimeout = 1000;
				setsockopt(sockTemp,SOL_SOCKET,SO_SNDTIMEO,(char*)&nNetTimeout,sizeof(int));
                {
					// send message header
					int sendRet = send(sockTemp,(char*)updateMsgPtr,msgTempPtr->headerLen,0);
					//int sendRet = sendto(sockClient,(char*)updateMsgPtr,msgTempPtr->headerLen,0,(SOCKADDR*)&toAddr,sizeof(struct sockaddr));
					if ((sendRet == INVALID_SOCKET)|| (sendRet == 0))
					{ 
						logPrintf(logLevelError_e,"UPDATE_MSG","Send message header to specified client failed!"); 
						continue;
						//delete updateMsgPtr->payload;
						//updateMsgPtr->payload = NULL;
					}
					else if(sendRet < msgTempPtr->headerLen)
					{
						logPrintf(logLevelError_e,"UPDATE_MSG","Send header message timeOut!");
						continue;
					}
					else
					{
                        std::stringstream msgStr;
                        msgStr << "<<<< Send message header to specified client OK, payload size: " << msgTempPtr->headerLen;
						logPrintf(logLevelInfo_e,"UPDATE_MSG",msgStr.str().c_str()); 

						// send payload
						if((msgTempPtr->payloadLen > 0) && (updateMsgPtr->payload != NULL))
						{
							int ret = send(sockTemp,(char*)updateMsgPtr->payload,msgTempPtr->payloadLen,0);
							//int sendRet = sendto(sockClient,(char*)updateMsgPtr->payload,msgTempPtr->payloadLen,0,(SOCKADDR*)&toAddr,sizeof(struct sockaddr));
							if ((ret == INVALID_SOCKET)|| (ret == 0))
							{ 
								logPrintf(logLevelError_e,"UPDATE_MSG","Send message playload  to specified client failed!"); 
								//delete updateMsgPtr->payload;
								//updateMsgPtr->payload = NULL;
							}
							else if(sendRet < msgTempPtr->headerLen)
							{
								logPrintf(logLevelError_e,"UPDATE_MSG","Send message timeOut!");
								continue;
							}
							else
							{
                                std::stringstream msgStr;
                                msgStr << "<<<< Send message playload to specified client OK, payload size: " << msgTempPtr->payloadLen;
								logPrintf(logLevelInfo_e,"UPDATE_MSG",msgStr.str().c_str()); 
								if(ret >=  msgTempPtr->payloadLen)
								{
									//delete updateMsgPtr->payload;
									//updateMsgPtr->payload = NULL;
								}
							}
						}
					}
				}
			}//end if(FD_ISSET(clientListIdx->sockClient, &write_fds))
			clientListIdx++;
			}//while(clientListIdx != clientList.end())
		}//end if(clientList.size() != 0)
			delete updateMsgPtr->payload;
			updateMsgPtr->payload = NULL;
		}//end if(!databaseQueue_gp->empty())
	}//end while(1)
	return 0;
}
