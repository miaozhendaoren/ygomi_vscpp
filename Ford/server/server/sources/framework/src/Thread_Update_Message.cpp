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
    while(1)
    {
        uint32 vehicleId;      
        //wait for dataBase access thread to get modificaiton signal
        WaitForSingleObject(g_readySema_readDb,INFINITE);
        if(!databaseQueue_gp->empty())
        {
			messageProcessClass updateMessage;
			diffRptMsg_t* updateMsgPtr;
            databaseQueue_gp->front(&updateMessage);
            databaseQueue_gp->pop();
            updateMsgPtr =  updateMessage.getUpdateRptMsg();
            updateMessage.getVehicleIDInMsg((uint32*)&updateMessage,&vehicleId);
			//delete updateMsgPtr->payload;
			
			list<sockInfo_t>::iterator clientListIdx = clientList.begin();
			while(clientListIdx != clientList.end())
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
				clientListIdx++;
				diffMsgHeader_t*    msgTempPtr = &(updateMsgPtr->msgHeader);

                {
					// send message header
					int sendRet = send(sockTemp,(char*)updateMsgPtr,msgTempPtr->headerLen,0);
					//int sendRet = sendto(sockClient,(char*)updateMsgPtr,msgTempPtr->headerLen,0,(SOCKADDR*)&toAddr,sizeof(struct sockaddr));
					if ((sendRet == INVALID_SOCKET)|| (sendRet == 0))
					{ 
						logPrintf(logLevelError_e,"UPDATE_MSG","Send message header to specified client failed!"); 
						//delete updateMsgPtr->payload;
						//updateMsgPtr->payload = NULL;
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
			}//while(clientListIdx != clientList.end())
			delete updateMsgPtr->payload;
			updateMsgPtr->payload = NULL;
		}//end if(!databaseQueue_gp->empty())
	}//end while(1)
	return 0;
}
