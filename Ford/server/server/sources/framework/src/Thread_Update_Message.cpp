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
#include <MMSystem.h>  //for window timer.
#include "appInitCommon.h"
#include "databaseServer.h" // databaseServer
#include "LogInfo.h"
#include "TimeStamp.h"

void onTimer(diffRptMsg_t* updateMsgPtr)
{

}

bool getUpateMsgTlv(messageProcessClass *sendMsg)
{
    bool flag = false;

    diffRptMsg_t* updateMsgPtr = sendMsg->getUpdateRptMsg();
    uint8 *payloadFromDb;
    uint32 outTlvLen;
    vector<uint32> vecPduOffset;
    vector<uint32> furPduOffset;

    flag = database_gp->getUpateMsgToTlv(&payloadFromDb, &outTlvLen, vecPduOffset, furPduOffset);

    if(true == flag)
    {
        int32 outPduIdx = 0;
        // only one vector PDU
        if(vecPduOffset.size() > 0)
        {
            updateMsgPtr->payloadHeader.pduHeader[outPduIdx].operate = addDatabase_e;
            updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduType = vectorList_e;
            updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduOffset = vecPduOffset[outPduIdx];

            ++outPduIdx;
        }

        for(int furPdnIndex = 0; furPdnIndex < furPduOffset.size(); furPdnIndex++)
        {
        
            updateMsgPtr->payloadHeader.pduHeader[outPduIdx].operate = addDatabase_e;
			updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduType = furnitureList_e;
			updateMsgPtr->payloadHeader.pduHeader[outPduIdx].pduOffset = furPduOffset[furPdnIndex];

            ++outPduIdx;
        }

        int numPDUs = vecPduOffset.size() + furPduOffset.size();

        updateMsgPtr->msgHeader.numPDUs = numPDUs;
        updateMsgPtr->msgHeader.headerLen = sizeof(updateMsgPtr->msgHeader) + numPDUs*sizeof(updateMsgPtr->payloadHeader.pduHeader[0]);
        updateMsgPtr->msgHeader.msgTypeID = UPDATE_REQ_MSG;
        updateMsgPtr->msgHeader.priority = highLevel_e;
        updateMsgPtr->msgHeader.payloadLen = outTlvLen;
        updateMsgPtr->payload = payloadFromDb;
    }

    return flag;

}

void WINAPI updateMsgTimer(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	if(1 == dwUser)
	{
		ReleaseSemaphore(g_readySema_readDb, 1 ,NULL);
	}
}

unsigned int __stdcall Thread_Update_Message(void *data)
{
	fd_set write_fds;
	fd_set exception_fds;

	list<sockInfo_t> sendclientList;	//send client address

    MMRESULT timer_id;
	timer_id = timeSetEvent((15000), 1, (LPTIMECALLBACK)updateMsgTimer, DWORD(1), TIME_PERIODIC);

    while(1)
    {
        uint32 vehicleId;      
        //wait for dataBase access thread to get modificaiton signal
		RD_ADD_TS(tsFunId_eThread_Send,1);
        WaitForSingleObject(g_readySema_readDb,INFINITE);
		RD_ADD_TS(tsFunId_eThread_Send,2);

        messageProcessClass updateMessage;

        if(!databaseQueue_gp->empty())
        {
			databaseQueue_gp->top(&updateMessage);
            databaseQueue_gp->pop();
        }
        else
        {
            if(false == getUpateMsgTlv(&updateMessage))
            {
                continue;
            }
        }
        
        {                    
        	struct timeval timeout;
            //init the read fds and exception_fds to zeros
    		FD_ZERO( &write_fds);
    		FD_ZERO( &exception_fds);
    		timeout.tv_sec = 1;
    		timeout.tv_usec = 0;
            
            diffRptMsg_t* updateMsgPtr;
            updateMsgPtr =  updateMessage.getUpdateRptMsg();
            updateMessage.getVehicleIDInMsg((uint32*)&updateMessage,&vehicleId);
    		
    		//copy receive list to send list
    		sendclientList.clear();
    		WaitForSingleObject(g_clientlistMutex,INFINITE);
    		sendclientList = clientList;
    		ReleaseMutex(g_clientlistMutex);

    		if(sendclientList.size() != 0)
    		{
    			list<sockInfo_t>::iterator clientListIdx1 = sendclientList.begin();
    			while(clientListIdx1 != sendclientList.end())
    			{		
    				FD_SET(clientListIdx1->sockClient, &write_fds); //bug
    				FD_SET(clientListIdx1->sockClient, &exception_fds);
    				++clientListIdx1;
    			}

    			//delete updateMsgPtr->payload;
    			int ret = select(sockClient+1, NULL, &write_fds, &exception_fds,&timeout);
    			if(ret <= 0)
    			{
					if(updateMsgPtr->payload!=NULL)
					{	
						delete updateMsgPtr->payload;
						updateMsgPtr->payload = NULL;
					}

    				continue;
    			}

    			list<sockInfo_t>::iterator clientListIdx = sendclientList.begin();
			    RD_ADD_TS(tsFunId_eThread_Send,3);
    			while(clientListIdx != sendclientList.end())
    			{
    				
        			//Sleep(8000); //test
        			if(FD_ISSET(clientListIdx->sockClient, &write_fds))
        			{
        				int* sendInt = (int*)updateMsgPtr;
        				//printf("send message ip: %d.%d.%d.%d, port:%d, %x,%x\n",
        				//clientListIdx->client.sin_addr.S_un.S_un_b.s_b1,
        				//clientListIdx->client.sin_addr.S_un.S_un_b.s_b2,
        				//clientListIdx->client.sin_addr.S_un.S_un_b.s_b3,
        				//clientListIdx->client.sin_addr.S_un.S_un_b.s_b4,
        				//clientListIdx->client.sin_port,
        				//sendInt[0],sendInt[1]
    				    //);
						{
							std::stringstream msgStr;
							msgStr << "send message ip:"<< (int)clientListIdx->client.sin_addr.S_un.S_un_b.s_b1
								<<"."<<(int)clientListIdx->client.sin_addr.S_un.S_un_b.s_b2<<"."
								<<(int)clientListIdx->client.sin_addr.S_un.S_un_b.s_b3<<"."
								<<(int)clientListIdx->client.sin_addr.S_un.S_un_b.s_b4<<", port:"
								<<clientListIdx->client.sin_port<<", "<<sendInt[0] << ","
								<<sendInt[1];
							logPrintf(logLevelInfo_e,"UPDATE_MSG", msgStr.str());
						}

    				SOCKET sockTemp = clientListIdx->sockClient;
    				
    				diffMsgHeader_t*    msgTempPtr = &(updateMsgPtr->msgHeader);
    				int nNetTimeout = 2000;
    				setsockopt(sockTemp,SOL_SOCKET,SO_SNDTIMEO,(char*)&nNetTimeout,sizeof(int));
                    {
    					// send message header
    					int sendRet = send(sockTemp,(char*)updateMsgPtr,msgTempPtr->headerLen,0);
    					//int sendRet = sendto(sockClient,(char*)updateMsgPtr,msgTempPtr->headerLen,0,(SOCKADDR*)&toAddr,sizeof(struct sockaddr));
    					if ((sendRet == INVALID_SOCKET)|| (sendRet == 0))
    					{ 
    						logPrintf(logLevelError_e,"UPDATE_MSG","Send message header to specified client failed!"); 
    						//continue;
    						//delete updateMsgPtr->payload;
    						//updateMsgPtr->payload = NULL;
    					}
    					else if(sendRet < msgTempPtr->headerLen)
    					{
    						logPrintf(logLevelError_e,"UPDATE_MSG","Send header message timeOut!");
    						//continue;
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
							    else if(ret < msgTempPtr->payloadLen)
    							{
    								logPrintf(logLevelError_e,"UPDATE_MSG","Send message timeOut!");
    								//continue;
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
			if(updateMsgPtr->payload!=NULL)	
			{	
				delete updateMsgPtr->payload;
				updateMsgPtr->payload = NULL;
			}
		}//end if(!databaseQueue_gp->empty())
		RD_ADD_TS(tsFunId_eThread_Send,14);
	}//end while(1)
	return 0;
}
